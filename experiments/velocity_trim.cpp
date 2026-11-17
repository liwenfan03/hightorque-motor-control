// EXPERIMENT (not part of the library / motor_tool; nothing there was changed): velocity control with a
// host outer correction ("trim") so the REAL speed matches the target.
//
// Uses run_control_pipelined() (pipeline.hpp) in velocity mode exactly as `motor_tool control --mode
// velocity` does (1 kHz, 24 B frames, host ramp, braking before the limits, all safety checks, Ctrl-C =
// brake + stop).  Only the setpoint is changed, through the trajectory callback:
//
//   1. speed from POSITION: v_pos = (newest position - position W s earlier) / time between them.
//      (Measured 2026-11-12, square +-30 deg/s, no trim: both motors' velocity loops hold their OWN reading at
//      the target on average, but the speed from position differs: 4438 ~7% (+) / ~5% (-) slower, 5047 ~7%
//      faster.  Speed from position is quantized to 0.088 deg / window (1.8 deg/s at 50 ms).)
//   2. compare with the target the motor was asked to run (average over the same window, shifted by
//      --trim-lag for the motor's response), only while the command is steady: not limited by the host
//      ramp / braking, same sign, |target| >= --trim-min.
//   3. per motor and per DIRECTION (positive / negative separately): corr += ki * (target - v_pos) * dt,
//      |corr| <= --trim-max.  Sent: target + corr (corr fades to 0 below --trim-min, so 0 stays 0).
//
// Build (no change to CMakeLists.txt; rebuild after the library changed):
//   cmake --build build && c++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Iexamples experiments/velocity_trim.cpp \
//       build/libhightorque.a -o build/velocity_trim
// Run (deg/s like motor_tool control --mode velocity; --trim-ki 0 = same run without correction):
//   ./build/velocity_trim --id 1,2 --profile square --amp 30 --freq 0.25 --duration 20 --rt
// Config: config/control.conf, section [velocity] / [common] (vel-ramp, max-tqe, safe-vel, brake, rate, ...).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cli_common.hpp"
#include "control_config.hpp"
#include "hightorque/control.hpp"
#include "hightorque/motor.hpp"
#include "hightorque/pipeline.hpp"
#include "hightorque/zero.hpp"

using namespace hightorque;

namespace {

std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  size_t a = 0;
  while (a <= s.size() && !s.empty()) {
    const size_t b = s.find(',', a);
    out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

double envelope(double t, double ramp, double duration) {
  if (ramp <= 0) return 1.0;
  double g = std::min(t / ramp, 1.0);
  if (duration > 0) g = std::min(g, (duration - t) / ramp);
  return std::clamp(g, 0.0, 1.0);
}

int sgn(double v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

struct Stat {  // per motor, direction, half of the run (deg/s)
  int n = 0;
  double goal = 0, vpos = 0, vrep = 0, err2 = 0;
};

struct Trim {
  // velocity from position
  struct P { double t, pos, vrep; };
  std::deque<P> win;
  // what the motor was asked to run (command - correction), and whether the host limited it
  struct C { double t, goal; bool limited; };
  std::deque<C> hist;
  double last_limited = -1e9;
  double corr[2] = {0, 0};  // [0] negative direction, [1] positive (turns/s)
  double vpos = 0, vrep_avg = 0;
  bool vpos_ok = false;
  Stat st[2][2];  // [half][direction]
};

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  if (!args.positional().empty()) {
    std::cerr << "unexpected argument '" << args.positional()[0] << "' (options need two dashes, e.g. --id 1,2)\n";
    return 2;
  }
  // ---- config + command line (the command line wins) ----
  cli::ControlConfig cfg;
  {
    const std::string path = args.str("config", "config/control.conf");
    std::ifstream probe(path);
    if (probe || args.has("config")) {
      try {
        cfg = cli::ControlConfig(path);
      } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 2;
      }
      std::printf("config: %s [velocity] / [common]\n", path.c_str());
    } else {
      std::printf("config: none (%s not found; run from the project folder), built-in defaults\n", path.c_str());
    }
  }
  const auto ids = split_csv(args.str("id", "1"));
  const size_t n = ids.size();
  std::vector<int> id_num(n);
  for (size_t k = 0; k < n; k++) id_num[k] = std::atoi(ids[k].c_str());
  auto per_joint = [&](const std::string& key, double def, std::vector<double>& out) {
    out.assign(n, def);
    if (args.has(key)) {
      const auto v = split_csv(args.str(key));
      if (v.size() != 1 && v.size() != n) {
        std::cerr << "--" << key << ": one value for all motors or one per --id entry\n";
        return false;
      }
      for (size_t k = 0; k < n; k++) out[k] = std::atof((v.size() == 1 ? v[0] : v[k]).c_str());
      return true;
    }
    for (size_t k = 0; k < n; k++) {
      if (const auto v = cfg.get("velocity", id_num[k], key)) out[k] = std::atof(v->c_str());
    }
    return true;
  };
  auto scalar = [&](const std::string& key, double def) {
    if (args.has(key)) return args.num(key, def);
    if (const auto v = cfg.get("velocity", key)) return std::atof(v->c_str());
    return def;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> bias, amp, freq, safe_vel, vel_ramp, max_tqe, ki, tmax;
  if (!per_joint("bias", 0.0, bias) || !per_joint("amp", 0.0, amp) || !per_joint("freq", 0.5, freq) ||
      !per_joint("safe-vel", 0.5, safe_vel) || !per_joint("vel-ramp", 1.0, vel_ramp) || !per_joint("max-tqe", nan, max_tqe)) {
    return 2;
  }
  // trim options: command line only (the config rejects unknown keys)
  auto trim_joint = [&](const std::string& key, double def, std::vector<double>& out) {
    out.assign(n, def);
    if (!args.has(key)) return true;
    const auto v = split_csv(args.str(key));
    if (v.size() != 1 && v.size() != n) {
      std::cerr << "--" << key << ": one value for all motors or one per --id entry\n";
      return false;
    }
    for (size_t k = 0; k < n; k++) out[k] = std::atof((v.size() == 1 ? v[0] : v[k]).c_str());
    return true;
  };
  if (!trim_joint("trim-ki", 2.0, ki) || !trim_joint("trim-max", 10.0, tmax)) return 2;  // 1/s, deg/s
  const double win_s = args.num("trim-window", 0.05);
  const double lag_s = args.num("trim-lag", 0.02);
  const double settle_s = args.num("trim-settle", 0.1);
  const double vmin = args.num("trim-min", 3.6) / 360.0;  // deg/s -> turns/s
  for (auto& v : tmax) v /= 360.0;
  if (win_s < 0.005 || lag_s < 0 || settle_s < 0 || vmin <= 0) {
    std::cerr << "--trim-window >= 0.005 s, --trim-lag / --trim-settle >= 0, --trim-min > 0\n";
    return 2;
  }
  const std::string wave = args.has("profile") ? args.str("profile") : cfg.get("velocity", "profile").value_or("hold");
  if (wave != "hold" && wave != "sine" && wave != "square") {
    std::cerr << "--profile: hold, sine or square\n";
    return 2;
  }

  PipelineOptions po;
  po.rate_hz = scalar("rate", 1000.0);
  po.duration_s = scalar("duration", 10.0);
  po.max_stale = static_cast<int>(scalar("max-stale", po.max_stale));
  po.max_outstanding = static_cast<int>(scalar("max-outstanding", po.max_outstanding));
  po.warmup_cycles = static_cast<int>(scalar("warmup", po.warmup_cycles));
  po.brake_s = scalar("brake", po.brake_s);
  const double ramp = scalar("ramp", 1.0);
  const double dt = 1.0 / po.rate_hz;
  if (po.duration_s <= 0) {
    std::cerr << "--duration > 0 (the report compares the first and the second half)\n";
    return 2;
  }

  try {
    Fdcanusb bus(args.str("port"));
    cli::configure_bus(bus, args);
    const auto models = split_csv(args.str("model"));
    std::vector<std::unique_ptr<Motor>> owned;
    std::vector<ControlJoint> joints(n);
    for (size_t k = 0; k < n; k++) {
      Motor::Options o;
      std::string m = models.empty() ? "" : models.size() == 1 ? models[0] : models[k];
      if (m.empty()) m = cfg.model(id_num[k]).value_or("");
      o.model = cli::parse_model(m);
      o.timeout = std::chrono::microseconds(static_cast<int64_t>(std::max(1000.0, 0.9e6 / po.rate_hz)));
      owned.push_back(std::make_unique<Motor>(bus, static_cast<uint8_t>(id_num[k]), o));
      ControlJoint& j = joints[k];
      j.motor = owned.back().get();
      j.mode = ControlMode::Velocity;
      j.safe_vel = safe_vel[k];
      j.vel_ramp = vel_ramp[k];
      j.regs.max_torque = max_tqe[k];
    }

    std::vector<Trim> tr(n);
    // target (turns/s) of joint i at time t, without correction
    auto target = [&](size_t i, double t) {
      const double g = envelope(t, ramp, po.duration_s), ph = 2.0 * M_PI * freq[i] * t;
      double v = bias[i] / 360.0;
      if (wave == "sine") v += g * amp[i] / 360.0 * std::sin(ph);
      if (wave == "square") v += g * amp[i] / 360.0 * (std::sin(ph) >= 0 ? 1.0 : -1.0);
      return v;
    };
    auto corr_for = [&](size_t i, double v) {
      if (v == 0) return 0.0;
      return tr[i].corr[v > 0 ? 1 : 0] * std::min(1.0, std::fabs(v) / vmin);
    };
    std::vector<double> applied(n, 0.0), last_target(n, 0.0);
    const ControlTrajectory traj = [&](size_t i, double t) {
      ControlSetpoint sp;
      last_target[i] = target(i, t);
      applied[i] = corr_for(i, last_target[i]);
      sp.velocity = last_target[i] + applied[i];
      return sp;
    };

    std::printf("velocity control + host trim (EXPERIMENT), loop %.0f Hz, %.0f s, profile %s (ramp %.1f s)\n", po.rate_hz,
                po.duration_s, wave.c_str(), ramp);
    for (size_t k = 0; k < n; k++) {
      std::printf("  ID %d (%s): bias %+.1f deg/s", id_num[k], motor_model_name(joints[k].motor->options().model), bias[k]);
      if (wave != "hold") std::printf(", amp %.1f deg/s at %.2f Hz", amp[k], freq[k]);
      std::printf(", host ramp %.2f r/s^2, max-tqe %.2f Nm, safe-vel %.2f r/s | trim ki %.2f /s, max +-%.1f deg/s%s\n",
                  vel_ramp[k], max_tqe[k], safe_vel[k], ki[k], tmax[k] * 360.0, ki[k] == 0 ? " (OFF: measure only)" : "");
    }
    std::printf("  speed from position over %.0f ms, motor lag %.0f ms, settle %.0f ms, only while |target| >= %.1f deg/s\n",
                win_s * 1e3, lag_s * 1e3, settle_s * 1e3, vmin * 360.0);
    std::fflush(stdout);
    cli::install_ctrl_c();
    if (args.has("rt")) cli::make_realtime(po.rate_hz);

    std::vector<PipelineSample> latest(n);
    double last_print = -1.0;
    const auto pr = run_control_pipelined(bus, joints, default_zero_file(), traj, po, [] { return !cli::g_stop; },
                                          [&](const PipelineSample& s) {
      Trim& T = tr[s.index];
      latest[s.index] = s;
      if (s.index + 1 == n && s.t - last_print >= 0.5) {
        last_print = s.t;
        std::printf("%6.1f s", s.t);
        for (size_t k = 0; k < n; k++) {
          const Trim& U = tr[k];
          std::printf("  | ID %d %+7.2f deg target %+6.1f from-pos %+6.1f reported %+6.1f corr +%+.1f/-%+.1f deg/s",
                      id_num[k], latest[k].position * 360.0, last_target[k] * 360.0, U.vpos * 360.0, latest[k].velocity * 360.0,
                      U.corr[1] * 360.0, U.corr[0] * 360.0);
        }
        std::printf("\n");
        std::fflush(stdout);
      }
      // command history: goal = what the motor was asked minus our correction; limited = host ramp / braking
      const bool limited = std::fabs(s.cmd - s.setpoint.velocity) > 1e-9;
      T.hist.push_back({s.t, s.cmd - applied[s.index], limited});
      if (limited) T.last_limited = s.t;
      while (!T.hist.empty() && T.hist.front().t < s.t - win_s - lag_s - 0.1) T.hist.pop_front();
      // velocity from position (fresh replies only; time = reply arrival)
      if (s.fresh) {
        const double t_req = s.t - (s.age - 1) * dt;
        T.win.push_back({t_req + s.latency_us * 1e-6, s.position, s.velocity});
        while (T.win.size() > 2 && T.win.back().t - T.win[1].t >= win_s) T.win.pop_front();
        const double span = T.win.back().t - T.win.front().t;
        T.vpos_ok = span >= 0.8 * win_s;
        if (T.vpos_ok) {
          T.vpos = (T.win.back().pos - T.win.front().pos) / span;
          double r = 0;
          for (const auto& p : T.win) r += p.vrep;
          T.vrep_avg = r / T.win.size();
        }
      }
      if (!T.vpos_ok || T.win.size() < 2) return;
      // average goal over the window, shifted by the motor's lag; steady and one sign only
      const double a = T.win.front().t - lag_s, b = T.win.back().t - lag_s;
      if (a - T.last_limited < settle_s) return;
      double sum = 0, mn = 1e9, mx = -1e9;
      int cnt = 0;
      for (const auto& c : T.hist) {
        if (c.t < a || c.t > b) continue;
        if (c.limited) return;
        sum += c.goal;
        mn = std::min(mn, c.goal);
        mx = std::max(mx, c.goal);
        cnt++;
      }
      if (cnt < 3 || sgn(mn) != sgn(mx) || std::min(std::fabs(mn), std::fabs(mx)) < vmin) return;
      const double goal = sum / cnt, e = goal - T.vpos;
      const int d = goal > 0 ? 1 : 0;
      T.corr[d] = std::clamp(T.corr[d] + ki[s.index] * e * dt, -tmax[s.index], tmax[s.index]);
      Stat& S = T.st[s.t < 0.5 * po.duration_s ? 0 : 1][d];
      S.n++;
      S.goal += goal * 360.0;
      S.vpos += T.vpos * 360.0;
      S.vrep += T.vrep_avg * 360.0;
      S.err2 += e * e * 360.0 * 360.0;
    });

    std::printf("%s", to_string(pr.status));
    if (pr.failed_index >= 0) std::printf(" (ID %d)", id_num[pr.failed_index]);
    if (pr.cycles == 0) {
      std::printf("\nrefused before sending any command; the motors were not touched\n");
      return 1;
    }
    std::printf(": %d cycles in %.2f s (%.0f Hz)\n", pr.cycles, pr.elapsed_s, pr.cycles / std::max(pr.elapsed_s, 1e-9));
    std::printf("speed while steady (deg/s; from-pos = from position, the real speed; err = target - from-pos):\n");
    for (size_t k = 0; k < n; k++) {
      const Trim& T = tr[k];
      for (int d = 1; d >= 0; d--) {
        std::printf("  ID %d %s:", id_num[k], d ? "positive" : "negative");
        for (int h = 0; h < 2; h++) {
          const Stat& S = T.st[h][d];
          if (!S.n) {
            std::printf("  %s half: no steady data", h ? "2nd" : "1st");
            continue;
          }
          std::printf("  %s half: target %+6.1f from-pos %+6.1f (err %+5.1f, RMS %.1f) reported %+6.1f",
                      h ? "2nd" : "1st", S.goal / S.n, S.vpos / S.n, (S.goal - S.vpos) / S.n, std::sqrt(S.err2 / S.n),
                      S.vrep / S.n);
        }
        std::printf("  | corr at the end %+.1f deg/s%s\n", T.corr[d] * 360.0,
                    std::fabs(T.corr[d]) >= tmax[k] - 1e-12 && tmax[k] > 0 ? " (AT --trim-max)" : "");
      }
    }
    std::printf("link:\n");
    std::string can;
    bus.bus_off(&can);
    std::printf("  CAN %s, adapter ERR %llu, worst send %.0f us late, %d sends > 25%% of a period\n", can.c_str(),
                static_cast<unsigned long long>(pr.adapter_errors), pr.max_send_late_us, pr.late_sends);
    for (size_t k = 0; k < n; k++) {
      const long long lost = pr.requests[k] - pr.replies[k];
      std::printf("  ID %d: %lld requests, %lld replies, lost %lld (%.3f%%, %d found while running), longest run without "
                  "a new state %d cycles, stopped %s\n",
                  id_num[k], pr.requests[k], pr.replies[k], lost, 100.0 * lost / std::max(pr.requests[k], 1LL),
                  pr.lost_detected[k], pr.max_stale_seen[k], pr.stopped[k] ? "yes" : "NOT CONFIRMED");
    }
    return pr.status == ControlResult::Status::Done || pr.status == ControlResult::Status::Interrupted ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
