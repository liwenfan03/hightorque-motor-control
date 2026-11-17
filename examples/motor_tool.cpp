// Utility commands (motor_config.c and friends).
//
//   ./motor_tool list                      list adapter serial ports
//   ./motor_tool scan                      find motors on the bus (IDs 1..32)
//   ./motor_tool query   [--id 1] [--full] read state once (motor does not move)
//   ./motor_tool version [--id 1]          firmware version
//   ./motor_tool stop    [--id 1]          stop mode (also clears faults)
//   ./motor_tool reset   [--id 1]          soft reboot
//   ./motor_tool timeout <ms> [--id 1]     watchdog: brake if no command for <ms>
//   ./motor_tool rezero  [--id 1]          vendor rezero: zero + "conf write" to the MOTOR FLASH (avoid)
//   ./motor_tool set-zero [--id 1]         host-side zero: stop the motor, store where it is now
//                                          (turn it by hand first).  Nothing is written to the motor.
//   ./motor_tool zero    [--id 1]          show the stored zero and where the motor is relative to it
//   ./motor_tool go-zero [--id 1] [--vel 0.2] [--acc 0.5] [--tol 0.002] [--timeout 15] [--hold] [--force]
//                        [--corr-rate 0.5] [--corr-max 0.03]   host-side correction of the steady error (0: off)
//                                          move to the stored zero (velocity / accel limited, case 7).
//                                          Angles are relative to the zero, folded into +-180 deg, so whole
//                                          turns in the reading (power cycle) do not matter.  Needs hardcoded
//                                          joint limits for the ID (src/zero.cpp) and the motor inside them;
//                                          stops if it leaves them.  --force skips the limit checks.
//   ./motor_tool go-zero --id 1,2 [--model M4438_32,M5047_36] ...   all at once, synchronized
//                                          (same start, same arrival; any problem stops all)
//   ./motor_tool zero --id 1,2 [--model ...]   show several motors
//   (zero file: ~/.hightorque/zero_positions.txt, or --zero-file PATH)
//   ./motor_tool control --mode position|velocity|torque|impedance --id 1,2 [--model M4438_32,M5047_36]
//        loop:      --rate 300 (control loop Hz) --duration 10 (0 = until Ctrl-C) [--rt] [--trace FILE]
//        setpoint:  --profile hold|sine|square --bias B --amp A --freq 0.5 --ramp 1
//                   (deg for position / impedance, deg/s for velocity, Nm for torque; from the stored zero)
//        safety:    --safe-vel 0.5 (r/s, stop all above) --max-start-deg 10 --max-missed 3 (stale cycles in a row)
//                   --brake 0.5 (s: at the end / Ctrl-C / safety stop, brake (mode 15) until slow, then stop; 0 = stop only)
//        frames:    --resend-every 100 (position / velocity: unchanged registers only every N cycles; 1 = always)
//        pipelined by default (all modes): send at a fixed interval, do not wait for replies (pipeline.hpp);
//                   --sequential: send -> wait for the replies -> next (run_control, control.hpp)
//                   --max-stale 10 (cycles without a new reply -> stop all) --max-outstanding 16
//                   --shape host (default: vel-max / acc applied on the host, not sent) | motor (sent every frame)
//                   --tqe-frame min (24 B, default) | short (32 B) | full (48 B): torque / impedance frames
//        position / velocity = 位置模式 (mode 10), registers sent as given (unset = not sent):
//                   --vel (0x021, r/s) [--ff-vel: send the setpoint velocity] --tqe (0x022, Nm)
//                   --kp-scale --kd-scale (0x023 / 0x024, 0..1) --max-tqe (0x025, Nm) --stop-pos (0x026, deg)
//                   --vel-max (0x028, r/s) --acc (0x029, r/s^2)   (only --vel-max + --acc = case 7, none = case 2)
//                   velocity: --vel-ramp 1.0 (host accel limit, r/s^2; brakes before the limits)
//        torque:    --max-tqe 0.3 (host clamp) [--kd 0]       impedance: --kp --kd --max-tqe 0.3 [--gain-ramp 1]
//        Values: one = all motors, or one per --id entry.  Any problem on any motor stops all of them.
//   ./motor_tool impedance ...             = control --mode impedance (--target-deg = --bias, --max-vel = --safe-vel)
//   ./motor_tool adapter "<text>"          raw adapter command, e.g. "conf get can.termination"
//   ./motor_tool status                    adapter CAN controller state (busoff=1 => nothing gets sent)
//   ./motor_tool recover                   restart the adapter's CAN controller after bus-off
//   ./motor_tool read <reg> [count]        read float registers, e.g. "read 0x2b 2" (kp, kd)
//   ./motor_tool write <reg> <v> [v...]    write float registers (RAM only; `reset` restores them)
//
// Common options: --port PATH, --model M, --type float|int32|int16, --no-brs, --adapter-timing, --debug
// (status / recover / adapter never change the adapter's sample points)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <sys/select.h>
#include <unistd.h>

#include "cli_common.hpp"
#include "control_config.hpp"
#include "hightorque/motor.hpp"
#include "hightorque/control.hpp"
#include "hightorque/pipeline.hpp"
#include "hightorque/zero.hpp"

using namespace hightorque;

namespace {

// Print the hardcoded joint limits of this ID (relative to the zero).
std::optional<JointLimits> print_limits(uint8_t id) {
  const auto lim = joint_limits(id);
  if (!lim) {
    std::printf("joint limits: none hardcoded for ID %u (go-zero will refuse to move it)\n", id);
  } else if (lim->continuous) {
    std::printf("joint limits (%s): none, continuous joint (go-zero takes the short way, <= 180 deg)\n",
                lim->name);
  } else {
    std::printf("joint limits (%s): %+.1f .. %+.1f deg from zero\n", lim->name, lim->min_offset * 360.0,
                lim->max_offset * 360.0);
  }
  return lim;
}


GoToZeroOptions parse_zero_options(const cli::Args& args) {
  GoToZeroOptions zo;
  zo.vel_max = args.num("vel", zo.vel_max);
  zo.acc = args.num("acc", zo.acc);
  zo.tolerance = args.num("tol", zo.tolerance);
  zo.timeout_s = args.num("timeout", zo.timeout_s);
  zo.stop_at_end = !args.has("hold");
  zo.correction_rate = args.num("corr-rate", zo.correction_rate);
  zo.correction_max = args.num("corr-max", zo.correction_max);
  if (args.has("force")) {
    zo.check_limits = false;
    std::printf("WARNING: --force: joint limits are NOT checked\n");
  }
  return zo;
}

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

// Exact reply accounting around a go-zero: counts before, then after the run + a short wait for late ones.
struct LinkCount {
  uint64_t req = 0, rep = 0, late = 0;
};
LinkCount link_count(const Fdcanusb& bus, uint8_t id) {
  return {bus.requests_sent(id), bus.replies_received(id), bus.late_replies(id)};
}

void warn_lost_replies(uint8_t id, const GoToZeroResult& r, const LinkCount* before = nullptr,
                       const LinkCount* after = nullptr) {
  if (r.start_query_retries || r.missed_replies) {
    std::printf("WARNING: ID %u lost CAN replies: %d extra start queries, %d of %d cycles without reply (%.1f%%), first at "
                "cycle %d.\n",
                id, r.start_query_retries, r.missed_replies, r.cycles, r.cycles ? 100.0 * r.missed_replies / r.cycles : 0.0,
                r.first_missed_cycle);
    if (before && after) {
      const long long rq = static_cast<long long>(after->req - before->req), rp = static_cast<long long>(after->rep - before->rep);
      std::printf("         exact count (+100 ms wait): %lld requests, %lld replies -> %s; %llu replies arrived late "
                  "(dropped)\n",
                  rq, rp, rq == rp ? "none lost, only late" : (std::to_string(rq - rp) + " NEVER arrived").c_str(),
                  static_cast<unsigned long long>(after->late - before->late));
    }
    std::printf("         A terminated, healthy bus must show 0 (README: CAN 丢包).\n");
  }
}

// Before go-zero: if a motor is outside its hardcoded limits (e.g. turned by hand, or pushed out),
// say so, stop it (free to turn by hand) and wait until it has been turned back: press Enter to
// check again, q to give up.  Returns true when every motor is inside (go-zero may start).
// Without a terminal on stdin it only prints the problem and returns false.
bool wait_until_inside_limits(const std::vector<Motor*>& motors, const std::string& zfile, double margin) {
  for (;;) {
    bool all_inside = true;
    std::vector<std::string> lines;
    for (Motor* m : motors) {
      const auto lim = joint_limits(m->id());
      const auto rec = load_zero(zfile, m->id());
      if (!lim || lim->continuous || !rec) continue;  // go_to_zero() reports these itself
      auto s = m->query();
      for (int k = 0; k < 4 && (!s || !s->position); k++) s = m->query();
      if (!s || !s->position) continue;                // go_to_zero() reports "no reply"
      const double zeff = effective_zero(rec->position, *s->position);
      const double off = *s->position - zeff;
      if (within_limits(*lim, zeff, *s->position, margin)) continue;
      all_inside = false;
      const double edge = off > lim->max_offset ? lim->max_offset : lim->min_offset;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "  ID %u (%s) is at %+.1f deg, OUTSIDE its limits %+.0f..%+.0f deg: turn it back by hand %s "
                    "by at least %.1f deg",
                    m->id(), lim->name, off * 360.0, lim->min_offset * 360.0, lim->max_offset * 360.0,
                    off > 0 ? "(towards negative)" : "(towards positive)", std::fabs(off - edge) * 360.0 + margin * 360.0);
      lines.push_back(buf);
      m->stop();  // free to turn by hand
    }
    if (all_inside) return true;
    std::printf("\nCannot go to zero yet:\n");
    for (const auto& l : lines) std::printf("%s\n", l.c_str());
    if (!isatty(STDIN_FILENO)) {
      std::printf("(no terminal for input: turn it back, then run go-zero again)\n");
      return false;
    }
    std::printf("The motor(s) are stopped (free). Turn back by hand, then press Enter to check again (q + Enter: quit): ");
    std::fflush(stdout);
    // Wait for a line, but give up at once on Ctrl-C (a plain getline would keep waiting for Enter).
    bool line_ready = false;
    while (!cli::g_stop && !line_ready) {
      fd_set rd;
      FD_ZERO(&rd);
      FD_SET(STDIN_FILENO, &rd);
      timeval tv{0, 200000};
      line_ready = ::select(STDIN_FILENO + 1, &rd, nullptr, nullptr, &tv) > 0;
    }
    std::string answer;
    if (cli::g_stop || !std::getline(std::cin, answer) || answer == "q") {
      std::printf("\ngo-zero cancelled, nothing moved\n");
      return false;
    }
  }
}

// `zero` / `go-zero` for several motors: --id 1,2 [--model M4438_32,M5047_36] (one model = all).
int run_multi_zero(const std::string& cmd, Fdcanusb& bus, const cli::Args& args, const Motor::Options& base) {
  const auto ids = split_csv(args.str("id"));
  const auto models = split_csv(args.str("model"));
  if (models.size() > 1 && models.size() != ids.size()) {
    std::cerr << "--model: give one model for all motors or one per --id entry\n";
    return 2;
  }
  std::vector<std::unique_ptr<Motor>> owned;
  std::vector<Motor*> motors;
  for (size_t k = 0; k < ids.size(); k++) {
    Motor::Options o = base;
    o.model = cli::parse_model(models.empty() ? "" : models.size() == 1 ? models[0] : models[k]);
    owned.push_back(std::make_unique<Motor>(bus, static_cast<uint8_t>(std::atoi(ids[k].c_str())), o));
    motors.push_back(owned.back().get());
  }
  const std::string zfile = args.str("zero-file", default_zero_file());

  if (cmd == "zero") {
    int rc = 0;
    for (Motor* m : motors) {
      const auto rec = load_zero(zfile, m->id());
      if (!rec) {
        std::printf("ID %u: no zero stored in %s\n", m->id(), zfile.c_str());
        rc = 1;
        continue;
      }
      auto s = m->query();
      for (int k = 0; k < 4 && (!s || !s->position); k++) s = m->query();
      if (!s || !s->position) {
        std::printf("ID %u: no reply\n", m->id());
        rc = 1;
        continue;
      }
      const double zeff = effective_zero(rec->position, *s->position);
      const double off = *s->position - zeff;
      const auto lim = joint_limits(m->id());
      char limits[48] = "none";
      if (lim && lim->continuous) std::snprintf(limits, sizeof(limits), "continuous");
      if (lim && !lim->continuous) {
        std::snprintf(limits, sizeof(limits), "%+.0f..%+.0f deg", lim->min_offset * 360.0, lim->max_offset * 360.0);
      }
      std::printf("ID %u: %+8.2f deg from zero (raw %.4f r, zero %.4f r)   limits %s   inside: %s\n", m->id(),
                  off * 360.0, *s->position, rec->position, limits,
                  !lim ? "-" : within_limits(*lim, zeff, *s->position) ? "yes" : "NO");
    }
    return rc;
  }

  // go-zero, synchronized
  cli::install_ctrl_c();
  const GoToZeroOptions zo = parse_zero_options(args);
  if (zo.check_limits && !wait_until_inside_limits(motors, zfile, zo.limit_margin)) return 1;
  std::vector<LinkCount> lc0;
  for (Motor* m : motors) lc0.push_back(link_count(bus, m->id()));
  std::printf("synchronized go-zero of IDs %s, vel max %.2f r/s, acc %.2f r/s^2, correction %.2f /s (max %.3f r)\n",
              args.str("id").c_str(), zo.vel_max, zo.acc, zo.correction_rate, zo.correction_max);
  const auto res = go_to_zero_many(
      bus, motors, zfile, zo, [] { return !cli::g_stop; },
      [&](size_t i, const MotorState& st, double error, double command_offset) {
        std::printf("  ID %u  error %+8.2f deg   vel %+.3f r/s   tqe %+.3f Nm   cmd-zero %+.4f r\n", motors[i]->id(),
                    error * 360.0, st.velocity.value_or(NAN), st.torque.value_or(NAN), command_offset);
        std::fflush(stdout);
      });
  std::printf("%s", to_string(res.status));
  if (res.failed_index >= 0) std::printf(" (ID %u)", motors[res.failed_index]->id());
  std::printf(": planned %.1f s, took %.1f s%s\n", res.planned_s, res.elapsed_s,
              res.status == GoToZeroResult::Status::Reached && !zo.stop_at_end ? ", holding (Ctrl-C to release)" : "");
  for (size_t i = 0; i < motors.size(); i++) {
    const auto& r = res.motors[i];
    if (r.cycles == 0 && res.status != GoToZeroResult::Status::OutOfLimits) continue;  // refused before reading
    std::printf("  ID %u: start %+7.2f deg, final error %+.4f r (%+.2f deg), correction %+.4f r\n", motors[i]->id(),
                r.start_offset * 360.0, r.final_error, r.final_error * 360.0, r.correction);
  }
  bus.drain(std::chrono::milliseconds(100));
  for (size_t i = 0; i < motors.size(); i++) {
    const LinkCount lc1 = link_count(bus, motors[i]->id());
    warn_lost_replies(motors[i]->id(), res.motors[i], &lc0[i], &lc1);
  }
  if (res.status == GoToZeroResult::Status::OutOfLimits && res.failed_index >= 0) {
    print_limits(motors[res.failed_index]->id());
    std::printf("ID %u is outside its limits. Turn it back by hand (motors stopped), check with `motor_tool zero`.\n",
                motors[res.failed_index]->id());
  }
  if (res.status == GoToZeroResult::Status::Reached && !zo.stop_at_end) {
    cli::Rate rate(100);
    while (!cli::g_stop) {
      std::vector<std::pair<Motor*, FrameBuilder>> hold;
      for (size_t i = 0; i < motors.size(); i++) {
        hold.emplace_back(motors[i], motors[i]->command_pos_velmax_acc(res.motors[i].command, zo.vel_max, zo.acc));
      }
      cycle(bus, hold);
      rate.sleep();
    }
    for (Motor* m : motors) m->stop();
    std::printf("released\n");
  }
  return res.status == GoToZeroResult::Status::Reached ? 0 : 1;
}

// Per-joint value from "--key a,b" (one value = all joints).
bool per_joint(const cli::Args& args, const std::string& key, size_t n, double def, std::vector<double>& out) {
  const auto v = split_csv(args.str(key));
  out.assign(n, def);
  if (v.empty() || (v.size() == 1 && v[0].empty())) return true;
  if (v.size() != 1 && v.size() != n) {
    std::cerr << "--" << key << ": give one value for all motors or one per --id entry\n";
    return false;
  }
  for (size_t i = 0; i < n; i++) out[i] = std::atof((v.size() == 1 ? v[0] : v[i]).c_str());
  return true;
}

// Setpoint generator for `control`: value(t) = bias + envelope(t) * amp * wave(2 pi f t).
// envelope ramps 0 -> 1 over --ramp seconds at the start (and 1 -> 0 before the end).
struct Profile {
  enum class Wave { Hold, Sine, Square } wave = Wave::Hold;
  double bias = 0.0, amp = 0.0, freq = 0.5;  // in the mode's unit (turns, turns/s or Nm), Hz
};
double envelope(double t, double ramp, double duration) {
  if (ramp <= 0) return 1.0;
  double g = std::min(t / ramp, 1.0);
  if (duration > 0) g = std::min(g, (duration - t) / ramp);
  return std::clamp(g, 0.0, 1.0);
}

// Host control loop: --mode position|velocity|torque|impedance, see the usage at the top.
int run_control_cmd(Fdcanusb& bus, const cli::Args& args, const Motor::Options& base, const std::string& mode_name) {
  ControlMode mode;
  if (mode_name == "position") mode = ControlMode::Position;
  else if (mode_name == "velocity") mode = ControlMode::Velocity;
  else if (mode_name == "torque") mode = ControlMode::Torque;
  else if (mode_name == "impedance") mode = ControlMode::Impedance;
  else {
    std::cerr << "--mode: position, velocity, torque or impedance\n";
    return 2;
  }
  const bool angle = mode == ControlMode::Position || mode == ControlMode::Impedance;
  // user unit -> turns / turns/s / Nm
  const double unit = mode == ControlMode::Torque ? 1.0 : 1.0 / 360.0;
  const char* unit_name = angle ? "deg" : mode == ControlMode::Velocity ? "deg/s" : "Nm";

  const auto ids = split_csv(args.str("id", "1"));
  const size_t n = ids.size();
  const auto models = split_csv(args.str("model"));
  if (models.size() > 1 && models.size() != n) {
    std::cerr << "--model: give one model for all motors or one per --id entry\n";
    return 2;
  }
  std::vector<int> id_num(n);
  for (size_t k = 0; k < n; k++) id_num[k] = std::atoi(ids[k].c_str());

  // ---- config file (config/control.conf); the command line overrides it ----
  cli::ControlConfig cfg;
  {
#ifdef HT_SOURCE_DIR
    const std::string def_path = std::string(HT_SOURCE_DIR) + "/config/control.conf";
#else
    const std::string def_path = "config/control.conf";
#endif
    const std::string path = args.str("config", def_path);
    std::ifstream probe(path);
    if (probe || args.has("config")) {
      try {
        cfg = cli::ControlConfig(path);
      } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 2;
      }
      std::printf("config: %s\n", path.c_str());
    } else {
      std::printf("config: none (%s not found), built-in defaults\n", path.c_str());
    }
  }
  // Per-motor value: --key (one or one per --id), else the config, else `def`.
  auto per_joint = [&](const cli::Args& a, const std::string& key, size_t, double def, std::vector<double>& out) {
    if (a.has(key)) return ::per_joint(a, key, n, def, out);
    out.assign(n, def);
    for (size_t k = 0; k < n; k++) {
      if (const auto v = cfg.get(mode_name, id_num[k], key)) out[k] = std::atof(v->c_str());
    }
    return true;
  };
  // Loop-wide value: --key, else [<mode>] / [common], else `def`.
  auto scalar = [&](const std::string& key, double def) {
    if (args.has(key)) return args.num(key, def);
    if (const auto v = cfg.get(mode_name, key)) return std::atof(v->c_str());
    return def;
  };
  const bool regs_mode = mode == ControlMode::Position || mode == ControlMode::Velocity;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> kp, kd, max_tqe, bias, amp, freq, safe_vel, vel_ramp;
  std::vector<double> r_vel, r_tqe, r_kps, r_kds, r_vmax, r_acc, r_stop;
  const std::string bias_key = args.has("target-deg") ? "target-deg" : "bias";  // old impedance option
  const std::string safe_key = args.has("max-vel") ? "max-vel" : "safe-vel";     // old impedance option
  if (!per_joint(args, "kp", n, 0.0, kp) || !per_joint(args, "kd", n, 0.0, kd) ||
      // position / velocity: register 0x025 (NaN = not sent); torque / impedance: host clamp
      !per_joint(args, "max-tqe", n, regs_mode ? nan : 0.3, max_tqe) ||
      !per_joint(args, bias_key, n, 0.0, bias) || !per_joint(args, "amp", n, 0.0, amp) ||
      !per_joint(args, "freq", n, 0.5, freq) || !per_joint(args, safe_key, n, 0.5, safe_vel) ||
      !per_joint(args, "vel-ramp", n, 1.0, vel_ramp) || !per_joint(args, "vel", n, 0.0, r_vel) ||
      !per_joint(args, "tqe", n, 0.0, r_tqe) || !per_joint(args, "kp-scale", n, 1.0, r_kps) ||
      !per_joint(args, "kd-scale", n, 1.0, r_kds) || !per_joint(args, "vel-max", n, nan, r_vmax) ||
      !per_joint(args, "acc", n, nan, r_acc) || !per_joint(args, "stop-pos", n, nan, r_stop)) {
    return 2;
  }
  if (!regs_mode) {
    for (const char* k : {"vel", "tqe", "kp-scale", "kd-scale", "vel-max", "acc", "stop-pos", "ff-vel"}) {
      if (args.has(k)) {
        std::cerr << "--" << k << " is a position-mode register: only for --mode position / velocity\n";
        return 2;
      }
    }
  }
  const std::string wave = args.has("profile") ? args.str("profile") : cfg.get(mode_name, "profile").value_or("hold");
  Profile::Wave w = Profile::Wave::Hold;
  if (wave == "sine") w = Profile::Wave::Sine;
  else if (wave == "square") w = Profile::Wave::Square;
  else if (wave != "hold") {
    std::cerr << "--profile: hold, sine or square\n";
    return 2;
  }
  std::vector<Profile> prof(n);
  for (size_t k = 0; k < n; k++) prof[k] = Profile{w, bias[k] * unit, w == Profile::Wave::Hold ? 0.0 : amp[k] * unit, freq[k]};

  ControlOptions co;
  co.rate_hz = scalar("rate", co.rate_hz);
  co.duration_s = scalar("duration", co.duration_s);
  co.max_start_error = scalar("max-start-deg", co.max_start_error * 360.0) / 360.0;
  co.gain_ramp_s = scalar("gain-ramp", co.gain_ramp_s);
  co.max_missed_in_row = static_cast<int>(scalar("max-missed", co.max_missed_in_row));
  co.resend_every = static_cast<int>(scalar("resend-every", co.resend_every));
  co.brake_s = scalar("brake", co.brake_s);
  if (co.max_missed_in_row < 1) {
    std::cerr << "--max-missed must be >= 1\n";
    return 2;
  }
  const double ramp = scalar("ramp", 1.0);

  std::vector<std::unique_ptr<Motor>> owned;
  std::vector<ControlJoint> joints(n);
  for (size_t k = 0; k < n; k++) {
    Motor::Options o = base;
    std::string model_name = models.empty() ? "" : models.size() == 1 ? models[0] : models[k];
    if (model_name.empty()) model_name = cfg.model(id_num[k]).value_or("");
    if (model_name.empty()) std::printf("WARNING: no model for ID %d (--model or [motors] in the config), using M5047_36\n", id_num[k]);
    o.model = cli::parse_model(model_name);
    // Wait at most about one period for a reply; a later reply counts as missed.
    o.timeout = std::chrono::microseconds(static_cast<int64_t>(std::max(1000.0, 0.9e6 / co.rate_hz)));
    owned.push_back(std::make_unique<Motor>(bus, static_cast<uint8_t>(id_num[k]), o));
    ControlJoint& j = joints[k];
    j.motor = owned.back().get();
    j.mode = mode;
    j.kp = kp[k];
    j.kd = kd[k];
    if (!regs_mode) j.max_torque = max_tqe[k];
    j.safe_vel = safe_vel[k];
    j.vel_ramp = vel_ramp[k];
    j.feed_forward_velocity = args.has("ff-vel") || cfg.get(mode_name, id_num[k], "ff-vel").value_or("0") == "1";
    PositionModeCommand& r = j.regs;
    r.velocity = r_vel[k];
    r.torque = r_tqe[k];
    r.kp_scale = r_kps[k];
    r.kd_scale = r_kds[k];
    r.max_torque = max_tqe[k];
    r.vel_limit = r_vmax[k];
    r.acc_limit = r_acc[k];
    r.stop_position = r_stop[k] / 360.0;  // deg from zero -> turns (NaN stays NaN)
  }
  const double duration = co.duration_s;
  const ControlTrajectory traj = [&](size_t i, double t) {
    const Profile& p = prof[i];
    const double g = envelope(t, ramp, duration);
    const double ph = 2.0 * M_PI * p.freq * t;
    double v = p.bias, dv = 0.0;
    if (p.wave == Profile::Wave::Sine) {
      v += g * p.amp * std::sin(ph);
      dv = g * p.amp * 2.0 * M_PI * p.freq * std::cos(ph);
    } else if (p.wave == Profile::Wave::Square) {
      v += g * p.amp * (std::sin(ph) >= 0 ? 1.0 : -1.0);
    }
    ControlSetpoint sp;
    if (angle) {
      sp.position = v;
      sp.velocity = dv;
    } else if (mode == ControlMode::Velocity) {
      sp.velocity = v;
    } else {
      sp.torque = v;
    }
    return sp;
  };

  const std::string zfile = args.str("zero-file", default_zero_file());
  const std::string log = args.str("log");
  cli::install_ctrl_c();
  std::FILE* trace = nullptr;
  if (args.has("trace")) {
    trace = std::fopen(args.str("trace").c_str(), "w");
    if (!trace) {
      std::cerr << "cannot write --trace " << args.str("trace") << "\n";
      return 2;
    }
    std::setvbuf(trace, nullptr, _IOFBF, 1 << 20);
    std::fprintf(trace, "# motor_tool control trace: rate_hz=%.1f ids=%s\n", co.rate_hz, args.str("id", "1").c_str());
    bus.set_trace(trace);
  }
  if (args.has("rt")) cli::make_realtime(co.rate_hz);
  std::printf("%s control, loop %.0f Hz (--rate), %s, profile %s (ramp %.1f s)\n", to_string(mode), co.rate_hz,
              duration > 0 ? (std::to_string(duration) + " s").c_str() : "until Ctrl-C", wave.c_str(), ramp);
  const bool host_shape = args.has("pipeline") && mode == ControlMode::Position &&
                          (args.has("shape") ? args.str("shape") : cfg.get(mode_name, "shape").value_or("host")) != "motor";
  auto reg = [](const char* name, double v, double neutral, const char* u) {
    const bool unused = std::isnan(neutral) ? std::isnan(v) : v == neutral;
    if (!unused) std::printf(" %s %.3g%s", name, v, u);
  };
  for (size_t k = 0; k < n; k++) {
    const ControlJoint& j = joints[k];
    std::printf("  ID %u (%s): %s %+.2f %s", j.motor->id(), motor_model_name(j.motor->options().model), angle ? "center" : "bias",
                bias[k], unit_name);
    if (w != Profile::Wave::Hold) std::printf(", amp %.2f %s at %.2f Hz", amp[k], unit_name, freq[k]);
    if (regs_mode) {
      std::printf(", registers:");
      const PositionModeCommand& r = j.regs;
      if (mode == ControlMode::Position && j.feed_forward_velocity) std::printf(" vel = setpoint velocity");
      else if (mode == ControlMode::Position) reg("vel", r.velocity, 0.0, " r/s");
      reg("tqe", r.torque, 0.0, " Nm");
      reg("kp-scale", r.kp_scale, 1.0, "");
      reg("kd-scale", r.kd_scale, 1.0, "");
      reg("max-tqe", r.max_torque, nan, " Nm");
      reg("stop-pos", r.stop_position * 360.0, nan, " deg");
      if (!host_shape) {
        reg("vel-max", r.vel_limit, nan, " r/s");
        reg("acc", r.acc_limit, nan, " r/s^2");
      }
      const bool none = r.velocity == 0 && r.torque == 0 && r.kp_scale == 1 && r.kd_scale == 1 && std::isnan(r.max_torque) &&
                        std::isnan(r.stop_position) && std::isnan(r.vel_limit) && std::isnan(r.acc_limit) &&
                        !(mode == ControlMode::Position && j.feed_forward_velocity);
      if (none) std::printf(" none (firmware defaults%s)", mode == ControlMode::Position ? " = case 2" : " = case 3");
    }
    if (regs_mode) {
      if (host_shape) std::printf(" (sent every cycle; vel-max / acc: on the host, see below)");
      else if (co.resend_every <= 1) std::printf(" (all sent every cycle)");
      else std::printf(" (sent when changed, all every %d cycles)", co.resend_every);
    }
    if (mode == ControlMode::Velocity) std::printf(", host ramp %.2f r/s^2", j.vel_ramp);
    if (mode == ControlMode::Impedance) std::printf(", kp %.2f Nm/turn kd %.3f Nm/(turn/s)", j.kp, j.kd);
    if (mode == ControlMode::Torque && j.kd > 0) std::printf(", damping kd %.3f", j.kd);
    if (mode == ControlMode::Torque || mode == ControlMode::Impedance) std::printf(", |tqe| <= %.2f Nm", j.max_torque);
    if (j.safe_vel > 0) std::printf(", stop if |vel| > %.2f r/s\n", j.safe_vel);
    else std::printf(", VELOCITY CHECK OFF (--safe-vel 0)\n");
  }
  for (size_t k = 0; k < n; k++) {
    if (mode == ControlMode::Torque && std::fabs(bias[k]) + (w == Profile::Wave::Hold ? 0.0 : amp[k]) > joints[k].max_torque) {
      std::printf("WARNING: ID %u: bias + amp = %.2f Nm > max-tqe %.2f Nm: the torque command is CLAMPED to +-%.2f Nm "
                  "(raise --max-tqe to really apply it)\n",
                  joints[k].motor->id(), std::fabs(bias[k]) + amp[k], joints[k].max_torque, joints[k].max_torque);
    }
  }
  for (const auto& j : joints) {
    if (j.safe_vel <= 0) {
      std::printf("WARNING: ID %u: no velocity stop. Joint limits, faults and lost replies are still checked; Ctrl-C stops.\n",
                  j.motor->id());
    }
  }
  std::fflush(stdout);

  // Pipelined (fixed-interval sends) is the default for every mode; --sequential (or "pipeline = 0" in the
  // config) uses run_control(): send -> wait for the replies -> next cycle.
  const bool pipelined = !args.has("sequential") && (args.has("pipeline") || cfg.get(mode_name, "pipeline").value_or("1") != "0");
  if (pipelined) {
    // ---- pipelined loop (run_control_pipelined, pipeline.hpp) ----
    PipelineOptions po;
    po.rate_hz = co.rate_hz;
    po.duration_s = co.duration_s;
    po.max_start_error = co.max_start_error;
    po.resend_every = co.resend_every;
    po.max_stale = static_cast<int>(scalar("max-stale", po.max_stale));
    po.max_outstanding = static_cast<int>(scalar("max-outstanding", po.max_outstanding));
    po.warmup_cycles = static_cast<int>(scalar("warmup", po.warmup_cycles));
    po.gain_ramp_s = co.gain_ramp_s;
    po.brake_s = co.brake_s;
    {
      const std::string tf = args.has("tqe-frame") ? args.str("tqe-frame") : cfg.get(mode_name, "tqe-frame").value_or("min");
      if (tf == "min") po.torque_frame = PipelineOptions::TorqueFrame::Min;
      else if (tf == "short") po.torque_frame = PipelineOptions::TorqueFrame::Short;
      else if (tf == "full") po.torque_frame = PipelineOptions::TorqueFrame::Full;
      else {
        std::cerr << "--tqe-frame: min (24 B: torque, kp/kd scale 0), short (32 B: + position NaN, velocity 0), full (48 B)\n";
        return 2;
      }
      if (mode == ControlMode::Torque || mode == ControlMode::Impedance) {
        std::printf("  torque frame: %s\n", tf == "min"     ? "min, 24 byte (torque, kp scale 0, kd scale 0)"
                                         : tf == "short" ? "short, 32 byte (+ position NaN, velocity 0)"
                                                         : "full, 48 byte (+ max torque NaN)");
      }
    }
    po.brake_margin = co.brake_margin;
    const std::string shape = args.has("shape") ? args.str("shape") : cfg.get(mode_name, "shape").value_or("host");
    if (shape != "host" && shape != "motor") {
      std::cerr << "--shape: host (limits on the host, default) or motor (vel-max / acc sent in every frame)\n";
      return 2;
    }
    po.host_shaping = shape == "host";
    if (mode != ControlMode::Position) {
      // shaping applies to position targets only
    } else if (po.host_shaping) {
      for (const auto& j : joints) {
        std::printf("  ID %u: --shape host: setpoints limited ON THE HOST to vel-max %.3g r/s, acc %.3g r/s^2 "
                    "(0x028 / 0x029 not sent)\n",
                    j.motor->id(), std::isnan(j.regs.vel_limit) ? 0.3 : j.regs.vel_limit,
                    std::isnan(j.regs.acc_limit) ? 1.0 : j.regs.acc_limit);
      }
    } else {
      std::printf("  --shape motor: vel-max / acc sent to the motor in every frame\n");
    }
    std::printf("PIPELINED: sends every %.0f us whatever the replies; stop if a motor gives no new reply for %d cycles "
                "or has > %d requests open\n",
                1e6 / po.rate_hz, po.max_stale, po.max_outstanding);
    std::fflush(stdout);
    struct Err { double sum2 = 0, max = 0; int count = 0; };
    std::vector<Err> perr(n), cerr(n);
    std::vector<std::vector<double>> lat(n);
    for (auto& v : lat) v.reserve(static_cast<size_t>(po.rate_hz * (po.duration_s > 0 ? po.duration_s : 600.0) + 16));
    std::vector<PipelineSample> latest(n);
    std::vector<int> fresh_count(n, 0);
    double last_print = -1.0;
    // tracking error of the controlled quantity: position / impedance deg, velocity deg/s, torque Nm
    auto ctrl_err = [&](const PipelineSample& s) {
      if (angle) return (s.setpoint.position - s.position) * 360.0;
      if (mode == ControlMode::Velocity) return (s.cmd - s.velocity) * 360.0;
      return s.cmd - s.torque_meas;
    };
    const auto pr = run_control_pipelined(bus, joints, zfile, traj, po, [] { return !cli::g_stop; },
                                          [&](const PipelineSample& s) {
      latest[s.index] = s;
      if (s.fresh) {
        fresh_count[s.index]++;
        if (lat[s.index].size() < lat[s.index].capacity()) lat[s.index].push_back(s.latency_us);
      }
      if (s.age > 0) {
        const double e = ctrl_err(s), ec = mode == ControlMode::Position ? (s.cmd - s.position) * 360.0 : e;
        Err& E = perr[s.index];
        E.sum2 += e * e;
        E.max = std::max(E.max, std::fabs(e));
        E.count++;
        Err& C = cerr[s.index];
        C.sum2 += ec * ec;
        C.max = std::max(C.max, std::fabs(ec));
        C.count++;
      }
      if (s.index + 1 == n && s.t - last_print >= 0.5) {
        last_print = s.t;
        std::printf("%6.1f s", s.t);
        for (size_t k = 0; k < n; k++) {
          const PipelineSample& L = latest[k];
          std::printf("  | ID %u %+7.2f deg %+6.3f r/s tqe %+6.3f Nm", joints[k].motor->id(), L.position * 360.0, L.velocity,
                      L.torque_meas);
          if (angle) std::printf(" sp %+7.2f deg", L.setpoint.position * 360.0);
          if (mode == ControlMode::Velocity) std::printf(" cmd %+6.3f r/s", L.cmd);
          if (mode == ControlMode::Torque || mode == ControlMode::Impedance) std::printf(" cmd %+6.3f Nm", L.cmd);
          std::printf(" age %d", L.age);
        }
        std::printf("\n");
        std::fflush(stdout);
      }
    });
    if (trace) {
      bus.set_trace(nullptr);
      std::fclose(trace);
      std::printf("trace: %s (check with: python3 tools/check_link.py --trace %s)\n", args.str("trace").c_str(),
                  args.str("trace").c_str());
    }
    std::printf("%s", to_string(pr.status));
    if (pr.failed_index >= 0) std::printf(" (ID %u)", joints[pr.failed_index].motor->id());
    if (pr.cycles == 0) {
      std::printf("\nrefused before sending any command; the motors were not touched\n");
      return 1;
    }
    std::printf(": %d cycles in %.2f s (%.0f Hz)\n", pr.cycles, pr.elapsed_s, pr.cycles / std::max(pr.elapsed_s, 1e-9));
    for (size_t k = 0; k < n; k++) {
      const Err& E = perr[k];
      const Err& C = cerr[k];
      if (mode == ControlMode::Position) {
        std::printf("  ID %u: position error vs setpoint RMS %.3f / max %.3f deg, vs command sent (after host shaping) "
                    "RMS %.3f / max %.3f deg, stopped %s\n",
                    joints[k].motor->id(), E.count ? std::sqrt(E.sum2 / E.count) : 0.0, E.max,
                    C.count ? std::sqrt(C.sum2 / C.count) : 0.0, C.max, pr.stopped[k] ? "yes" : "NOT CONFIRMED");
      } else {
        const char* what = mode == ControlMode::Impedance ? "position error (setpoint - measured)"
                           : mode == ControlMode::Velocity ? "velocity error (command - measured)"
                                                           : "torque error (command - measured)";
        std::printf("  ID %u: %s RMS %.3f / max %.3f %s, stopped %s\n", joints[k].motor->id(), what,
                    E.count ? std::sqrt(E.sum2 / E.count) : 0.0, E.max, unit_name, pr.stopped[k] ? "yes" : "NOT CONFIRMED");
      }
    }
    // ---- verdict for a PIPELINED run (README "Pipelining: 判定标准") ----
    // Each check is PASS, WARN (acceptable while testing, must be explained) or FAIL.
    const double period_us = 1e6 / po.rate_hz;
    enum Level { PASS = 0, WARN = 1, FAIL = 2 };
    int worst = PASS;
    auto verdict = [&](Level l, const std::string& what) {
      worst = std::max(worst, static_cast<int>(l));
      std::printf("  [%s] %s\n", l == PASS ? "PASS" : l == WARN ? "WARN" : "FAIL", what.c_str());
    };
    auto fmt = [](const char* f, auto... v) {
      char b[512];
      std::snprintf(b, sizeof(b), f, v...);
      return std::string(b);
    };
    std::printf("link (pipelined, period %.0f us):\n", period_us);
    // 1. run ended normally
    verdict(pr.status == ControlResult::Status::Done || pr.status == ControlResult::Status::Interrupted ? PASS : FAIL,
            fmt("run: %s", to_string(pr.status)));
    // 2. CAN bus / adapter
    std::string can_status;
    bus.bus_off(&can_status);
    const bool can_errors = can_status.find("err=0") == std::string::npos || can_status.find("warn=0") == std::string::npos ||
                            can_status.find("busoff=0") == std::string::npos;
    verdict(can_errors || pr.adapter_errors ? FAIL : PASS,
            fmt("CAN / adapter: %s, ERR lines %llu (must be clean)", can_status.c_str(),
                static_cast<unsigned long long>(pr.adapter_errors)));
    // 3. fixed schedule: FAIL if any send is > 50% of a period late, WARN if any is > 25%
    verdict(pr.max_send_late_us > 0.5 * period_us ? FAIL : pr.late_sends ? WARN : PASS,
            fmt("schedule: worst send %.0f us late (PASS <= 25%% = %.0f us, FAIL > 50%%), %d sends > 25%%",
                pr.max_send_late_us, 0.25 * period_us, pr.late_sends));
    for (size_t k = 0; k < n; k++) {
      const unsigned id = joints[k].motor->id();
      // 4. replies that never arrived: 0 = PASS; <= 0.2% and all found while running = WARN (firmware drops a
      //    request during its 10 ms stall); more, or not all found = FAIL
      const long long lost = pr.requests[k] - pr.replies[k];
      const double lost_pct = 100.0 * lost / std::max(pr.requests[k], 1LL);
      const bool all_found = pr.lost_detected[k] >= lost;
      verdict(lost == 0 ? PASS : (lost > 0 && lost_pct <= 0.2 && all_found) ? WARN : FAIL,
              fmt("ID %u lost: %lld of %lld requests never answered (%.3f%%), %d found while running "
                  "(PASS 0; WARN <= 0.2%% and all found; FAIL above)",
                  id, lost, pr.requests[k], lost_pct, pr.lost_detected[k]));
      // 5. age of the state the controller used: p99 <= 2 cycles PASS, p99.9 <= 8 WARN, else FAIL
      const auto& h = pr.age_hist[k];
      int total = 0;
      for (int v : h) total += v;
      auto age_pct = [&](double q) {
        int acc = 0;
        for (size_t a2 = 0; a2 < h.size(); a2++) {
          acc += h[a2];
          if (acc >= q * total) return static_cast<int>(a2);
        }
        return static_cast<int>(h.size() - 1);
      };
      const int a99 = age_pct(0.99), a999 = age_pct(0.999);
      std::string dist;
      for (size_t a2 = 1; a2 < h.size(); a2++) {
        if (h[a2]) dist += fmt(" %s%zu:%.2f%%", a2 == 9 ? ">=" : "", a2, 100.0 * h[a2] / std::max(total, 1));
      }
      verdict(a99 <= 2 ? PASS : a999 <= 8 ? WARN : FAIL,
              fmt("ID %u state age (cycles): p99 %d, p99.9 %d%s  (PASS p99 <= 2; WARN p99.9 <= 8)", id, a99, a999, dist.c_str()));
      // 6. freshness: longest run of cycles without a new state
      verdict(pr.max_stale_seen[k] <= 2 ? PASS : pr.max_stale_seen[k] < po.max_stale ? WARN : FAIL,
              fmt("ID %u longest run without a new state: %d cycles (PASS <= 2; FAIL >= --max-stale %d)", id,
                  pr.max_stale_seen[k], po.max_stale));
      // info only
      std::vector<double> L = lat[k];
      std::sort(L.begin(), L.end());
      auto pct = [&](double q) { return L.empty() ? 0.0 : L[std::min(L.size() - 1, static_cast<size_t>(q * L.size()))]; };
      std::printf("         (info) reply latency p50 %.0f / p99 %.0f / max %.0f us (p99 / max include up to 20 ms after "
                  "each loss, before it is found), %d cycles without a new reply (%.1f%%)\n",
                  pct(0.5), pct(0.99), L.empty() ? 0.0 : L.back(), pr.not_fresh[k],
                  100.0 * pr.not_fresh[k] / std::max(pr.cycles, 1));
    }
    std::printf("  => %s\n", worst == PASS ? "PASS" : worst == WARN ? "PASS WITH WARNINGS (see WARN lines)" : "FAIL (see FAIL lines)");
    return pr.status == ControlResult::Status::Done || pr.status == ControlResult::Status::Interrupted ? 0 : 1;
  }

  // Tracking error of the controlled quantity: position (deg), velocity (deg/s) or torque (Nm).
  struct Err { double sum2 = 0, max = 0; int count = 0; };
  std::vector<Err> err(n);
  std::vector<ControlSample> buf;
  if (!log.empty()) buf.reserve(static_cast<size_t>(n * co.rate_hz * (duration > 0 ? duration : 120.0) + 16));
  std::vector<ControlSample> latest(n);
  std::vector<int> run_missed(n, 0), run_late(n, 0);  // running totals for the live line
  std::vector<double> rtts;  // one per cycle
  rtts.reserve(static_cast<size_t>(co.rate_hz * (duration > 0 ? duration : 600.0) + 16));
  double last_print = -1.0;
  const auto res = run_control(bus, joints, zfile, traj, co, [] { return !cli::g_stop; }, [&](const ControlSample& s) {
    if (!log.empty() && buf.size() < buf.capacity()) buf.push_back(s);
    if (s.index == 0 && rtts.size() < rtts.capacity()) rtts.push_back(s.rtt_us);
    latest[s.index] = s;
    if (s.stale) run_missed[s.index]++;
    run_late[s.index] += s.late;
    if (!s.stale) {
      double e = 0;
      if (angle) e = (s.setpoint.position - s.position) * 360.0;
      else if (mode == ControlMode::Velocity) e = (s.cmd - s.velocity) * 360.0;
      else e = s.cmd - s.torque_meas;
      Err& E = err[s.index];
      E.sum2 += e * e;
      E.max = std::max(E.max, std::fabs(e));
      E.count++;
    }
    if (s.index + 1 == n && s.t - last_print >= 0.5) {
      last_print = s.t;
      std::printf("%6.1f s", s.t);
      for (size_t k = 0; k < n; k++) {
        const ControlSample& L = latest[k];
        std::printf("  | ID %u %+7.2f deg %+6.3f r/s tqe %+6.3f Nm", joints[k].motor->id(), L.position * 360.0, L.velocity,
                    L.torque_meas);
        if (angle) std::printf(" sp %+7.2f deg", L.setpoint.position * 360.0);
        if (mode == ControlMode::Velocity) std::printf(" cmd %+6.3f r/s", L.cmd);
        if (mode == ControlMode::Torque || mode == ControlMode::Impedance) std::printf(" cmd %+6.3f Nm", L.cmd);
        std::printf(" miss %d late %d", run_missed[k], run_late[k]);
        if (L.stale) std::printf(" STALE");
      }
      std::printf("\n");
      std::fflush(stdout);
    }
  });

  if (trace) {
    bus.set_trace(nullptr);
    std::fclose(trace);
    std::printf("trace: %s (check with: python3 tools/check_link.py --trace %s)\n", args.str("trace").c_str(),
                args.str("trace").c_str());
  }
  std::printf("%s", to_string(res.status));
  if (res.failed_index >= 0) std::printf(" (ID %u)", joints[res.failed_index].motor->id());
  if (res.cycles == 0) {
    std::printf("\nrefused before sending any command; the motors were not touched\n");
    if (res.status == ControlResult::Status::OutOfLimits && res.failed_index >= 0) print_limits(joints[res.failed_index].motor->id());
    return 1;
  }
  std::printf(": %d cycles in %.1f s (%.0f Hz), %d overruns\n", res.cycles, res.elapsed_s,
              res.elapsed_s > 0 ? res.cycles / res.elapsed_s : 0.0, res.overruns);
  const char* what = angle ? "position error (setpoint - measured)" : mode == ControlMode::Velocity
                                                                          ? "velocity error (command - measured)"
                                                                          : "torque error (command - measured)";
  for (size_t k = 0; k < n; k++) {
    const Err& E = err[k];
    std::printf("  ID %u: %s RMS %.3f / max %.3f %s, stopped %s\n", joints[k].motor->id(), what,
                E.count ? std::sqrt(E.sum2 / E.count) : 0.0, E.max, unit_name, res.stopped[k] ? "yes" : "NOT CONFIRMED");
  }
  // ---- link report: lost vs late replies, round trip, adapter CAN state ----
  const double period_us = 1e6 / co.rate_hz;
  const double timeout_us = std::max(1000.0, 0.9e6 / co.rate_hz);
  std::printf("link (%d cycles, period %.0f us, reply timeout %.0f us):\n", res.cycles, period_us, timeout_us);
  bool link_ok = true;
  for (size_t k = 0; k < n; k++) {
    const long long lost = res.requests[k] - res.replies[k];
    std::printf("  ID %u: no reply in time %d (%.3f%%), of which seen arriving late %d\n", joints[k].motor->id(),
                res.missed[k], 100.0 * res.missed[k] / std::max(res.cycles, 1), res.late[k]);
    std::printf("         exact count over the run (+%.0f ms wait at the end): %lld requests, %lld replies -> %s\n",
                co.drain_s * 1e3, res.requests[k], res.replies[k],
                lost == 0 ? "NONE lost (every missing reply was only late)"
                          : lost > 0 ? (std::to_string(lost) + " NEVER arrived (really lost)").c_str()
                                     : (std::to_string(-lost) + " more replies than requests (unexpected)").c_str());
    if (res.missed[k] > 0 || lost != 0) link_ok = false;
  }
  if (!rtts.empty()) {
    std::vector<double> r = rtts;
    std::sort(r.begin(), r.end());
    auto pct = [&](double p) { return r[std::min(r.size() - 1, static_cast<size_t>(p * r.size()))]; };
    const auto over = std::count_if(r.begin(), r.end(), [&](double v) { return v > period_us; });
    std::printf("  round trip (send all -> all replies): p50 %.0f / p99 %.0f / p99.9 %.0f / max %.0f us, "
                "longer than the period: %lld (%.3f%%)\n",
                pct(0.5), pct(0.99), pct(0.999), r.back(), static_cast<long long>(over), 100.0 * over / r.size());
  }
  std::string can_status;
  bus.bus_off(&can_status);
  const bool can_errors = can_status.find("err=0") == std::string::npos || can_status.find("warn=0") == std::string::npos ||
                          can_status.find("busoff=0") == std::string::npos;
  std::printf("  adapter: %s%s\n", can_status.c_str(), can_errors ? "   <-- CAN errors on the bus" : "");
  if (can_errors) link_ok = false;
  std::printf("  => %s\n", link_ok ? "OK: no lost or late replies, no CAN errors"
                                    : "NOT OK: a healthy bus must show 0 (README: CAN 丢包). Only late = timing (round "
                                      "trip > period); NEVER arrived = the reply was not sent / lost (bus or motor)");
  if (res.status == ControlResult::Status::OutOfLimits && res.failed_index >= 0) print_limits(joints[res.failed_index].motor->id());
  if (!log.empty()) {
    FILE* f = std::fopen(log.c_str(), "w");
    if (!f) {
      std::fprintf(stderr, "cannot write %s\n", log.c_str());
    } else {
      std::fprintf(f, "# mode=%s rate_hz=%.1f timeout_us=%.0f\n", to_string(mode), co.rate_hz, timeout_us);
      std::fprintf(f, "t,id,stale,late,rtt_us,pos_turns,vel_tps,torque_meas_nm,sp_pos_turns,sp_vel_tps,sp_torque_nm,cmd\n");
      for (const auto& s : buf) {
        std::fprintf(f, "%.5f,%u,%d,%d,%.0f,%.6f,%.5f,%.4f,%.6f,%.5f,%.4f,%.6f\n", s.t, joints[s.index].motor->id(),
                     s.stale ? 1 : 0, s.late, s.rtt_us, s.position, s.velocity, s.torque_meas, s.setpoint.position,
                     s.setpoint.velocity, s.setpoint.torque, s.cmd);
      }
      std::fclose(f);
      std::printf("log: %s (%zu rows)\n", log.c_str(), buf.size());
    }
  }
  return res.status == ControlResult::Status::Done || res.status == ControlResult::Status::Interrupted ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  const auto& pos = args.positional();
  if (pos.empty()) {
    std::cerr << "usage: motor_tool list|scan|query|version|stop|reset|timeout <ms>|rezero|set-zero|zero|go-zero|adapter \"<text>\"|status|recover"
                 " [--id 1] [--no-brs]\n";
    return 2;
  }
  const std::string cmd = pos[0];
  // Commands that take no positional argument: refuse a stray word (e.g. "id 1,2" instead of "--id 1,2"),
  // which would otherwise be ignored silently and the default ID used.
  {
    const bool takes_args = cmd == "timeout" || cmd == "adapter" || cmd == "read" || cmd == "write";
    if (!takes_args && pos.size() > 1) {
      std::cerr << "unexpected argument '" << pos[1] << "'";
      if (pos[1].rfind("-", 0) != 0) std::cerr << " (options need two dashes, e.g. --" << pos[1] << ")";
      std::cerr << "\n";
      return 2;
    }
  }

  if (cmd == "list") {
    for (const auto& p : Fdcanusb::detect()) std::cout << p << "\n";
    return 0;
  }

  try {
    Fdcanusb bus(args.str("port"));
    if (cmd == "status" || cmd == "recover") {
      bus.set_debug(args.has("debug"));
      if (cmd == "recover") bus.restart_can();
      std::string status;
      const bool off = bus.bus_off(&status);
      std::cout << status << (off ? "   <-- bus-off: run `motor_tool recover`, and fix termination or use --no-brs" : "")
                << "\n";
      return off ? 1 : 0;
    }
    if (cmd == "adapter") {
      // Raw access: do not touch the adapter settings first, so "conf get" shows what is really there.
      if (pos.size() < 2) {
        std::cerr << "usage: motor_tool adapter \"conf get can.termination\"\n";
        return 2;
      }
      bus.set_debug(args.has("debug"));
      for (const auto& line : bus.command(pos[1])) std::cout << line << "\n";
      return 0;
    }
    cli::configure_bus(bus, args);
    Motor::Options opt;
    // A list (--model A,B) belongs to --id 1,2 and is parsed per motor in run_multi_zero().
    if (args.str("model").find(',') == std::string::npos) opt.model = cli::parse_model(args.str("model"));
    opt.query_full = args.has("full");
    opt.type = cli::parse_type(args.str("type"));  // float (default) / int32 / int16
    if ((cmd == "go-zero" || cmd == "zero") && args.str("id").find(',') != std::string::npos) {
      return run_multi_zero(cmd, bus, args, opt);
    }
    if (cmd == "control") return run_control_cmd(bus, args, opt, args.str("mode"));
    if (cmd == "impedance") return run_control_cmd(bus, args, opt, "impedance");
    const uint8_t id = static_cast<uint8_t>(args.num("id", 1));
    Motor motor(bus, id, opt);

    if (cmd == "scan") {
      int found = 0;
      for (int i = 1; i <= 32; i++) {
        Motor m(bus, static_cast<uint8_t>(i), opt);
        m.options().timeout = std::chrono::milliseconds(10);
        if (auto s = m.query()) {
          std::cout << format_state(*s) << "\n";
          found++;
        }
      }
      std::cout << found << " motor(s) found\n";
      if (!found) cli::explain_no_reply(bus, 0);
      return found ? 0 : 1;
    }
    if (cmd == "query") {
      auto s = motor.query();
      if (!s) {
        cli::explain_no_reply(bus, id);
        return 1;
      }
      std::cout << format_state(*s) << "\n";
      return 0;
    }
    if (cmd == "version") {
      auto v = motor.get_version();
      if (!v) {
        cli::explain_no_reply(bus, id);
        return 1;
      }
      if (!v->reported) {
        std::cout << "ID " << int(id) << ": motor does not support the version register (old firmware; the vendor\n"
                  << "code reports this as 3.9.1). One-to-many compact replies need >= 4.2.3, so use\n"
                  << "test_motor_many --request basic.\n";
        return 0;
      }
      std::cout << "ID " << int(id) << " firmware " << v->major << "." << v->minor << "." << v->patch << "\n";
      return 0;
    }
    if (cmd == "read") {
      if (pos.size() < 2) {
        std::cerr << "usage: motor_tool read <reg> [count]   (reg in hex or decimal, values as float)\n";
        return 2;
      }
      const auto reg = static_cast<uint16_t>(std::strtoul(pos[1].c_str(), nullptr, 0));
      const uint32_t count = pos.size() > 2 ? static_cast<uint32_t>(std::atoi(pos[2].c_str())) : 1;
      const auto values = motor.read_float_registers(reg, count);
      if (!values) {
        std::cout << "no float reply (no answer, or the register is not supported)\n";
        return 1;
      }
      for (uint32_t k = 0; k < values->size(); k++) std::printf("0x%03x = %g\n", reg + k, (*values)[k]);
      return 0;
    }
    if (cmd == "write") {
      if (pos.size() < 3) {
        std::cerr << "usage: motor_tool write <reg> <value> [value...]   (float, RAM only)\n";
        return 2;
      }
      const auto reg = static_cast<uint16_t>(std::strtoul(pos[1].c_str(), nullptr, 0));
      std::vector<double> values;
      for (size_t k = 2; k < pos.size(); k++) values.push_back(std::atof(pos[k].c_str()));
      FrameBuilder fb;
      fb.write(DataType::Float, reg, values, std::vector<Quantity>(values.size(), Quantity::Raw));
      fb.query_state(DataType::Float);
      auto s = motor.transact(motor.frame(fb));
      if (!s) {
        cli::explain_no_reply(bus, id);
        return 1;
      }
      std::cout << format_state(*s) << "\n";
      return s->errors.empty() ? 0 : 1;
    }
    if (cmd == "stop") {
      auto s = motor.stop();
      std::cout << (s ? format_state(*s) : "no reply") << "\n";
      return s ? 0 : 1;
    }
    if (cmd == "reset") {
      motor.reset();
      std::cout << "reset sent\n";
      return 0;
    }
    if (cmd == "timeout") {
      if (pos.size() < 2) {
        std::cerr << "usage: motor_tool timeout <ms>\n";
        return 2;
      }
      const bool ok = motor.set_timeout_ms(static_cast<int16_t>(std::atoi(pos[1].c_str())));
      std::cout << (ok ? "timeout set" : "no reply (the value may still have been applied)") << "\n";
      return 0;
    }
    if (cmd == "set-zero" || cmd == "zero" || cmd == "go-zero") {
      const std::string zfile = args.str("zero-file", default_zero_file());
      if (cmd == "set-zero") {
        // Stop first so the motor is free (mode 0) and nothing is commanded while we read.
        if (!motor.stop()) {
          cli::explain_no_reply(bus, id);
          return 1;
        }
        std::string err;
        auto r = set_zero_here(motor, zfile, 10, &err);
        if (!r) {
          std::cerr << "set-zero failed: " << err << "\n";
          return 1;
        }
        std::printf("zero for motor %u set at raw position %.4f r (%s)\nsaved to %s\n", r->id, r->position,
                    r->time.c_str(), zfile.c_str());
        print_limits(id);
        return 0;
      }
      const auto rec = load_zero(zfile, id);
      if (!rec) {
        std::cerr << "no zero stored for motor " << int(id) << " in " << zfile << " (run `motor_tool set-zero` first)\n";
        return 1;
      }
      if (cmd == "zero") {
        auto s = motor.query();
        if (!s || !s->position) {
          cli::explain_no_reply(bus, id);
          return 1;
        }
        const double zeff = effective_zero(rec->position, *s->position);
        const double off = *s->position - zeff;
        std::printf("stored zero: %.4f r (set %s)\nnow:         %.4f r  ->  %+.4f r from zero (%+.2f deg)\n",
                    rec->position, rec->time.c_str(), *s->position, off, off * 360.0);
        if (std::fabs(zeff - rec->position) > 0.5) {
          std::printf("             (the reading carries %+.0f whole turn(s) relative to the stored zero; ignored)\n",
                      zeff - rec->position);
        }
        if (const auto lim = print_limits(id)) {
          std::printf("now inside limits: %s\n", within_limits(*lim, zeff, *s->position) ? "yes" : "NO");
        }
        return 0;
      }
      // go-zero
      cli::install_ctrl_c();
      const GoToZeroOptions zo = parse_zero_options(args);
      if (zo.check_limits && !wait_until_inside_limits({&motor}, zfile, zo.limit_margin)) return 1;
      const LinkCount lc0 = link_count(bus, id);
      std::printf("going to zero %.4f r (set %s), vel max %.2f r/s, acc %.2f r/s^2, correction %.2f /s (max %.3f r)\n",
                  rec->position, rec->time.c_str(), zo.vel_max, zo.acc, zo.correction_rate, zo.correction_max);
      const auto res = go_to_zero(
          motor, zfile, zo, [] { return !cli::g_stop; },
          [&](const MotorState& st, double error, double command_offset) {
            std::printf("  pos %.4f r   error %+.4f r (%+.2f deg)   vel %+.3f r/s   tqe %+.3f Nm   cmd-zero %+.4f r\n",
                        st.position.value_or(NAN), error, error * 360.0, st.velocity.value_or(NAN),
                        st.torque.value_or(NAN), command_offset);
            std::fflush(stdout);
          });
      if (res.status == GoToZeroResult::Status::NoLimits || res.status == GoToZeroResult::Status::NoZeroStored ||
          res.status == GoToZeroResult::Status::Int16Continuous || (res.status == GoToZeroResult::Status::NoReply && res.elapsed_s == 0)) {
        std::printf("%s\n", to_string(res.status));  // refused before reading the motor
      } else {
        std::printf("%s: start %+.2f deg from zero, final error %+.4f r (%+.2f deg), correction %+.4f r, %.1f s%s\n",
                    to_string(res.status), res.start_offset * 360.0, res.final_error, res.final_error * 360.0,
                    res.correction, res.elapsed_s,
                    res.status == GoToZeroResult::Status::Reached && !zo.stop_at_end ? ", holding (Ctrl-C to release)" : "");
      }
      bus.drain(std::chrono::milliseconds(100));
      const LinkCount lc1 = link_count(bus, id);
      warn_lost_replies(id, res, &lc0, &lc1);
      switch (res.status) {
        case GoToZeroResult::Status::NoLimits:
          std::printf("joint limits for ID %u are not hardcoded (src/zero.cpp, kJointLimits).\n"
                      "--force moves it without limit checks.\n", id);
          break;
        case GoToZeroResult::Status::OutOfLimits:
          print_limits(id);
          std::printf("motor at %+.1f deg from zero. Was it turned by hand while off? Turn it back into the\n"
                      "limits by hand (motor stopped, the way the cables allow), check with `motor_tool zero`,\n"
                      "then try again.\n",
                      res.final_error * 360.0);
          break;
        default:
          break;
      }
      if (res.status == GoToZeroResult::Status::Reached && !zo.stop_at_end) {
        cli::Rate rate(100);
        while (!cli::g_stop) {
          motor.set_pos_velmax_acc(res.command, zo.vel_max, zo.acc);  // zero + correction, inside limits
          rate.sleep();
        }
        motor.stop();
        std::printf("released\n");
      }
      return res.status == GoToZeroResult::Status::Reached ? 0 : 1;
    }
    if (cmd == "rezero") {
      motor.stop();
      const int r = motor.rezero_and_save();
      std::cout << (r == 0 ? "zero set and saved" : r == 1 ? "rezero failed (no OK from motor)" : "save failed") << "\n";
      return r;
    }
    std::cerr << "unknown command " << cmd << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
