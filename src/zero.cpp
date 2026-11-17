#include "hightorque/zero.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

namespace hightorque {

namespace {

std::string now_string() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
  return buf;
}

// Line format:  id=2 position=5.302012 time=2026-11-05 14:03:00
std::optional<ZeroRecord> parse_line(const std::string& line) {
  if (line.empty() || line[0] == '#') return std::nullopt;
  ZeroRecord r;
  const auto id_pos = line.find("id=");
  const auto p_pos = line.find("position=");
  if (id_pos == std::string::npos || p_pos == std::string::npos) return std::nullopt;
  char* end = nullptr;
  const long id = std::strtol(line.c_str() + id_pos + 3, &end, 10);
  if (end == line.c_str() + id_pos + 3 || id < 0 || id > 127) return std::nullopt;
  const double p = std::strtod(line.c_str() + p_pos + 9, &end);
  if (end == line.c_str() + p_pos + 9 || !std::isfinite(p)) return std::nullopt;
  r.id = static_cast<uint8_t>(id);
  r.position = p;
  const auto t_pos = line.find("time=");
  if (t_pos != std::string::npos) r.time = line.substr(t_pos + 5);
  return r;
}

std::string format_line(const ZeroRecord& r) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "id=%u position=%.6f time=", r.id, r.position);
  return std::string(buf) + r.time;
}

std::string parent_dir(const std::string& path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? std::string() : path.substr(0, slash);
}

}  // namespace

double wrap_turns(double x) { return x - std::floor(x + 0.5); }

double effective_zero(double zero_raw, double pos_raw) { return pos_raw - wrap_turns(pos_raw - zero_raw); }

// ---- hardcoded software joint limits ----
//
// Offsets relative to the zero, in turns.  Must stay strictly inside +-0.5 (see zero.hpp).
//   ID 2, 5047 joint: +-90 deg of the 5047 flange (belt drive 1:1, so also +-90 deg of the joint).
//   ID 1, 4438 EE joint: +-170 deg.  No cable crosses this joint (the 4438 body rides on the 5047 side,
//                     the flange turns the EE), so it could turn without end, but the range is bounded
//                     on purpose so the 9-DOF controller has a bounded joint (2026-11-10).  +-10 deg
//                     around 180 is excluded: near 180 deg the reading cannot tell the side.
//                     Continuous alternative: {1, "...", true, 0.0, 0.0}.
namespace {
const JointLimits kJointLimits[] = {
    {2, "5047 joint (ID 2)", false, -90.0 / 360.0, +90.0 / 360.0},
    {1, "4438 EE joint (ID 1)", false, -170.0 / 360.0, +170.0 / 360.0},
};
}  // namespace

std::optional<JointLimits> joint_limits(uint8_t id) {
  for (const auto& l : kJointLimits) {
    if (l.id == id) return l;
  }
  return std::nullopt;
}

bool within_limits(const JointLimits& lim, double zero_eff, double pos_raw, double margin) {
  if (lim.continuous) return true;
  const double off = pos_raw - zero_eff;
  return off >= lim.min_offset - margin && off <= lim.max_offset + margin;
}

double clamp_to_limits(const JointLimits& lim, double zero_eff, double target_raw) {
  if (lim.continuous) return target_raw;
  return std::clamp(target_raw, zero_eff + lim.min_offset, zero_eff + lim.max_offset);
}

std::string default_zero_file() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/.hightorque/zero_positions.txt";
}

std::optional<ZeroRecord> load_zero(const std::string& path, uint8_t id) {
  std::ifstream in(path);
  if (!in) return std::nullopt;
  std::optional<ZeroRecord> found;
  std::string line;
  while (std::getline(in, line)) {
    auto r = parse_line(line);
    if (r && r->id == id) found = r;  // last line for this ID wins
  }
  return found;
}

bool save_zero(const std::string& path, const ZeroRecord& record, std::string* error) {
  std::vector<std::string> keep;
  {
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
      auto r = parse_line(line);
      if (r && r->id == record.id) continue;  // replaced below
      if (!line.empty()) keep.push_back(line);
    }
  }
  const std::string dir = parent_dir(path);
  if (!dir.empty()) ::mkdir(dir.c_str(), 0755);  // ok if it already exists
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
      if (error) *error = "cannot write " + tmp;
      return false;
    }
    if (keep.empty() || keep[0].rfind("#", 0) != 0) {
      out << "# hightorque host-side zero positions (raw motor position in turns). One line per motor ID.\n";
    }
    for (const auto& l : keep) out << l << "\n";
    out << format_line(record) << "\n";
    if (!out) {
      if (error) *error = "write failed: " + tmp;
      return false;
    }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    if (error) *error = "cannot rename " + tmp + " to " + path;
    return false;
  }
  return true;
}

std::optional<ZeroRecord> set_zero_here(Motor& motor, const std::string& path, int samples, std::string* error) {
  double sum = 0;
  int n = 0;
  for (int i = 0; i < std::max(samples, 1); i++) {
    auto s = motor.query();
    if (s && s->position) {
      sum += *s->position;
      n++;
    }
  }
  if (n == 0) {
    if (error) *error = "no reply from motor";
    return std::nullopt;
  }
  ZeroRecord r;
  r.id = motor.id();
  r.position = sum / n;
  r.time = now_string();
  if (!save_zero(path, r, error)) return std::nullopt;
  return r;
}

const char* to_string(GoToZeroResult::Status s) {
  switch (s) {
    case GoToZeroResult::Status::Reached: return "reached";
    case GoToZeroResult::Status::NoZeroStored: return "no zero stored for this motor";
    case GoToZeroResult::Status::NoReply: return "no reply from motor";
    case GoToZeroResult::Status::NoLimits: return "no hardcoded joint limits for this motor ID (not moved)";
    case GoToZeroResult::Status::OutOfLimits: return "motor is outside its joint limits";
    case GoToZeroResult::Status::Int16Continuous:
      return "continuous joint with the int16 wire type (positions saturate at +-3.28 r); use float (not moved)";
    case GoToZeroResult::Status::Timeout: return "timeout";
    case GoToZeroResult::Status::Interrupted: return "interrupted";
  }
  return "?";
}

GoToZeroResult go_to_zero(Motor& motor, const std::string& path, const GoToZeroOptions& opt,
                          const std::function<bool()>& keep_running,
                          const std::function<void(const MotorState&, double error, double command_offset)>& progress) {
  using Clock = std::chrono::steady_clock;
  GoToZeroResult res;
  const auto rec = load_zero(path, motor.id());
  if (!rec) {
    res.status = GoToZeroResult::Status::NoZeroStored;
    return res;
  }
  std::optional<JointLimits> lim;
  if (opt.check_limits) {
    lim = joint_limits(motor.id());
    if (!lim) {
      res.status = GoToZeroResult::Status::NoLimits;
      return res;
    }
  }
  if (const auto l = joint_limits(motor.id()); l && l->continuous && motor.options().type == DataType::Int16) {
    res.status = GoToZeroResult::Status::Int16Continuous;
    return res;
  }

  // A query is harmless, so retry a few times.  WORKAROUND for the unterminated bus (4438 end, 2026-11):
  // the retries are counted and reported; on a terminated bus there must be none (README).
  auto first = motor.query();
  for (int i = 0; i < 4 && (!first || !first->position); i++) {
    res.start_query_retries++;
    first = motor.query();
  }
  if (!first || !first->position) {
    res.status = GoToZeroResult::Status::NoReply;
    return res;
  }
  res.start = *first->position;
  // The zero in the motor's current frame (the reading may carry whole turns, e.g. after a power cycle).
  // The motor keeps this frame while it stays powered, so it is computed once.
  res.target = effective_zero(rec->position, res.start);
  res.start_offset = res.start - res.target;
  res.final_error = res.start_offset;
  if (lim && !within_limits(*lim, res.target, res.start, opt.limit_margin)) {
    res.status = GoToZeroResult::Status::OutOfLimits;
    return res;
  }

  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / opt.rate_hz));
  const auto t0 = Clock::now();
  auto next = t0;
  auto last_progress = t0;
  std::optional<Clock::time_point> in_tol_since;
  int no_reply = 0;
  auto last_sample = t0;
  res.command = res.target;

  for (;;) {
    if (keep_running && !keep_running()) {
      res.status = GoToZeroResult::Status::Interrupted;
      break;
    }
    // Commanded position = zero + host-side correction, never outside the joint limits.
    res.command = res.target + res.correction;
    if (lim) res.command = clamp_to_limits(*lim, res.target, res.command);
    auto s = motor.set_pos_velmax_acc(res.command, opt.vel_max, opt.acc);
    res.cycles++;
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last_sample).count();
    last_sample = now;
    res.elapsed_s = std::chrono::duration<double>(now - t0).count();
    if (s && s->position) {
      no_reply = 0;
      res.final_error = *s->position - res.target;
      if (lim && !within_limits(*lim, res.target, *s->position, opt.limit_margin)) {
        res.status = GoToZeroResult::Status::OutOfLimits;
        break;
      }
      const bool slow = !s->velocity || std::fabs(*s->velocity) < 0.01;
      const double err = res.final_error;
      if (std::fabs(err) <= opt.tolerance && slow) {
        if (!in_tol_since) in_tol_since = now;
        if (now - *in_tol_since >= std::chrono::duration<double>(opt.settle_s)) {
          res.status = GoToZeroResult::Status::Reached;
          break;
        }
      } else {
        in_tol_since.reset();
      }
      // Slow integral on the commanded target.  Only near the zero and while (almost) still, so it
      // does not wind up during the move.  It keeps working inside the tolerance until the error is
      // below half of it, so the result does not stop right at the tolerance edge.  Bounded.
      const bool still = !s->velocity || std::fabs(*s->velocity) < 0.02;
      if (opt.correction_rate > 0 && still && std::fabs(err) < opt.correction_capture &&
          std::fabs(err) > 0.5 * opt.tolerance) {
        res.correction = std::clamp(res.correction - opt.correction_rate * err * std::min(dt, 0.05),
                                    -opt.correction_max, opt.correction_max);
      }
      if (progress && now - last_progress >= std::chrono::milliseconds(500)) {
        last_progress = now;
        progress(*s, res.final_error, res.command - res.target);
      }
    } else {
      if (res.first_missed_cycle < 0) res.first_missed_cycle = res.cycles - 1;
      res.missed_replies++;
      if (++no_reply > 20) {
        res.status = GoToZeroResult::Status::NoReply;
        break;
      }
    }
    if (res.elapsed_s >= opt.timeout_s) {
      res.status = GoToZeroResult::Status::Timeout;
      break;
    }
    next += period;
    std::this_thread::sleep_until(next);
  }

  if (res.status != GoToZeroResult::Status::Reached || opt.stop_at_end) {
    auto s = motor.stop();
    if (s && s->position) res.final_error = *s->position - res.target;
  }
  return res;
}

double min_jerk(double tau) {
  const double t = std::clamp(tau, 0.0, 1.0);
  return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

double min_jerk_duration(double distance, double vel_max, double acc) {
  const double d = std::fabs(distance);
  if (d <= 0.0) return 0.0;
  double t = 0.0;
  if (vel_max > 0) t = std::max(t, 1.875 * d / vel_max);
  if (acc > 0) t = std::max(t, std::sqrt(5.7735 * d / acc));
  return t;
}

GoToZeroManyResult go_to_zero_many(
    Fdcanusb& bus, const std::vector<Motor*>& motors, const std::string& path, const GoToZeroOptions& opt,
    const std::function<bool()>& keep_running,
    const std::function<void(size_t, const MotorState&, double, double)>& progress) {
  using Clock = std::chrono::steady_clock;
  using Status = GoToZeroResult::Status;
  GoToZeroManyResult R;
  const size_t n = motors.size();
  R.motors.resize(n);
  if (n == 0) {
    R.status = Status::Reached;
    return R;
  }
  auto fail = [&](size_t i, Status st) {
    if (R.failed_index < 0) {
      R.status = st;
      R.failed_index = static_cast<int>(i);
    }
    R.motors[i].status = st;
  };

  struct Per {
    std::optional<JointLimits> lim;
    double dist = 0.0;
    double vcmd = 0.0, acmd = 0.0;  // limits given to the motor's own planner while it follows our targets
    int no_reply = 0;
    bool done = false;
    std::optional<Clock::time_point> in_tol_since;
    std::optional<MotorState> last;
  };
  std::vector<Per> P(n);

  // ---- checks, no command sent ----
  for (size_t i = 0; i < n; i++) {
    Motor& m = *motors[i];
    for (size_t j = 0; j < i; j++) {
      if (motors[j]->id() == m.id()) {
        fail(i, Status::NoReply);  // duplicate ID: replies cannot be told apart
        return R;
      }
    }
    const auto rec = load_zero(path, m.id());
    if (!rec) { fail(i, Status::NoZeroStored); return R; }
    if (opt.check_limits) {
      P[i].lim = joint_limits(m.id());
      if (!P[i].lim) { fail(i, Status::NoLimits); return R; }
    }
    if (const auto l = joint_limits(m.id()); l && l->continuous && m.options().type == DataType::Int16) {
      fail(i, Status::Int16Continuous);
      return R;
    }
    R.motors[i].target = rec->position;  // replaced by the effective zero below
  }
  for (size_t i = 0; i < n; i++) {
    Motor& m = *motors[i];
    GoToZeroResult& r = R.motors[i];
    auto first = m.query();
    for (int k = 0; k < 4 && (!first || !first->position); k++) {  // counted workaround, see README
      r.start_query_retries++;
      first = m.query();
    }
    if (!first || !first->position) { fail(i, Status::NoReply); return R; }
    r.start = *first->position;
    r.target = effective_zero(r.target, r.start);
    r.start_offset = r.start - r.target;
    r.final_error = r.start_offset;
    r.command = r.start;
    P[i].last = first;
    if (P[i].lim && !within_limits(*P[i].lim, r.target, r.start, opt.limit_margin)) {
      fail(i, Status::OutOfLimits);
      return R;
    }
  }

  // ---- plan: one duration for all ----
  double T = 0.0;
  for (size_t i = 0; i < n; i++) {
    P[i].dist = std::fabs(R.motors[i].target - R.motors[i].start);
    T = std::max(T, min_jerk_duration(P[i].dist, opt.vel_max, opt.acc));
  }
  R.planned_s = T;
  for (size_t i = 0; i < n; i++) {
    const double peak_v = T > 0 ? 1.875 * P[i].dist / T : 0.0;
    const double peak_a = T > 0 ? 5.7735 * P[i].dist / (T * T) : 0.0;
    // Some headroom so the motor's planner does not lag behind our trajectory, never more than 1.5x the
    // user limits; a small floor for the final correction phase.
    P[i].vcmd = std::min(std::max(1.5 * peak_v, 0.05), 1.5 * opt.vel_max);
    P[i].acmd = std::min(std::max(1.5 * peak_a, 0.2), 1.5 * opt.acc);
  }

  // ---- run ----
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / opt.rate_hz));
  const auto t0 = Clock::now();
  auto next = t0;
  auto last_progress = t0;
  auto last_sample = t0;
  bool finished = false;

  while (!finished) {
    if (keep_running && !keep_running()) {
      R.status = Status::Interrupted;
      break;
    }
    const auto now0 = Clock::now();
    const double t = std::chrono::duration<double>(now0 - t0).count();
    const double tau = T > 0 ? std::min(t / T, 1.0) : 1.0;
    const double sv = min_jerk(tau);

    std::vector<std::pair<Motor*, FrameBuilder>> cmds;
    cmds.reserve(n);
    for (size_t i = 0; i < n; i++) {
      GoToZeroResult& r = R.motors[i];
      double c = r.start + sv * (r.target - r.start);
      if (tau >= 1.0) c = r.target + r.correction;
      if (P[i].lim) c = clamp_to_limits(*P[i].lim, r.target, c);
      r.command = c;
      cmds.emplace_back(motors[i], motors[i]->command_pos_velmax_acc(c, P[i].vcmd, P[i].acmd));
    }
    const auto states = cycle(bus, cmds);
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last_sample).count();
    last_sample = now;
    R.elapsed_s = std::chrono::duration<double>(now - t0).count();

    bool all_done = true;
    for (size_t i = 0; i < n; i++) {
      GoToZeroResult& r = R.motors[i];
      Per& p = P[i];
      r.cycles++;
      r.elapsed_s = R.elapsed_s;
      const auto& s = states[i];
      if (s && s->position) {
        p.no_reply = 0;
        p.last = s;
        const double err = *s->position - r.target;
        r.final_error = err;
        if (p.lim && !within_limits(*p.lim, r.target, *s->position, opt.limit_margin)) {
          fail(i, Status::OutOfLimits);
          continue;
        }
        if (tau >= 1.0) {
          const bool slow = !s->velocity || std::fabs(*s->velocity) < 0.01;
          if (std::fabs(err) <= opt.tolerance && slow) {
            if (!p.in_tol_since) p.in_tol_since = now;
            if (now - *p.in_tol_since >= std::chrono::duration<double>(opt.settle_s)) p.done = true;
          } else if (!p.done) {
            p.in_tol_since.reset();
          }
          const bool still = !s->velocity || std::fabs(*s->velocity) < 0.02;
          if (opt.correction_rate > 0 && still && std::fabs(err) < opt.correction_capture &&
              std::fabs(err) > 0.5 * opt.tolerance) {
            r.correction = std::clamp(r.correction - opt.correction_rate * err * std::min(dt, 0.05),
                                      -opt.correction_max, opt.correction_max);
          }
        }
      } else {
        if (r.first_missed_cycle < 0) r.first_missed_cycle = r.cycles - 1;
        r.missed_replies++;
        if (++p.no_reply > 20) fail(i, Status::NoReply);
      }
      if (!p.done) all_done = false;
    }
    if (R.failed_index >= 0) break;
    if (all_done) {
      R.status = Status::Reached;
      for (auto& r : R.motors) r.status = Status::Reached;
      break;
    }
    if (R.elapsed_s >= T + opt.timeout_s) {
      for (size_t i = 0; i < n; i++) {
        if (!P[i].done) { fail(i, Status::Timeout); break; }
      }
      break;
    }
    if (progress && now - last_progress >= std::chrono::milliseconds(500)) {
      last_progress = now;
      for (size_t i = 0; i < n; i++) {
        if (P[i].last && P[i].last->position) {
          progress(i, *P[i].last, *P[i].last->position - R.motors[i].target,
                   R.motors[i].command - R.motors[i].target);
        }
      }
    }
    next += period;
    std::this_thread::sleep_until(next);
  }
  // Motors that did not cause the end get the overall reason (they were stopped because of it).
  for (size_t i = 0; i < n; i++) {
    if (R.failed_index != static_cast<int>(i)) R.motors[i].status = R.status;
  }

  // ---- stop all (unless holding after success) ----
  if (R.status != Status::Reached || opt.stop_at_end) {
    std::vector<bool> stopped(n, false);
    for (int attempt = 0; attempt < 3; attempt++) {
      std::vector<std::pair<Motor*, FrameBuilder>> stops;
      std::vector<size_t> idx;
      for (size_t i = 0; i < n; i++) {
        if (!stopped[i]) {
          stops.emplace_back(motors[i], motors[i]->command_mode(mode::kStop));
          idx.push_back(i);
        }
      }
      if (stops.empty()) break;
      const auto st = cycle(bus, stops);
      for (size_t k = 0; k < idx.size(); k++) {
        const size_t i = idx[k];
        auto s = st[k];
        if (!s) s = motors[i]->query();  // the stop may have arrived even if its reply was lost
        if (s && s->mode && *s->mode == mode::kStop) {
          stopped[i] = true;
          if (s->position) R.motors[i].final_error = *s->position - R.motors[i].target;
        }
      }
    }
  }
  return R;
}

}  // namespace hightorque
