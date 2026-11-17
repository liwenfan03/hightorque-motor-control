#include "hightorque/control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace hightorque {

const char* to_string(ControlMode m) {
  switch (m) {
    case ControlMode::Position: return "position";
    case ControlMode::Velocity: return "velocity";
    case ControlMode::Torque: return "torque";
    case ControlMode::Impedance: return "impedance";
  }
  return "?";
}

const char* to_string(ControlResult::Status s) {
  using S = ControlResult::Status;
  switch (s) {
    case S::Done: return "done";
    case S::Interrupted: return "interrupted";
    case S::NoZeroStored: return "no zero stored for this motor";
    case S::NoLimits: return "no joint limits hardcoded for this motor";
    case S::NoReply: return "no reply";
    case S::OutOfLimits: return "outside the joint limits";
    case S::StartTooFar: return "too far from the start setpoint (go-zero first)";
    case S::TooFast: return "faster than safe_vel (--safe-vel)";
    case S::LostReplies: return "lost CAN replies";
    case S::Fault: return "motor reported a fault";
    case S::BadArgument: return "bad argument (gains, limits, setpoint outside the joint limits, or duplicate ID)";
  }
  return "?";
}

double impedance_torque(double pos, double vel, double pos_d, double vel_d, double tau_d, double kp, double kd,
                        double max_torque) {
  const double tau = kp * (pos_d - pos) + kd * (vel_d - vel) + tau_d;
  if (!std::isfinite(tau)) return 0.0;
  const double m = std::fabs(max_torque);
  return std::clamp(tau, -m, m);
}

double SetpointShaper::update(double target, double target_vel, double dt) {
  if (!std::isfinite(target) || !(dt > 0) || !(vel_max > 0) || !(acc > 0)) return pos;
  if (!std::isfinite(target_vel)) target_vel = 0.0;
  target_vel = std::clamp(target_vel, -vel_max, vel_max);
  const double e = target - pos;
  const double dv = acc * dt;
  // close enough to stop on the target in this step: land exactly (no chatter around it)
  if (std::fabs(e) <= std::fabs(vel - target_vel) * dt + 0.5 * dv * dt && std::fabs(vel - target_vel) <= dv) {
    pos = target;
    vel = target_vel;
    return pos;
  }
  // fastest speed from which steps of -acc*dt still stop within |e| (discrete time: v^2/2a + v dt/2 = |e|)
  const double v_brake =
      acc * dt * (-0.5 + std::sqrt(0.25 + 2.0 * std::fabs(e) / (acc * dt * dt))) * (e > 0 ? 1.0 : -1.0);
  const double v_want = std::clamp(target_vel + v_brake, -vel_max, vel_max);
  vel += std::clamp(v_want - vel, -dv, dv);
  pos += vel * dt;
  return pos;
}

double brake_limited_velocity(double v, double pos, double lo, double hi, double acc) {
  if (!std::isfinite(v)) return 0.0;
  const double d = v > 0 ? hi - pos : pos - lo;  // distance to the limit we are moving towards
  if (d <= 0) return 0.0;
  const double vmax = std::sqrt(2.0 * std::fabs(acc) * d);
  return std::clamp(v, -vmax, vmax);
}

namespace {

bool finite_setpoint(const ControlSetpoint& s) {
  return std::isfinite(s.position) && std::isfinite(s.velocity) && std::isfinite(s.torque);
}

}  // namespace

std::vector<bool> brake_then_stop(Fdcanusb& bus, const std::vector<Motor*>& motors, double brake_s) {
  using Clock = std::chrono::steady_clock;
  const size_t n = motors.size();
  // 1. brake until slow (or brake_s)
  if (brake_s > 0) {
    const auto end = Clock::now() + std::chrono::microseconds(static_cast<int64_t>(brake_s * 1e6));
    while (Clock::now() < end) {
      std::vector<std::pair<Motor*, FrameBuilder>> cmds;
      for (Motor* m : motors) cmds.emplace_back(m, m->command_mode(mode::kBrake));
      const auto st = cycle(bus, cmds, std::chrono::milliseconds(20));
      bool slow = true;
      for (size_t i = 0; i < n; i++) {
        if (!st[i] || !st[i]->velocity || std::fabs(*st[i]->velocity) > 0.02) slow = false;
      }
      if (slow) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  // 2. stop (free), until every motor reports mode 0 (up to 3 attempts)
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
    const auto st = cycle(bus, stops, std::chrono::milliseconds(20));
    for (size_t k = 0; k < idx.size(); k++) {
      auto s = st[k];
      if (!s || !s->mode || *s->mode != mode::kStop) s = motors[idx[k]]->query();  // reply lost / late one taken
      if (s && s->mode && *s->mode == mode::kStop) stopped[idx[k]] = true;
    }
  }
  return stopped;
}

ControlResult run_control(Fdcanusb& bus, const std::vector<ControlJoint>& joints, const std::string& zero_path,
                          const ControlTrajectory& trajectory, const ControlOptions& opt,
                          const std::function<bool()>& keep_running,
                          const std::function<void(const ControlSample&)>& on_sample) {
  using Clock = std::chrono::steady_clock;
  using Status = ControlResult::Status;
  ControlResult R;
  const size_t n = joints.size();
  R.missed.assign(n, 0);
  R.late.assign(n, 0);
  R.zero_eff.assign(n, 0.0);
  R.stopped.assign(n, false);
  auto fail = [&](size_t i, Status st) {
    if (R.failed_index < 0) {
      R.status = st;
      R.failed_index = static_cast<int>(i);
    }
  };
  if (n == 0) {
    R.status = Status::Done;
    return R;
  }
  if (!(opt.rate_hz > 0) || !trajectory) {
    R.status = Status::BadArgument;
    return R;
  }

  // ---- checks, no command sent ----
  std::vector<JointLimits> lim(n);
  std::vector<double> pos(n, 0.0), vel(n, 0.0), vcmd(n, 0.0);
  std::vector<int> no_reply(n, 0);
  for (size_t i = 0; i < n; i++) {
    const ControlJoint& j = joints[i];
    auto pos_ok = [](double x) { return std::isfinite(x) && x > 0; };
    auto nan_or_pos = [](double x) { return std::isnan(x) || (std::isfinite(x) && x > 0); };
    auto scale_ok = [](double x) { return std::isfinite(x) && x >= 0 && x <= 1; };
    const PositionModeCommand& r = j.regs;
    bool bad = !j.motor || !std::isfinite(j.kp) || !std::isfinite(j.kd) || j.kp < 0 || j.kd < 0 ||
               !pos_ok(j.max_torque) || !pos_ok(j.vel_ramp) || !(std::isfinite(j.safe_vel) && j.safe_vel >= 0) || !std::isfinite(r.velocity) ||
               !std::isfinite(r.torque) || !scale_ok(r.kp_scale) || !scale_ok(r.kd_scale) ||
               !nan_or_pos(r.max_torque) || !nan_or_pos(r.vel_limit) || !nan_or_pos(r.acc_limit);
    for (size_t k = 0; k < i && !bad; k++) bad = joints[k].motor->id() == j.motor->id();
    if (bad) { fail(i, Status::BadArgument); return R; }
    const auto rec = load_zero(zero_path, j.motor->id());
    if (!rec) { fail(i, Status::NoZeroStored); return R; }
    const auto l = joint_limits(j.motor->id());
    if (!l) { fail(i, Status::NoLimits); return R; }
    lim[i] = *l;
    // regs.stop_position is relative to the zero, like every setpoint
    if (!std::isnan(r.stop_position) && !l->continuous &&
        (r.stop_position < l->min_offset || r.stop_position > l->max_offset)) {
      fail(i, Status::BadArgument);
      return R;
    }
    const ControlSetpoint sp0 = trajectory(i, 0.0);
    if (!finite_setpoint(sp0)) { fail(i, Status::BadArgument); return R; }
    const bool uses_pos = j.mode == ControlMode::Position || j.mode == ControlMode::Impedance;
    if (uses_pos && !l->continuous && (sp0.position < l->min_offset || sp0.position > l->max_offset)) {
      fail(i, Status::BadArgument);
      return R;
    }
    const auto s = j.motor->query();
    if (!s || !s->position) { fail(i, Status::NoReply); return R; }
    R.zero_eff[i] = effective_zero(rec->position, *s->position);
    pos[i] = *s->position - R.zero_eff[i];
    vel[i] = s->velocity.value_or(0.0);
    if (!within_limits(lim[i], R.zero_eff[i], *s->position, opt.limit_margin)) { fail(i, Status::OutOfLimits); return R; }
    if (uses_pos && std::fabs(pos[i] - sp0.position) > opt.max_start_error) { fail(i, Status::StartTooFar); return R; }
  }

  std::vector<uint64_t> req0(n), rep0(n);
  for (size_t i = 0; i < n; i++) {
    req0[i] = bus.requests_sent(joints[i].motor->id());
    rep0[i] = bus.replies_received(joints[i].motor->id());
  }

  // ---- run ----
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / opt.rate_hz));
  const auto t0 = Clock::now();
  auto next = t0;
  auto last = t0;
  std::vector<ControlSample> samples(n);
  while (true) {
    if (keep_running && !keep_running()) { R.status = Status::Interrupted; break; }
    const auto now0 = Clock::now();
    const double t = std::chrono::duration<double>(now0 - t0).count();
    const double dt = std::min(std::chrono::duration<double>(now0 - last).count(), 0.05);
    last = now0;
    if (opt.duration_s > 0 && t >= opt.duration_s) { R.status = Status::Done; break; }

    std::vector<std::pair<Motor*, FrameBuilder>> cmds;
    cmds.reserve(n);
    const bool full = opt.resend_every <= 1 || R.cycles % opt.resend_every == 0;
    for (size_t i = 0; i < n; i++) {
      const ControlJoint& j = joints[i];
      ControlSample& smp = samples[i];
      smp = ControlSample{};
      smp.t = t;
      smp.index = i;
      smp.setpoint = trajectory(i, t);
      if (!finite_setpoint(smp.setpoint)) smp.setpoint = ControlSetpoint{pos[i], 0.0, 0.0};
      const ControlSetpoint& sp = smp.setpoint;
      switch (j.mode) {
        case ControlMode::Position: {
          PositionModeCommand c = j.regs;
          c.position = clamp_to_limits(lim[i], R.zero_eff[i], R.zero_eff[i] + sp.position);
          if (j.feed_forward_velocity) c.velocity = sp.velocity;
          if (!std::isnan(c.stop_position)) c.stop_position += R.zero_eff[i];
          smp.cmd = c.position - R.zero_eff[i];
          cmds.emplace_back(j.motor, full ? j.motor->command_position_mode(c) : j.motor->command_position_mode_changed(c));
          break;
        }
        case ControlMode::Velocity: {
          // host ramp, then the braking envelope towards the limit we are moving to
          double v = std::clamp(sp.velocity, vcmd[i] - j.vel_ramp * dt, vcmd[i] + j.vel_ramp * dt);
          if (!lim[i].continuous) {
            v = brake_limited_velocity(v, pos[i], lim[i].min_offset + opt.brake_margin,
                                       lim[i].max_offset - opt.brake_margin, j.vel_ramp);
          }
          vcmd[i] = v;
          smp.cmd = v;
          PositionModeCommand c = j.regs;
          c.position = std::numeric_limits<double>::quiet_NaN();
          c.velocity = v;
          if (!std::isnan(c.stop_position)) c.stop_position += R.zero_eff[i];
          cmds.emplace_back(j.motor, full ? j.motor->command_position_mode(c) : j.motor->command_position_mode_changed(c));
          break;
        }
        case ControlMode::Torque:
        case ControlMode::Impedance: {
          const bool imp = j.mode == ControlMode::Impedance;
          const double g = imp && opt.gain_ramp_s > 0 ? std::min(t / opt.gain_ramp_s, 1.0) : 1.0;
          const double tau = impedance_torque(pos[i], vel[i], sp.position, imp ? sp.velocity : 0.0, sp.torque,
                                              imp ? g * j.kp : 0.0, g * j.kd, j.max_torque);
          smp.cmd = tau;
          cmds.emplace_back(j.motor, j.motor->command_tqe(tau));
          break;
        }
      }
    }
    std::vector<uint64_t> late0(n);
    for (size_t i = 0; i < n; i++) late0[i] = bus.late_replies(joints[i].motor->id());
    const auto tc = Clock::now();
    std::chrono::microseconds to{0};
    for (const auto& j : joints) to = std::max(to, j.motor->options().timeout);
    if (R.cycles < opt.startup_cycles) {
      to = std::max(to, std::chrono::microseconds(static_cast<int64_t>(opt.startup_timeout_s * 1e6)));
    }
    const auto states = cycle(bus, cmds, to);
    const double rtt_us = std::chrono::duration<double, std::micro>(Clock::now() - tc).count();
    R.cycles++;

    for (size_t i = 0; i < n; i++) {
      const auto& s = states[i];
      ControlSample& smp = samples[i];
      smp.rtt_us = rtt_us;
      smp.late = static_cast<int>(bus.late_replies(joints[i].motor->id()) - late0[i]);
      R.late[i] += smp.late;
      if (s && s->position) {
        no_reply[i] = 0;
        pos[i] = *s->position - R.zero_eff[i];
        vel[i] = s->velocity.value_or(0.0);
        smp.torque_meas = s->torque.value_or(0.0);  // Motor::decode() already applied k
        if (s->fault && *s->fault != 0) fail(i, Status::Fault);
        if (!within_limits(lim[i], R.zero_eff[i], *s->position, opt.limit_margin)) fail(i, Status::OutOfLimits);
        if (joints[i].safe_vel > 0 && std::fabs(vel[i]) > joints[i].safe_vel) fail(i, Status::TooFast);
      } else {
        smp.stale = true;
        R.missed[i]++;
        if (++no_reply[i] >= opt.max_missed_in_row) fail(i, Status::LostReplies);
      }
      smp.position = pos[i];
      smp.velocity = vel[i];
      if (on_sample) on_sample(smp);
    }
    if (R.failed_index >= 0) break;

    next += period;
    const auto now = Clock::now();
    if (now > next) {  // late: do not try to catch up with a burst of cycles
      R.overruns++;
      next = now;
    } else {
      std::this_thread::sleep_until(next);
    }
  }
  R.elapsed_s = std::chrono::duration<double>(Clock::now() - t0).count();
  {
    std::vector<Motor*> ms;
    for (const auto& j : joints) ms.push_back(j.motor);
    R.stopped = brake_then_stop(bus, ms, opt.brake_s);
  }
  bus.drain(std::chrono::microseconds(static_cast<int64_t>(opt.drain_s * 1e6)));
  R.requests.assign(n, 0);
  R.replies.assign(n, 0);
  for (size_t i = 0; i < n; i++) {
    R.requests[i] = static_cast<long long>(bus.requests_sent(joints[i].motor->id()) - req0[i]);
    R.replies[i] = static_cast<long long>(bus.replies_received(joints[i].motor->id()) - rep0[i]);
  }
  return R;
}

}  // namespace hightorque
