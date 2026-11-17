#include "hightorque/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <thread>

namespace hightorque {

namespace {

using Clock = std::chrono::steady_clock;

}  // namespace

PipelineResult run_position_pipelined(Fdcanusb& bus, const std::vector<ControlJoint>& joints,
                                      const std::string& zero_path, const ControlTrajectory& trajectory,
                                      const PipelineOptions& opt, const std::function<bool()>& keep_running,
                                      const std::function<void(const PipelineSample&)>& on_sample) {
  return run_control_pipelined(bus, joints, zero_path, trajectory, opt, keep_running, on_sample);
}

PipelineResult run_control_pipelined(Fdcanusb& bus, const std::vector<ControlJoint>& joints,
                                     const std::string& zero_path, const ControlTrajectory& trajectory,
                                     const PipelineOptions& opt, const std::function<bool()>& keep_running,
                                     const std::function<void(const PipelineSample&)>& on_sample) {
  using Status = ControlResult::Status;
  PipelineResult R;
  const size_t n = joints.size();
  R.zero_eff.assign(n, 0.0);
  R.stopped.assign(n, false);
  R.requests.assign(n, 0);
  R.replies.assign(n, 0);
  R.age_hist.assign(n, std::vector<int>(10, 0));
  R.not_fresh.assign(n, 0);
  R.max_stale_seen.assign(n, 0);
  R.lost_detected.assign(n, 0);
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
  if (!(opt.rate_hz > 0) || !trajectory || opt.max_stale < 1 || opt.max_outstanding < 1) {
    R.status = Status::BadArgument;
    return R;
  }

  // ---- checks, no command sent (same as run_control) ----
  std::vector<JointLimits> lim(n);
  std::vector<double> pos(n, 0.0), vel(n, 0.0), tqe(n, 0.0);
  for (size_t i = 0; i < n; i++) {
    const ControlJoint& j = joints[i];
    auto nan_or_pos = [](double x) { return std::isnan(x) || (std::isfinite(x) && x > 0); };
    auto scale_ok = [](double x) { return std::isfinite(x) && x >= 0 && x <= 1; };
    const PositionModeCommand& r = j.regs;
    auto pos_ok = [](double x) { return std::isfinite(x) && x > 0; };
    const bool regs_mode = j.mode == ControlMode::Position || j.mode == ControlMode::Velocity;
    bool bad = !j.motor || !(std::isfinite(j.safe_vel) && j.safe_vel >= 0) || !std::isfinite(j.kp) ||
               !std::isfinite(j.kd) || j.kp < 0 || j.kd < 0 || !pos_ok(j.max_torque) || !pos_ok(j.vel_ramp);
    if (!bad && regs_mode) {
      bad = !std::isfinite(r.velocity) || !std::isfinite(r.torque) || !scale_ok(r.kp_scale) || !scale_ok(r.kd_scale) ||
            !nan_or_pos(r.max_torque) || !nan_or_pos(r.vel_limit) || !nan_or_pos(r.acc_limit) ||
            (j.mode == ControlMode::Position && !std::isnan(r.stop_position));
    }
    for (size_t k = 0; k < i && !bad; k++) bad = joints[k].motor->id() == j.motor->id();
    if (bad) { fail(i, Status::BadArgument); return R; }
    const auto rec = load_zero(zero_path, j.motor->id());
    if (!rec) { fail(i, Status::NoZeroStored); return R; }
    const auto l = joint_limits(j.motor->id());
    if (!l) { fail(i, Status::NoLimits); return R; }
    lim[i] = *l;
    const bool uses_pos = j.mode == ControlMode::Position || j.mode == ControlMode::Impedance;
    const ControlSetpoint sp0 = trajectory(i, 0.0);
    if (!std::isfinite(sp0.position) || !std::isfinite(sp0.velocity) || !std::isfinite(sp0.torque) ||
        (uses_pos && !l->continuous && (sp0.position < l->min_offset || sp0.position > l->max_offset)) ||
        (j.mode == ControlMode::Velocity && !std::isnan(r.stop_position) && !l->continuous &&
         (r.stop_position < l->min_offset || r.stop_position > l->max_offset))) {
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

  // ---- run ----
  bus.drain(std::chrono::milliseconds(5));  // nothing old may be taken as an answer
  std::vector<uint64_t> req0(n), rep0(n);
  for (size_t i = 0; i < n; i++) {
    req0[i] = bus.requests_sent(joints[i].motor->id());
    rep0[i] = bus.replies_received(joints[i].motor->id());
  }
  const uint64_t err0 = bus.adapter_errors();

  // per motor: send time and cycle of every request still waiting for its reply (FIFO)
  struct Pending {
    int cycle;
    Clock::time_point sent;
  };
  std::vector<std::deque<Pending>> pending(n);
  std::vector<int> last_fresh_cycle(n, -1), state_age(n, 0);
  std::vector<double> state_latency(n, 0.0);
  std::vector<bool> fresh(n, false);

  // ---- warm-up: a few blocking cycles with the first setpoint (mode switch, register reset) ----
  for (int w = 0; w < opt.warmup_cycles && R.failed_index < 0; w++) {
    std::vector<std::pair<Motor*, FrameBuilder>> cmds;
    for (size_t i = 0; i < n; i++) {
      const ControlJoint& j = joints[i];
      PositionModeCommand c = j.regs;
      const ControlSetpoint sp0 = trajectory(i, 0.0);
      switch (j.mode) {
        case ControlMode::Position:
          // host shaping: hold where the joint is (the shaper then moves to the setpoint within the limits)
          c.position = clamp_to_limits(lim[i], R.zero_eff[i], R.zero_eff[i] + (opt.host_shaping ? pos[i] : sp0.position));
          if (opt.host_shaping) {
            c.vel_limit = std::numeric_limits<double>::quiet_NaN();
            c.acc_limit = std::numeric_limits<double>::quiet_NaN();
          } else if (j.feed_forward_velocity) {
            c.velocity = sp0.velocity;
          }
          cmds.emplace_back(j.motor, j.motor->command_position_mode(c));
          break;
        case ControlMode::Velocity:  // velocity 0 (the host ramp starts from 0)
          c.position = std::numeric_limits<double>::quiet_NaN();
          c.velocity = 0.0;
          if (!std::isnan(c.stop_position)) c.stop_position += R.zero_eff[i];
          cmds.emplace_back(j.motor, j.motor->command_position_mode(c));
          break;
        case ControlMode::Torque:
        case ControlMode::Impedance:  // zero torque; the full frame also resets 0x025 once
          cmds.emplace_back(j.motor, j.motor->command_tqe(0.0));
          break;
      }
    }
    const auto st = cycle(bus, cmds, std::chrono::milliseconds(20));
    for (size_t i = 0; i < n; i++) {
      if (!st[i] || !st[i]->position) {
        fail(i, Status::NoReply);
        continue;
      }
      pos[i] = *st[i]->position - R.zero_eff[i];
      vel[i] = st[i]->velocity.value_or(0.0);
      tqe[i] = st[i]->torque.value_or(0.0);
      if (st[i]->fault && *st[i]->fault != 0) fail(i, Status::Fault);
    }
  }
  // host shaping (see PipelineOptions::host_shaping): limits from the registers, removed from the frames
  std::vector<SetpointShaper> shaper(n);
  std::vector<PositionModeCommand> regs(n);
  std::vector<double> vcmd(n, 0.0);  // velocity mode: host-ramped command
  for (size_t i = 0; i < n; i++) {
    regs[i] = joints[i].regs;
    if (opt.host_shaping && joints[i].mode == ControlMode::Position) {
      shaper[i].vel_max = std::isnan(regs[i].vel_limit) ? 0.3 : regs[i].vel_limit;
      shaper[i].acc = std::isnan(regs[i].acc_limit) ? 1.0 : regs[i].acc_limit;
      shaper[i].reset(pos[i]);  // start from where the joint is (after warm-up)
      regs[i].vel_limit = std::numeric_limits<double>::quiet_NaN();
      regs[i].acc_limit = std::numeric_limits<double>::quiet_NaN();
    }
  }
  const double dt = 1.0 / opt.rate_hz;
  const auto min_lat = std::chrono::microseconds(static_cast<int64_t>(opt.min_latency_us));
  const auto realign = std::chrono::microseconds(static_cast<int64_t>(opt.realign_window_s * 1e6));

  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / opt.rate_hz));
  const auto t0 = Clock::now();
  std::vector<Clock::time_point> clean_since(n, t0);  // last time no request older than min_lat was open
  auto next = t0;  // scheduled time of the next send
  std::vector<PipelineSample> samples(n);
  std::vector<CanFrame> rx;

  for (int k = 0; R.failed_index < 0; k++) {
    // ---- send cycle k at its scheduled time, whatever the replies did ----
    if (keep_running && !keep_running()) { R.status = Status::Interrupted; break; }
    // never earlier than the schedule (the motors drop requests that come too close together)
    if (Clock::now() < next) std::this_thread::sleep_until(next);
    const auto now = Clock::now();
    const double t = std::chrono::duration<double>(next - t0).count();  // scheduled time: no drift
    if (opt.duration_s > 0 && t >= opt.duration_s) { R.status = Status::Done; break; }
    const double late_us = std::chrono::duration<double, std::micro>(now - next).count();
    R.max_send_late_us = std::max(R.max_send_late_us, late_us);
    if (late_us > 0.25e6 / opt.rate_hz) R.late_sends++;  // > 25% of a period (macOS jitter ~0.1 ms is normal)

    const bool full = opt.resend_every <= 1 || k % opt.resend_every == 0;
    std::vector<CanFrame> tx;
    tx.reserve(n);
    for (size_t i = 0; i < n; i++) {
      const ControlJoint& j = joints[i];
      PipelineSample& smp = samples[i];
      smp = PipelineSample{};
      smp.t = t;
      smp.cycle = k;
      smp.index = i;
      smp.setpoint = trajectory(i, t);
      if (!std::isfinite(smp.setpoint.position)) smp.setpoint = ControlSetpoint{pos[i], 0.0, 0.0};
      smp.send_late_us = late_us;
      const ControlSetpoint& sp = smp.setpoint;
      switch (j.mode) {
        case ControlMode::Position: {
          PositionModeCommand c = regs[i];
          double target = sp.position;
          double target_vel = sp.velocity;
          if (opt.host_shaping) {
            const double lo = lim[i].continuous ? -1e9 : lim[i].min_offset, hi = lim[i].continuous ? 1e9 : lim[i].max_offset;
            target = shaper[i].update(std::clamp(target, lo, hi), target_vel, dt);
            target_vel = shaper[i].vel;
          }
          c.position = clamp_to_limits(lim[i], R.zero_eff[i], R.zero_eff[i] + target);
          if (j.feed_forward_velocity) c.velocity = target_vel;
          smp.cmd = c.position - R.zero_eff[i];
          tx.push_back(j.motor->frame(full ? j.motor->command_position_mode(c) : j.motor->command_position_mode_changed(c)));
          break;
        }
        case ControlMode::Velocity: {
          // host ramp, then the braking envelope towards the limit we are moving to (newest state)
          double v = std::clamp(sp.velocity, vcmd[i] - j.vel_ramp * dt, vcmd[i] + j.vel_ramp * dt);
          if (!lim[i].continuous) {
            v = brake_limited_velocity(v, pos[i], lim[i].min_offset + opt.brake_margin, lim[i].max_offset - opt.brake_margin,
                                       j.vel_ramp);
          }
          vcmd[i] = v;
          smp.cmd = v;
          PositionModeCommand c = regs[i];
          c.position = std::numeric_limits<double>::quiet_NaN();
          c.velocity = v;
          if (!std::isnan(c.stop_position)) c.stop_position += R.zero_eff[i];
          tx.push_back(j.motor->frame(full ? j.motor->command_position_mode(c) : j.motor->command_position_mode_changed(c)));
          break;
        }
        case ControlMode::Torque:
        case ControlMode::Impedance: {
          const bool imp = j.mode == ControlMode::Impedance;
          const double g = imp && opt.gain_ramp_s > 0 ? std::min(t / opt.gain_ramp_s, 1.0) : 1.0;
          const double tau = impedance_torque(pos[i], vel[i], sp.position, imp ? sp.velocity : 0.0, sp.torque,
                                              imp ? g * j.kp : 0.0, g * j.kd, j.max_torque);
          smp.cmd = tau;
          using TF = PipelineOptions::TorqueFrame;
          tx.push_back(j.motor->frame(opt.torque_frame == TF::Min     ? j.motor->command_tqe_min(tau)
                                      : opt.torque_frame == TF::Short ? j.motor->command_tqe_short(tau)
                                                                      : j.motor->command_tqe(tau)));
          break;
        }
      }
    }
    bus.write_frames(tx);
    const auto sent = Clock::now();
    for (size_t i = 0; i < n; i++) pending[i].push_back({k, sent});
    R.cycles++;

    // ---- until the next send time: take replies as they come ----
    next += period;
    if (Clock::now() > next) next = Clock::now();  // we are late (host): do not burst to catch up
    std::fill(fresh.begin(), fresh.end(), false);
    while (Clock::now() < next) {
      rx.clear();
      if (!bus.wait_frames(rx, next)) break;  // deadline reached
      for (const CanFrame& f : rx) {
        const uint8_t src = can_id_source(f.id);
        for (size_t i = 0; i < n; i++) {
          if (joints[i].motor->id() != src) continue;
          if (pending[i].empty()) break;  // not an answer to our requests (should not happen)
          const Pending p = pending[i].front();
          pending[i].pop_front();
          // lost reply detection / realignment (see PipelineOptions)
          size_t open_old = 0;
          for (const Pending& q : pending[i]) {
            if (q.sent < f.rx_time - min_lat) open_old++;
          }
          if (open_old == 0) {
            clean_since[i] = f.rx_time;
          } else if (f.rx_time - clean_since[i] > realign && !pending[i].empty()) {
            pending[i].pop_front();  // its reply was taken by the shifted pairing: count one lost, realign
            R.lost_detected[i]++;
            clean_since[i] = f.rx_time;
          }
          const MotorState s = joints[i].motor->decode(f);
          if (!s.position) break;  // e.g. an error-only reply: the request is answered, no state
          pos[i] = *s.position - R.zero_eff[i];
          vel[i] = s.velocity.value_or(0.0);
          tqe[i] = s.torque.value_or(0.0);  // decode() applied k
          fresh[i] = true;
          last_fresh_cycle[i] = k;
          state_age[i] = k + 1 - p.cycle;  // seen from the next send: 1 = answers the request just sent
          state_latency[i] = std::chrono::duration<double, std::micro>(f.rx_time - p.sent).count();
          R.age_hist[i][std::min(state_age[i], 9)]++;
          if (s.fault && *s.fault != 0) fail(i, Status::Fault);
          if (!within_limits(lim[i], R.zero_eff[i], *s.position, opt.limit_margin)) fail(i, Status::OutOfLimits);
          if (joints[i].safe_vel > 0 && std::fabs(vel[i]) > joints[i].safe_vel) fail(i, Status::TooFast);
          break;
        }
      }
      if (R.failed_index >= 0) break;
    }

    for (size_t i = 0; i < n; i++) {
      PipelineSample& smp = samples[i];
      smp.fresh = fresh[i];
      smp.age = last_fresh_cycle[i] < 0 ? 0 : state_age[i] + (k - last_fresh_cycle[i]);
      smp.latency_us = fresh[i] ? state_latency[i] : 0.0;
      smp.position = pos[i];
      smp.velocity = vel[i];
      smp.torque_meas = tqe[i];
      if (!fresh[i]) R.not_fresh[i]++;
      const int stale_run = last_fresh_cycle[i] < 0 ? k + 1 : k - last_fresh_cycle[i];
      R.max_stale_seen[i] = std::max(R.max_stale_seen[i], stale_run);
      if (stale_run >= opt.max_stale) fail(i, Status::LostReplies);
      if (static_cast<int>(pending[i].size()) > opt.max_outstanding) fail(i, Status::LostReplies);
      if (on_sample) on_sample(smp);
    }
    if (bus.adapter_errors() != err0 && R.failed_index < 0) fail(0, Status::NoReply);
    if (R.failed_index >= 0) break;
  }
  R.elapsed_s = std::chrono::duration<double>(Clock::now() - t0).count();

  // ---- stop all, then count exactly ----
  bus.drain(std::chrono::milliseconds(5));  // replies still in flight are not answers to the stops
  {
    std::vector<Motor*> ms;
    for (const auto& j : joints) ms.push_back(j.motor);
    R.stopped = brake_then_stop(bus, ms, opt.brake_s);
  }
  bus.drain(std::chrono::microseconds(static_cast<int64_t>(opt.drain_s * 1e6)));
  for (size_t i = 0; i < n; i++) {
    R.requests[i] = static_cast<long long>(bus.requests_sent(joints[i].motor->id()) - req0[i]);
    R.replies[i] = static_cast<long long>(bus.replies_received(joints[i].motor->id()) - rep0[i]);
  }
  R.adapter_errors = bus.adapter_errors() - err0;
  return R;
}

}  // namespace hightorque
