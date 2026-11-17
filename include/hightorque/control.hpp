// One host-side control loop for several motors on one bus: position, velocity, torque or
// impedance, with the same safety checks for every mode.
//
// Every cycle (default 300 Hz) a trajectory callback gives each joint its setpoint; the loop turns it
// into one command per motor and sends all of them in one cycle() (one USB write, then wait for all
// replies).  Angles are relative to the stored zero (wrap-based, see zero.hpp), in turns.
//
//   Position   位置模式 (mode 10) with the registers of ControlJoint::regs (motor.hpp
//              PositionModeCommand: velocity, feed-forward torque, kp / kd scale, max torque, velocity /
//              acceleration limit; unset = not sent).  Position = the setpoint, clamped to the joint limits.
//              (regs with only vel_limit + acc_limit = case 7, nothing = case 2.)
//   Velocity   mode 10 with position NaN and velocity = the setpoint, changed at most vel_ramp per second
//              by the host and reduced near the joint limits so the joint can always brake in time
//              (|v| <= sqrt(2 vel_ramp d), d = distance to the limit); the other regs are sent as set.
//   Torque     case 4 (motor_set_tqe): tau = setpoint torque - kd * velocity (kd = optional damping),
//              clamped to +-max_torque.
//   Impedance  case 4: tau = kp (pos_d - pos) + kd (vel_d - vel) + tau_d, clamped to +-max_torque.
//              (Host-side, because the lab motors' old firmware rejects mode 21; README "Impedance".)
//
// Safety (any joint -> stop ALL motors, confirm mode 0):
//   before any command: zero stored, hardcoded limits, reply, inside the limits, setpoint at t = 0 within
//   max_start_error of the joint (position / impedance), gains / limits valid, IDs different;
//   every cycle: outside the limits (+ limit_margin), |velocity| > safe_vel, fault, max_missed_in_row
//   cycles without reply.  The motors are stopped at the end whatever the status.
// Not covered: if the process is killed (not Ctrl-C / SIGHUP / SIGPIPE), velocity and torque modes keep
// the last command.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "hightorque/motor.hpp"
#include "hightorque/zero.hpp"

namespace hightorque {

enum class ControlMode { Position, Velocity, Torque, Impedance };
const char* to_string(ControlMode m);

struct ControlJoint {
  Motor* motor = nullptr;
  ControlMode mode = ControlMode::Impedance;
  double kp = 0.0;          // impedance: Nm / turn
  double kd = 0.0;          // impedance / torque: Nm / (turn/s)
  double max_torque = 0.3;  // torque / impedance: |torque command| never exceeds this (Nm)
  PositionModeCommand regs;  // position / velocity: register values sent every cycle (position / velocity
                             // fields are replaced by the setpoint, see feed_forward_velocity)
  bool feed_forward_velocity = false;  // position: send the setpoint velocity in 0x021 (else regs.velocity)
  double vel_ramp = 1.0;    // velocity: host acceleration limit and braking deceleration (turns/s^2)
  double safe_vel = 0.5;    // all modes: |measured velocity| above this -> stop all (turns/s); 0 = check OFF
};

// Setpoint of one joint (relative to the zero).  Which fields are used depends on the mode.
struct ControlSetpoint {
  double position = 0.0;  // turns (position, impedance)
  double velocity = 0.0;  // turns/s (velocity, impedance)
  double torque = 0.0;    // Nm (torque, impedance feed-forward)
};
// (joint index, time since start in s) -> setpoint.  Called every cycle for every joint.
using ControlTrajectory = std::function<ControlSetpoint(size_t joint, double t)>;

struct ControlOptions {
  double rate_hz = 300.0;
  double duration_s = 10.0;               // <= 0: until keep_running() returns false
  double max_start_error = 10.0 / 360.0;  // turns (position / impedance)
  double limit_margin = 0.0;              // turns of slack on the joint limits before stopping
  int max_missed_in_row = 3;              // consecutive cycles without reply (per motor) -> stop all
  double gain_ramp_s = 1.0;               // impedance: kp and kd ramp from 0 to full (soft start)
  double brake_margin = 0.01;             // velocity: brake to stop this far (turns, 3.6 deg) inside the limits
  // The first cycles after stop -> mode 10 are slow (the first frame also resets every leftover
  // register, 64 bytes; round trip measured about 1.7-1.8 ms).  They wait up to startup_timeout.
  int startup_cycles = 3;
  double startup_timeout_s = 0.02;
  double drain_s = 0.1;                   // at the end: wait this long for late replies before counting
  double brake_s = 0.5;                   // at the end: brake up to this long before stopping (0 = stop only)
  // position / velocity: registers that did not change since the previous cycle are not sent again
  // (shorter frames); every resend_every-th cycle (and the first) sends all of them.  1 = send every
  // register every cycle (DEFAULT).  WARNING (2026-11-12): the velocity / acceleration limits 0x028 /
  // 0x029 seem to act only in the frame that carries them: with resend_every 100 a 30 deg position step
  // in a frame without them ran at full speed (> 0.5 r/s with vel-max 0.3, stopped by safe_vel).  Do not
  // use > 1 with vel-max / acc until verified (README "不变的寄存器只发一次").
  int resend_every = 1;
};

// The impedance / torque law alone (for tests).  Returns the clamped torque.
double impedance_torque(double pos, double vel, double pos_d, double vel_d, double tau_d, double kp, double kd,
                        double max_torque);
// Host-side setpoint shaper ("方案 A"): turns any target stream (also a step) into commands that move
// at most vel_max and accelerate at most acc, arriving at the target with zero relative velocity.
// Every update(): command velocity v moves towards  v_target + sign(e) * v_stop(|e|)  (clamped to
// +-vel_max; v_stop = the speed that can still stop within |e|, ~sqrt(2 acc |e|)), changing at most
// acc * dt; e = target - command.  v_target is the target's own velocity
// (feed-forward, so a target that moves within the limits is followed without lag).  Units: turns, s.
struct SetpointShaper {
  double vel_max = 0.3;  // turns/s
  double acc = 1.0;      // turns/s^2
  double pos = 0.0;      // current command
  double vel = 0.0;      // current command velocity
  void reset(double p) {
    pos = p;
    vel = 0.0;
  }
  double update(double target, double target_vel, double dt);
};

// Largest |velocity| that still lets the joint stop inside [lo, hi] with deceleration acc, for a joint at
// pos moving in the direction of `sign(v)`.  Returns v reduced accordingly (0 at or beyond the limit).
double brake_limited_velocity(double v, double pos, double lo, double hi, double acc);

struct ControlSample {
  double t = 0.0;
  size_t index = 0;
  bool stale = false;           // no reply this cycle (position / velocity are from the last reply)
  double position = 0.0;        // turns from zero
  double velocity = 0.0;        // turns/s
  double torque_meas = 0.0;     // Nm (k applied)
  ControlSetpoint setpoint;     // what the trajectory asked for
  double cmd = 0.0;             // what was sent: position (turns), velocity (turns/s) or torque (Nm)
  double rtt_us = 0.0;          // this cycle: time from sending all commands until all replies (or timeout)
  int late = 0;                 // replies of this motor to EARLIER cycles found (and dropped) in this cycle
};

struct ControlResult {
  enum class Status { Done, Interrupted, NoZeroStored, NoLimits, NoReply, OutOfLimits, StartTooFar, TooFast,
                      LostReplies, Fault, BadArgument };
  Status status = Status::NoReply;
  int failed_index = -1;          // joint that caused the status, -1: none
  double elapsed_s = 0.0;
  int cycles = 0;                 // 0: refused before any command was sent
  int overruns = 0;               // cycles longer than one period (the loop then resyncs)
  std::vector<int> missed;        // per joint: cycles without reply.  Must be 0 on a healthy bus.
  std::vector<int> late;          // per joint: of those, replies that did arrive, but too late (dropped)
  // Exact, per joint, over the whole run (all cycles + the final stop), counted by the driver after
  // waiting drain_s for stragglers: frames that asked for a reply, and replies that arrived.
  // requests - replies = replies that NEVER arrived (really lost); 0 means every miss was only late.
  std::vector<long long> requests, replies;
  std::vector<double> zero_eff;   // per joint: zero in the motor's raw frame
  std::vector<bool> stopped;      // per joint: stop confirmed (mode 0) at the end
};
const char* to_string(ControlResult::Status s);

// End of a run (also Ctrl-C and every safety stop): first BRAKE (mode 15, phases shorted: a torque
// against the motion, proportional to speed) until every motor is slower than 0.02 r/s or brake_s
// passed, then STOP (mode 0, free, confirmed).  A plain stop lets a fast, low-friction motor (4438)
// coast on.  brake_s = 0: stop at once.  Returns, per motor, whether mode 0 was confirmed.
std::vector<bool> brake_then_stop(Fdcanusb& bus, const std::vector<Motor*>& motors, double brake_s = 0.5);

// Blocking.  `on_sample` is called for every joint in every cycle (keep it fast).
// Set Motor::Options::timeout to about one period (e.g. 3 ms at 300 Hz).
ControlResult run_control(Fdcanusb& bus, const std::vector<ControlJoint>& joints, const std::string& zero_path,
                          const ControlTrajectory& trajectory, const ControlOptions& opt = {},
                          const std::function<bool()>& keep_running = {},
                          const std::function<void(const ControlSample&)>& on_sample = {});

}  // namespace hightorque
