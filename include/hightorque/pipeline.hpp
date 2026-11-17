// Pipelined host control loop: position, velocity, torque and impedance.  A separate implementation next to
// run_control() (control.hpp), which is unchanged:
//
//   run_control()            send all -> WAIT for all replies (or timeout) -> next cycle.  A late
//                            reply delays the next send; the rate cannot exceed 1 / round trip.
//   run_position_pipelined() send all at a FIXED interval, whatever the replies do; replies are
//                            collected while waiting for the next send time and used as they arrive.
//
// Replies carry no sequence number, but each motor answers its requests in order and does not drop
// them (README "丢包 / 迟到检测"), so the k-th reply of a motor answers its k-th request.  For every reply
// the loop knows how many cycles old it is ("age": 1 = it answers the request of the previous cycle).
// The command of cycle k is computed from the newest state of each motor, whatever its age.
//
// Safety, like run_control(): checks before any command (zero, limits, reply, start error, arguments),
// and every new state: outside the limits, |velocity| > safe_vel, fault.  In addition:
//   * no new reply from a motor for max_stale cycles -> stop all (LostReplies)
//   * more than max_outstanding requests without reply -> stop all (LostReplies)
//   * adapter "ERR" (e.g. a rejected can send) -> stop all (NoReply)
// All motors are stopped at the end (mode 0 confirmed), then the loop waits drain_s for the last
// replies and counts requests / replies exactly.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "hightorque/control.hpp"

namespace hightorque {

struct PipelineOptions {
  double rate_hz = 1000.0;
  double duration_s = 10.0;               // <= 0: until keep_running() returns false
  double max_start_error = 10.0 / 360.0;  // turns
  double limit_margin = 0.0;              // turns
  int max_stale = 10;                     // cycles without a new reply from one motor -> stop all
  int max_outstanding = 16;               // requests without reply (one motor) -> stop all
  int resend_every = 1;                   // full register frame every N cycles (1 = always, default; see ControlOptions)
  double drain_s = 0.1;
  double brake_s = 0.5;  // at the end (also Ctrl-C / safety stop): brake up to this long, then stop (0 = stop only)
  // Before the fixed schedule: this many send -> wait (up to 20 ms) cycles.  The first frame switches
  // stop -> mode 10 and resets every leftover register; the motor needs ~1.8 ms for it and drops
  // requests that arrive meanwhile (seen in the first pipelined runs: requests #1, #3, #4 lost).
  int warmup_cycles = 3;
  // true ("方案 A", default): the velocity / acceleration limits of ControlJoint::regs (vel_limit /
  // acc_limit) are applied on the host by a SetpointShaper and NOT sent to the motor (shorter frames;
  // the motor's 0x028 / 0x029 only act in the frame that carries them).  false: sent in every frame.
  bool host_shaping = true;
  double gain_ramp_s = 1.0;   // impedance: kp / kd ramp up from 0 (as run_control)
  // torque / impedance frame: Min = command_tqe_min (24 B, default), Short = command_tqe_short (32 B),
  // Full = command_tqe (48 B)
  enum class TorqueFrame { Min, Short, Full } torque_frame = TorqueFrame::Min;
  double brake_margin = 0.01; // velocity: brake to stop this far (turns) inside the limits (as run_control)
  // Lost reply detection: after each reply, count the requests sent more than min_latency_us ago that
  // are still open.  Normally that drops to 0 again within a few ms (even after a 10 ms-task stall);
  // after a lost reply the in-order pairing is shifted by one and it stays >= 1.  If it has not been 0
  // for realign_window_s, the oldest open request is taken as lost and dropped (pairing realigned).
  double min_latency_us = 400.0;
  double realign_window_s = 0.02;
};

struct PipelineSample {
  double t = 0.0;            // s since start (time of this cycle's send)
  int cycle = 0;
  size_t index = 0;          // joint
  bool fresh = false;        // a new reply of this motor arrived since the previous send
  int age = 0;               // of the newest state: cycles between its request and now (0 = none yet)
  double latency_us = 0.0;   // of the newest reply (request written -> reply parsed); 0 if not fresh
  double position = 0.0;     // newest state, turns from zero
  double velocity = 0.0;
  double torque_meas = 0.0;
  ControlSetpoint setpoint;
  double cmd = 0.0;          // sent: position (turns from zero), velocity (turns/s) or torque (Nm)
  double send_late_us = 0.0; // how much later than scheduled this cycle's frames were written
};

struct PipelineResult {
  ControlResult::Status status = ControlResult::Status::NoReply;
  int failed_index = -1;
  double elapsed_s = 0.0;
  int cycles = 0;                    // 0: refused before any command was sent
  int late_sends = 0;                // cycles written more than 25% of a period after their schedule
  double max_send_late_us = 0.0;
  std::vector<double> zero_eff;
  std::vector<bool> stopped;
  // per joint, exact over the run (all cycles + final stop + drain_s): requests, replies
  std::vector<long long> requests, replies;
  // per joint: replies by age in cycles (index 0..9, last = 9 or more), and cycles without a fresh reply
  std::vector<std::vector<int>> age_hist;
  std::vector<int> not_fresh;
  std::vector<int> max_stale_seen;   // longest run of cycles without a new reply
  std::vector<int> lost_detected;    // lost replies found while running (pairing realigned)
  uint64_t adapter_errors = 0;
};

// Any ControlMode (same meaning and safety checks as run_control()), the same trajectory callback:
//   position   PositionModeCommand (regs; vel-max / acc on the host with host_shaping)
//   velocity   mode 10, position NaN, velocity = setpoint (host ramp vel_ramp, braking before the limits)
//   torque     torque frame (opt.torque_frame) with setpoint torque - kd * velocity, clamped to max_torque
//   impedance  torque frame with kp (pos_d - pos) + kd (vel_d - vel) + tau_d, clamped, gains ramped
// The state used for velocity braking, torque damping and impedance is the newest reply (1-2 cycles old).
PipelineResult run_control_pipelined(Fdcanusb& bus, const std::vector<ControlJoint>& joints,
                                     const std::string& zero_path, const ControlTrajectory& trajectory,
                                     const PipelineOptions& opt = {}, const std::function<bool()>& keep_running = {},
                                     const std::function<void(const PipelineSample&)>& on_sample = {});
// Older name (position only at first); the same function.
PipelineResult run_position_pipelined(Fdcanusb& bus, const std::vector<ControlJoint>& joints,
                                      const std::string& zero_path, const ControlTrajectory& trajectory,
                                      const PipelineOptions& opt = {}, const std::function<bool()>& keep_running = {},
                                      const std::function<void(const PipelineSample&)>& on_sample = {});

}  // namespace hightorque
