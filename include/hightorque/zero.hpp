// Host-side zero position ("软件零位").
//
// set_zero_here():  read where the motor is now (e.g. after turning it by hand while it is stopped)
//                   and store that raw position in a small text file on the host.
// go_to_zero():     read the last stored zero and move there with a velocity / acceleration limit
//                   (case 7, motor_set_pos_velmax_acc), then report the final error.
//
// Nothing is written to the motor's flash.  (The vendor rezero, Motor::rezero_and_save(), sends
// "conf write", which saves the motor's whole RAM configuration to flash; we avoid that.)
//
// Power cycle behaviour (measured 2026-11-09/10 on 5047 #1, see README "零位"):
//   the motor keeps its angle within one turn but loses the turn count, so after power-up (or after
//   turning it by whole turns while powered) the raw reading can differ from before by whole turns.
//
// Therefore nothing here compares raw positions directly.  The joint angle is
//     offset = wrap(raw - zero)   folded into [-0.5, +0.5) turns,
// which is the same physical angle whatever whole turns the reading carries.  effective_zero() gives
// the zero in the motor's *current* position frame (zero + whole turns); every target sent to the motor
// is effective_zero + offset.  This only works if the joint can never be more than half a turn away
// from its zero, so the hardcoded joint limits (joint_limits()) must stay strictly inside +-180 deg,
// leaving a "never stop here" band around 180 deg (needed anyway for joints that must not cross 180
// deg, e.g. because of cables: near 180 deg the reading cannot tell which side the joint came from).
//
// A *continuous* joint (nothing is twisted when it turns, e.g. the 4438 flange with an EE without
// cables) has no limits: every angle is valid, whole turns have no physical meaning, and go_to_zero()
// takes the short way (at most half a turn).  Use the Float (or Int32) wire type for it: while
// powered the raw reading keeps counting turns, and Int16 positions saturate at +-3.2767 r.
//
//   * go_to_zero() refuses to move a motor ID without hardcoded limits, or a limited joint outside its
//     limits (e.g. turned by hand while off), and stops if it leaves them while moving.
//   * Not detectable for a limited joint: turned by (almost) a whole turn while unpowered.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "hightorque/motor.hpp"

namespace hightorque {

struct ZeroRecord {
  uint8_t id = 0;
  double position = 0.0;  // raw motor position in turns at the time of set_zero (may carry whole turns)
  std::string time;       // local time, "YYYY-MM-DD HH:MM:SS"
};

// Default file: $HOME/.hightorque/zero_positions.txt (one line per motor ID).
std::string default_zero_file();

// File I/O.  save_zero() replaces the line of that ID and keeps the other IDs.
bool save_zero(const std::string& path, const ZeroRecord& record, std::string* error = nullptr);
std::optional<ZeroRecord> load_zero(const std::string& path, uint8_t id);

// ---- angle relative to the zero ----
// x folded into [-0.5, +0.5) turns.
double wrap_turns(double x);
// Zero expressed in the frame of the current raw reading: zero_raw + n whole turns such that
// pos_raw - effective_zero(...) == wrap_turns(pos_raw - zero_raw).
double effective_zero(double zero_raw, double pos_raw);

// ---- hardcoded software joint limits (per motor ID / joint) ----
struct JointLimits {
  uint8_t id = 0;
  const char* name = "";     // which joint / motor, for messages
  bool continuous = false;   // true: no limits (min/max_offset unused)
  double min_offset = 0.0;   // allowed range relative to the zero, turns (-0.25 = -90 deg)
  double max_offset = 0.0;   // must satisfy -0.5 < min_offset < max_offset < +0.5
};

// Limits for this motor ID, or std::nullopt if none are hardcoded (see zero.cpp).
std::optional<JointLimits> joint_limits(uint8_t id);
// Both take the *effective* zero (same frame as the raw positions).
// pos_raw lies inside zero + [min_offset - margin, max_offset + margin].  Always true if continuous.
bool within_limits(const JointLimits& lim, double zero_eff, double pos_raw, double margin = 0.0);
// target_raw clamped to zero + [min_offset, max_offset] (unchanged if continuous).  Use for every
// position target.
double clamp_to_limits(const JointLimits& lim, double zero_eff, double target_raw);

// Read the current position (motor must answer; it is averaged over `samples` queries) and store it.
// The motor is not moved and not commanded; call stop() first if you want it to be free to turn.
std::optional<ZeroRecord> set_zero_here(Motor& motor, const std::string& path, int samples = 10,
                                        std::string* error = nullptr);

struct GoToZeroOptions {
  double vel_max = 0.2;        // turns/s
  double acc = 0.5;            // turns/s^2
  double tolerance = 0.002;    // turns (0.72 deg)
  double settle_s = 0.3;       // must stay within tolerance (and slow) this long
  double timeout_s = 15.0;     // give up after this
  double rate_hz = 200.0;      // command rate while moving
  bool stop_at_end = true;     // false: keep holding the zero position when returning
  // true: require hardcoded limits, check them before moving and while moving (recommended).
  // false (--force): no limit checks (the move is still at most half a turn, the short way).
  bool check_limits = true;
  double limit_margin = 0.01;  // turns (3.6 deg) of slack on the limits for measurement / overshoot
  // Host-side correction of a steady error (measured on 5047 #1: static friction stops the motor
  // about 0.007 r from the target).  While almost still, within correction_capture of the zero and
  // error above half the tolerance, the commanded target is moved by -correction_rate * error per
  // second, bounded by +-correction_max (and by the limits).  0 disables.
  double correction_rate = 0.5;      // 1/s
  double correction_max = 0.03;      // turns (10.8 deg)
  double correction_capture = 0.05;  // turns (18 deg)
};

struct GoToZeroResult {
  enum class Status { Reached, NoZeroStored, NoReply, NoLimits, OutOfLimits, Int16Continuous, Timeout,
                      Interrupted };
  Status status = Status::NoReply;
  double target = 0.0;         // effective zero: raw position of the zero in the motor's current frame
  double start = 0.0;          // raw position before moving
  double start_offset = 0.0;   // start - target (turns, within [-0.5, 0.5))
  double final_error = 0.0;    // final position - target (turns)
  double elapsed_s = 0.0;
  double correction = 0.0;     // host-side correction at the end (command - target, turns)
  double command = 0.0;        // last commanded raw position (hold here to stay where it ended)
  // Lost replies.  On a healthy, terminated bus both must be 0 (see README "零位" / CAN 丢包).
  int start_query_retries = 0; // extra queries needed before moving
  int missed_replies = 0;      // command cycles without a reply while moving
  int first_missed_cycle = -1; // index of the first such cycle (0 = the first command), -1: none
  int cycles = 0;              // command cycles while moving
};

const char* to_string(GoToZeroResult::Status s);

// Blocking.  `keep_running` is polled every cycle (e.g. a Ctrl-C flag); return false to abort
// (the motor is then stopped).  `progress` is called about every 0.5 s with the latest state, its
// error to the zero and the commanded offset (command - zero, i.e. the correction), all in turns.
// NoZeroStored / NoLimits / Int16Continuous / NoReply / OutOfLimits found at the start are returned
// before any command is sent.  Once moving, the motor is stopped at the end unless the status is Reached and
// stop_at_end is false.
GoToZeroResult go_to_zero(Motor& motor, const std::string& path, const GoToZeroOptions& opt = {},
                          const std::function<bool()>& keep_running = {},
                          const std::function<void(const MotorState&, double error, double command_offset)>&
                              progress = {});

// ---- several motors at once, synchronized (like a robot arm going home) ----
//
// One host-side trajectory for all motors: every cycle each motor gets the target
//     start_i + s(t) * (zero_i - start_i),   s = minimum-jerk profile from 0 to 1 over T,
// so all motors start together, are at the same fraction of their way at every moment, and arrive
// together.  T is the shortest duration that keeps every motor within opt.vel_max / opt.acc.  The
// commands of all motors go out in one cycle() (one USB write; on the CAN bus the frames follow each
// other within about 0.1 ms).  After T each motor holds its zero with the host-side friction correction;
// the result is Reached when every motor is within tolerance (settled).  Any problem on any motor
// (limits, lost replies, timeout = T + opt.timeout_s, interrupt) stops ALL motors.

// Minimum-jerk profile: s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5 for tau in [0, 1] (clamped outside).
// Velocity and acceleration are zero at both ends.
double min_jerk(double tau);
// Shortest T for a minimum-jerk move of `distance` turns with peak velocity <= vel_max and peak
// acceleration <= acc (peak velocity 1.875 d / T, peak acceleration 5.7735 d / T^2).  0 for d = 0.
double min_jerk_duration(double distance, double vel_max, double acc);

struct GoToZeroManyResult {
  GoToZeroResult::Status status = GoToZeroResult::Status::NoReply;  // Reached only if all reached
  int failed_index = -1;                  // index (into motors) of the motor that caused status, -1: none
  std::vector<GoToZeroResult> motors;     // per motor: target, start, final_error, correction, counters
  double planned_s = 0.0;                 // duration T of the synchronized move
  double elapsed_s = 0.0;
};

// Blocking.  Motor IDs must be different.  `progress(index, state, error, command_offset)` is called
// about every 0.5 s for each motor with its latest state.  Statuses found before moving (no zero, no
// limits, no reply, outside limits) are returned before any command is sent.
GoToZeroManyResult go_to_zero_many(
    Fdcanusb& bus, const std::vector<Motor*>& motors, const std::string& path, const GoToZeroOptions& opt = {},
    const std::function<bool()>& keep_running = {},
    const std::function<void(size_t index, const MotorState&, double error, double command_offset)>& progress = {});

}  // namespace hightorque
