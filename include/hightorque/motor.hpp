// High level motor API.  C++ port of the vendor STM32 example:
//   Motor      <-> motor_control.c / motor_config.c  (one motor per CAN frame, "普通模式")
//   MotorMany  <-> motor_many.c                      (one frame drives many motors, "一拖多模式")
//
// Every set_xxx() call sends ONE CAN-FD frame that (1) writes the command and (2) asks the motor
// for its state, then waits for the reply.  So one call == one control cycle, exactly like the
// vendor functions, but the reply is returned directly instead of being parsed in an interrupt.
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <limits>

#include "hightorque/fdcanusb.hpp"
#include "hightorque/protocol.hpp"

namespace hightorque {

// 位置模式 (mode 10) with every register exposed.  All vendor position-type cases are combinations of
// these registers (case 2: position only; 3: position NaN + velocity; 4: position NaN, velocity 0,
// torque, kp / kd scale 0; 5 / 6: position NaN, velocity, max torque, stop position; 7: position,
// velocity limit, acceleration limit).  A field left at its default ("not used") is not sent, and a
// register set by an earlier command is reset to it (sticky registers, README).  Units: turns (or the
// motor's AngleUnit), Nm (real output torque, k applied).
struct PositionModeCommand {
  double position = std::numeric_limits<double>::quiet_NaN();       // 0x020 target; NaN = none (run at velocity)
  double velocity = 0.0;                                           // 0x021 velocity / feed-forward velocity
  double torque = 0.0;                                             // 0x022 feed-forward torque
  double kp_scale = 1.0;                                           // 0x023 scale of the firmware kp (0..1)
  double kd_scale = 1.0;                                           // 0x024 scale of the firmware kd (0..1)
  double max_torque = std::numeric_limits<double>::quiet_NaN();     // 0x025 NaN = no limit
  double stop_position = std::numeric_limits<double>::quiet_NaN();  // 0x026 NaN = none
  double vel_limit = std::numeric_limits<double>::quiet_NaN();      // 0x028 NaN = firmware default
  double acc_limit = std::numeric_limits<double>::quiet_NaN();      // 0x029 NaN = firmware default
};

class Motor {
 public:
  struct Options {
    MotorModel model = MotorModel::M5047_36;  // torque correction factor k (HTDW-5047-36-NE)
    DataType type = DataType::Float;          // wire type: Float / Int32 / Int16 (precision vs. size)
    AngleUnit unit = AngleUnit::Turns;        // unit for pos / vel / acc on this API
    bool query_full = false;                  // also read bus voltage + temperature in every reply
    std::chrono::microseconds timeout = std::chrono::milliseconds(20);  // wait for reply
    // Registers of position mode (mode 10) keep their value until written again, so a limit set
    // by one function (e.g. set_pos_vel_max_tqe's stop position) would still act on the next
    // one.  When true (default), every mode-10 command also resets the optional registers it does
    // not use to their neutral value (see README "register 残留").  false = vendor-identical frames.
    bool clear_sticky_registers = true;
  };

  Motor(Fdcanusb& bus, uint8_t id);
  Motor(Fdcanusb& bus, uint8_t id, Options options);

  uint8_t id() const { return id_; }
  const Options& options() const { return opt_; }
  Options& options() { return opt_; }

  // ----- control commands (vendor motor_set_xxx), each returns the motor state -----
  // All of them return std::nullopt if the motor did not answer within options().timeout.

  // DQ 电压模式: Q-axis voltage in V.                                   (vendor case 0)
  std::optional<MotorState> set_dq_volt(double volt);
  // DQ 电流模式: Q-axis current in A.                                   (vendor case 1)
  std::optional<MotorState> set_dq_current(double current);
  // 位置模式: go to `pos` with max velocity and acceleration. Violent!   (vendor case 2)
  std::optional<MotorState> set_pos(double pos);
  // 速度模式: reach `vel` with max acceleration.                         (vendor case 3)
  std::optional<MotorState> set_vel(double vel);
  // 力矩模式: output torque in Nm (k-corrected).                        (vendor case 4)
  std::optional<MotorState> set_tqe(double torque);
  // 位置速度模式: move to `pos` at speed `vel`.                          (vendor case 5)
  std::optional<MotorState> set_pos_vel(double pos, double vel);
  // 位置速度最大力矩模式: like set_pos_vel with a torque limit (NaN = none). (vendor case 6)
  std::optional<MotorState> set_pos_vel_max_tqe(double pos, double vel, double max_torque);
  // 梯形控制: move to `pos`, velocity limit `vel_max`, acceleration `acc`.  (vendor case 7)
  std::optional<MotorState> set_pos_velmax_acc(double pos, double vel_max, double acc);
  // 速度加速度模式: reach `vel` with acceleration `acc`.
  std::optional<MotorState> set_vel_acc(double vel, double acc);
  // 速度 + 速度限幅 (vendor set_vel_velmax_int16).
  std::optional<MotorState> set_vel_velmax(double vel, double vel_max);
  // 运控模式2 (真运控, mode 21):
  //   torque = (pos - pos_now) * kp + (vel - vel_now) * kd + tqe         (vendor case 8)
  // Old firmware rejects mode 21 but still stores kp / kd in 0x02b / 0x02c, which makes every
  // position-mode command soft afterwards.  So the first call reads the original 0x02b / 0x02c,
  // and if the motor rejects the mode they are written back right away (README "register 残留").
  std::optional<MotorState> set_pos_vel_tqe_kp_kd(double pos, double vel, double torque, double kp, double kd);
  // vfoc 固定模式: hold the rotor with a fixed D voltage (quiet stop, more current).
  std::optional<MotorState> set_vfoc_lock(double volt);
  // 刹车模式: all phases shorted, damping proportional to speed.         (vendor case 9)
  std::optional<MotorState> brake();
  // 停止模式: all phases floating, also clears faults.                   (vendor case 10)
  std::optional<MotorState> stop();
  // 查询电机状态 only, no command.                                       (vendor case 11)
  std::optional<MotorState> query();
  // 查询电机固件版本.                                                    (vendor case 12)
  std::optional<FirmwareVersion> get_version();
  // Read `count` registers as float (no unit conversion), e.g. read_float_registers(0x2b, 2).
  // std::nullopt if there is no reply or the motor reports a read error.
  std::optional<std::vector<double>> read_float_registers(uint16_t start_reg, uint32_t count);

  // ----- configuration -----

  // Soft reboot; the motor comes back in stop mode and sends no reply.
  void reset();
  // Watchdog: if no command arrives for `ms`, the motor brakes (vendor set_out_time_int16).
  bool set_timeout_ms(int16_t ms);
  // Make the motor send its state every `ms` ms by itself (0 = off). Frames arrive via bus.receive().
  void set_timed_return_ms(int16_t ms);
  // Set the current position as zero and save it to flash (vendor motor_pos_reset).
  // 0 = ok, 1 = rezero failed, 2 = save failed.  The motor must be stopped first.
  int rezero_and_save();
  // Save the configuration to flash (vendor motor_conf_write).  0 = ok, 1 = failed.
  int conf_write();

  // ----- lower level -----

  // Build the frame of a command without sending it (for cycle() with several motors).
  // Build a command right before sending it: the mode-10 builders update the record of which
  // optional registers are currently set (Options::clear_sticky_registers).
  CanFrame frame(const FrameBuilder& fb) const;
  FrameBuilder command_dq_volt(double volt) const;
  FrameBuilder command_dq_current(double current) const;
  FrameBuilder command_pos(double pos);
  FrameBuilder command_vel(double vel);
  FrameBuilder command_tqe(double torque);
  // Same pure torque command without the max-torque reset (0x025 = NaN): position NaN, velocity 0,
  // torque, kp scale 0, kd scale 0 in every frame.  32 instead of 48 byte (float).  Only for loops that
  // clamp the torque themselves (pipelined torque / impedance); a 0x025 left over from an earlier
  // command is cleared once by the sticky-register logic.
  FrameBuilder command_tqe_short(double torque);
  // Shortest pure torque command: only torque, kp scale 0, kd scale 0 (0x022..0x024) every frame, no
  // position / velocity target (with both scales 0 they do not act: torque = scales x (...) + torque).
  // 24 byte (float).  Leftover 0x021 / 0x025 values are cleared once by the sticky-register logic.
  FrameBuilder command_tqe_min(double torque);
  FrameBuilder command_pos_vel_max_tqe(double pos, double vel, double max_torque);
  FrameBuilder command_pos_velmax_acc(double pos, double vel_max, double acc);
  FrameBuilder command_vel_acc(double vel, double acc);
  FrameBuilder command_vel_velmax(double vel, double vel_max);
  FrameBuilder command_pos_vel_tqe_kp_kd(double pos, double vel, double torque, double kp, double kd);
  FrameBuilder command_position_mode(const PositionModeCommand& c);  // mode 10, any register combination
  // Same, but registers 0x021..0x029 whose value equals what the previous command_position_mode() of
  // this Motor wrote are NOT sent again (the motor keeps them).  The position is always sent.  Only
  // valid while nothing else wrote these registers: any other command, reset() or a motor reboot
  // forgets the cache, so send a full command_position_mode() now and then (run_control: --resend-every).
  FrameBuilder command_position_mode_changed(const PositionModeCommand& c);
  FrameBuilder command_vfoc_lock(double volt) const;
  FrameBuilder command_mode(uint8_t mode) const;  // mode + query (stop / brake)
  FrameBuilder command_query() const;

  // Send one frame and wait for this motor's reply.
  std::optional<MotorState> transact(const CanFrame& frame);
  // Convert a raw reply into user units (pos/vel unit, torque * k).
  MotorState decode(const CanFrame& reply) const;

  // Last successfully received state.
  const MotorState& last_state() const { return last_; }

 private:
  double tqe_adjust(double torque) const;  // real torque -> register value (divide by k)
  double pid_adjust(double gain) const;    // kp / kd -> register value (divide by k)
  void add_query(FrameBuilder& fb) const;
  // Reset the optional mode-10 registers that are set on the motor but not written by this
  // command (`written` = bit mask, see motor.cpp), then remember which ones this command sets.
  void clear_sticky(FrameBuilder& fb, uint32_t written);
  bool diagnostic_with_ack(const std::string& text);

  Fdcanusb& bus_;
  uint8_t id_;
  Options opt_;
  MotorState last_;
  // Optional mode-10 registers that may hold a non-neutral value.  All of them at start, because
  // an earlier program may have left anything there.
  uint32_t sticky_set_ = ~0u;
  // Values (as sent, after unit / k conversion) the last command_position_mode() wrote to 0x021..0x029;
  // invalid after any other command (see clear_sticky()).
  bool pm_cache_valid_ = false;
  double pm_cache_[9] = {};
  FrameBuilder position_mode_frame(const PositionModeCommand& c, bool skip_unchanged);
  // 0x02b / 0x02c as they were before the first set_pos_vel_tqe_kp_kd() (see there).
  std::optional<std::vector<double>> saved_kp_kd_;
};

// Send one command to each of several motors in one go (frames are written back to back, then
// all replies are collected).  Much faster than calling set_xxx() motor by motor.
std::vector<std::optional<MotorState>> cycle(Fdcanusb& bus, const std::vector<std::pair<Motor*, FrameBuilder>>& commands);
// Same, but wait at most `timeout` for the replies (instead of the motors' Options::timeout).
std::vector<std::optional<MotorState>> cycle(Fdcanusb& bus, const std::vector<std::pair<Motor*, FrameBuilder>>& commands,
                                             std::chrono::microseconds timeout);

// Pretty print, like motor_print_state().
std::string format_state(const MotorState& s, AngleUnit unit = AngleUnit::Turns);

// ---------------------------------------------------------------------------
// 一拖多模式: one CAN frame carries the commands of all motors (IDs 1..N), int16 only.
// Usage (same as the vendor):
//     many.vel(1, 0.1);  many.vel(2, 0.1);   // write into the buffer
//     auto states = many.send();             // send + collect replies; states[id - 1]
// All motors on the bus are in the same mode.  At 1 Mbps / 5 Mbps up to ~10 motors at 1 kHz.
// ---------------------------------------------------------------------------
class MotorMany {
 public:
  enum class Request {
    ModeFaultPosVelTqe,  // 0xFF 0xFF, motor firmware >= 4.2.3
    TempFaultPosVelTqe,  // 0xFF 0xFE, motor firmware >= 4.5.0
    PosVelTqe,           // 0x17 0x01, all firmware versions
  };

  // `num_motors` motors with IDs 1..num_motors.
  MotorMany(Fdcanusb& bus, int num_motors, MotorModel model = MotorModel::M5047_36,
            AngleUnit unit = AngleUnit::Turns);

  void set_model(uint8_t id, MotorModel model);
  void set_timeout(std::chrono::microseconds t) { timeout_ = t; }

  // Buffer writers (motor_many_xxx).  Switching to another mode resets every slot to its default.
  void dq_volt(uint8_t id, double volt);
  void dq_current(uint8_t id, double current);
  void pos(uint8_t id, double pos);
  void vel(uint8_t id, double vel);
  void tqe(uint8_t id, double torque);
  void time_out(uint8_t id, int16_t ms);
  void pos_vel(uint8_t id, double pos, double vel);
  void pos_vel_max_tqe(uint8_t id, double pos, double vel, double max_torque);
  void pos_vel_acc(uint8_t id, double pos, double vel, double acc);
  void pos_vel_tqe_kp_kd(uint8_t id, double pos, double vel, double torque, double kp, double kd);
  void pos_vel_tqe_kp_kd_2(uint8_t id, double pos, double vel, double torque, double kp, double kd);

  // Send the buffer (motor_many_send) and collect the replies, index = id - 1.
  std::vector<std::optional<MotorState>> send(Request request = Request::TempFaultPosVelTqe);

  // Frames that send() would transmit (for testing / debugging).
  std::vector<CanFrame> frames(Request request) const;

 private:
  void select_mode(uint8_t mode);
  void put(uint8_t id, std::initializer_list<int16_t> values);
  int16_t i16(double value, Quantity q) const;
  size_t slot_bytes() const;
  double k(uint8_t id) const;

  Fdcanusb& bus_;
  int num_motors_;
  std::vector<MotorModel> models_;
  AngleUnit unit_;
  uint8_t mode_ = 0;
  std::vector<int16_t> slots_;
  std::chrono::microseconds timeout_ = std::chrono::milliseconds(20);
};

}  // namespace hightorque
