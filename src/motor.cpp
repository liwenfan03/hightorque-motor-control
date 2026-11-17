#include "hightorque/motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>

namespace hightorque {

using std::chrono::milliseconds;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

// ---------------------------------------------------------------------------
// Motor
// ---------------------------------------------------------------------------

Motor::Motor(Fdcanusb& bus, uint8_t id) : Motor(bus, id, Options{}) {}

Motor::Motor(Fdcanusb& bus, uint8_t id, Options options) : bus_(bus), id_(id), opt_(options) {}

double Motor::tqe_adjust(double torque) const {
  if (std::isnan(torque)) return torque;
  return torque / torque_factor(opt_.model);
}

double Motor::pid_adjust(double gain) const {
  if (std::isnan(gain)) return gain;
  return gain / torque_factor(opt_.model);
}

void Motor::add_query(FrameBuilder& fb) const {
  if (opt_.query_full) {
    fb.query_state_full(opt_.type);
  } else {
    fb.query_state(opt_.type);
  }
}

// Optional position-mode registers and the value that means "not used".  They keep whatever was
// written last, even across stop and across programs (only a motor reset clears them), so a
// command that does not use one must reset it.  Table order = register order, so neighbours can
// be cleared in one sub-frame.
namespace {
struct StickyRegister {
  uint16_t reg;
  Quantity quantity;
  double neutral;
};
const StickyRegister kSticky[] = {
    {reg::kVelocityCmd, Quantity::Velocity, 0.0},        // 0x021 target / feedforward velocity
    {reg::kFeedforwardTorque, Quantity::Torque, 0.0},    // 0x022 feedforward torque
    {reg::kKpScale, Quantity::Scale, 1.0},               // 0x023 kp scale (set_tqe writes 0)
    {reg::kKdScale, Quantity::Scale, 1.0},               // 0x024 kd scale (set_tqe writes 0)
    {reg::kMaxTorque, Quantity::Torque, kNaN},           // 0x025 max torque, NaN = no limit
    {reg::kStopPosition, Quantity::Position, kNaN},      // 0x026 stop position, NaN = none
    {reg::kVelocityLimit, Quantity::Velocity, kNaN},     // 0x028 velocity limit, NaN = default
    {reg::kAccelLimit, Quantity::Accel, kNaN},           // 0x029 acceleration limit, NaN = default
};
constexpr uint32_t bit(uint16_t r) { return 1u << (r - reg::kPositionCmd); }
}  // namespace

void Motor::clear_sticky(FrameBuilder& fb, uint32_t written) {
  pm_cache_valid_ = false;  // another command writes these registers (position_mode_frame() sets it again)
  if (!opt_.clear_sticky_registers) return;
  const uint32_t to_clear = sticky_set_ & ~written;
  const size_t n = sizeof(kSticky) / sizeof(kSticky[0]);
  for (size_t i = 0; i < n;) {
    if (!(to_clear & bit(kSticky[i].reg))) {
      i++;
      continue;
    }
    const uint16_t start = kSticky[i].reg;
    std::vector<double> values;
    std::vector<Quantity> quantities;
    while (i < n && kSticky[i].reg == start + values.size() && (to_clear & bit(kSticky[i].reg))) {
      values.push_back(kSticky[i].neutral);
      quantities.push_back(kSticky[i].quantity);
      i++;
    }
    fb.write(opt_.type, start, values, quantities);
  }
  sticky_set_ = written;
}

CanFrame Motor::frame(const FrameBuilder& fb) const {
  CanFrame f;
  f.id = kReplyFlag | id_;  // source 0, destination id, reply requested
  f.data = fb.build();
  return f;
}

// The layouts below reproduce livelybot_fdcan.c register by register.

FrameBuilder Motor::command_dq_volt(double volt) const {
  FrameBuilder fb;
  fb.set_mode(mode::kVoltageDq);
  fb.write(opt_.type, reg::kDVoltage, Quantity::Voltage, {0.0, volt});  // D = 0, Q = volt
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_dq_current(double current) const {
  FrameBuilder fb;
  fb.set_mode(mode::kCurrentDq);
  fb.write(opt_.type, reg::kQCurrentCmd, Quantity::Current, {current, 0.0});  // Q = current, D = 0
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_pos(double pos) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, Quantity::Position, {to_turns(pos, opt_.unit)});
  clear_sticky(fb, 0);
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_vel(double vel) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, to_turns(vel, opt_.unit)},
           {Quantity::Position, Quantity::Velocity});
  clear_sticky(fb, bit(reg::kVelocityCmd));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_tqe(double torque) {
  // Position mode with position = NaN, velocity = 0, kp scale = kd scale = 0  => pure feedforward torque.
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, 0.0, tqe_adjust(torque), 0.0, 0.0, kNaN},
           {Quantity::Position, Quantity::Velocity, Quantity::Torque, Quantity::Scale, Quantity::Scale,
            Quantity::Torque});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kFeedforwardTorque) | bit(reg::kKpScale) | bit(reg::kKdScale) |
                       bit(reg::kMaxTorque));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_tqe_short(double torque) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, 0.0, tqe_adjust(torque), 0.0, 0.0},
           {Quantity::Position, Quantity::Velocity, Quantity::Torque, Quantity::Scale, Quantity::Scale});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kFeedforwardTorque) | bit(reg::kKpScale) | bit(reg::kKdScale));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_tqe_min(double torque) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kFeedforwardTorque, {tqe_adjust(torque), 0.0, 0.0},
           {Quantity::Torque, Quantity::Scale, Quantity::Scale});
  clear_sticky(fb, bit(reg::kFeedforwardTorque) | bit(reg::kKpScale) | bit(reg::kKdScale));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_pos_vel_max_tqe(double pos, double vel, double max_torque) {
  // position = NaN + velocity + stop position = target: run at `vel` and stop at `pos`.
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, to_turns(vel, opt_.unit)}, {Quantity::Position, Quantity::Velocity});
  fb.write(opt_.type, reg::kMaxTorque, {tqe_adjust(max_torque), to_turns(pos, opt_.unit)},
           {Quantity::Torque, Quantity::Position});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kMaxTorque) | bit(reg::kStopPosition));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_pos_velmax_acc(double pos, double vel_max, double acc) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, Quantity::Position, {to_turns(pos, opt_.unit)});
  fb.write(opt_.type, reg::kVelocityLimit, {to_turns(vel_max, opt_.unit), to_turns(acc, opt_.unit)},
           {Quantity::Velocity, Quantity::Accel});
  clear_sticky(fb, bit(reg::kVelocityLimit) | bit(reg::kAccelLimit));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_vel_acc(double vel, double acc) {
  // Note: the vendor float version writes 0x80000000 (= -0.0f) as position; that is the int32
  // "no limit" value and looks like a copy/paste slip.  We send a real NaN for every type.
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, to_turns(vel, opt_.unit)}, {Quantity::Position, Quantity::Velocity});
  fb.write(opt_.type, reg::kAccelLimit, Quantity::Accel, {to_turns(acc, opt_.unit)});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kAccelLimit));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_vel_velmax(double vel, double vel_max) {
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  fb.write(opt_.type, reg::kPositionCmd, {kNaN, to_turns(vel, opt_.unit)}, {Quantity::Position, Quantity::Velocity});
  fb.write(opt_.type, reg::kVelocityLimit, Quantity::Velocity, {to_turns(vel_max, opt_.unit)});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kVelocityLimit));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_pos_vel_tqe_kp_kd(double pos, double vel, double torque, double kp, double kd) {
  FrameBuilder fb;
  fb.set_mode(mode::kPdControl2);
  fb.write(opt_.type, reg::kPositionCmd, {to_turns(pos, opt_.unit), to_turns(vel, opt_.unit), tqe_adjust(torque)},
           {Quantity::Position, Quantity::Velocity, Quantity::Torque});
  fb.write(opt_.type, reg::kKp, Quantity::Pid, {pid_adjust(kp), pid_adjust(kd)});
  clear_sticky(fb, bit(reg::kVelocityCmd) | bit(reg::kFeedforwardTorque));
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_position_mode(const PositionModeCommand& c) { return position_mode_frame(c, false); }

FrameBuilder Motor::command_position_mode_changed(const PositionModeCommand& c) { return position_mode_frame(c, true); }

FrameBuilder Motor::position_mode_frame(const PositionModeCommand& c, bool skip_unchanged) {
  auto is = [](double a, double b) { return (std::isnan(a) && std::isnan(b)) || a == b; };
  struct Reg {
    uint16_t reg;
    Quantity q;
    double value;
    bool used;
  };
  const Reg regs[] = {
      {reg::kPositionCmd, Quantity::Position, to_turns(c.position, opt_.unit), true},  // always written
      {reg::kVelocityCmd, Quantity::Velocity, to_turns(c.velocity, opt_.unit), !is(c.velocity, 0.0)},
      {reg::kFeedforwardTorque, Quantity::Torque, tqe_adjust(c.torque), !is(c.torque, 0.0)},
      {reg::kKpScale, Quantity::Scale, c.kp_scale, !is(c.kp_scale, 1.0)},
      {reg::kKdScale, Quantity::Scale, c.kd_scale, !is(c.kd_scale, 1.0)},
      {reg::kMaxTorque, Quantity::Torque, tqe_adjust(c.max_torque), !std::isnan(c.max_torque)},
      {reg::kStopPosition, Quantity::Position, to_turns(c.stop_position, opt_.unit), !std::isnan(c.stop_position)},
      {reg::kVelocityLimit, Quantity::Velocity, to_turns(c.vel_limit, opt_.unit), !std::isnan(c.vel_limit)},
      {reg::kAccelLimit, Quantity::Accel, to_turns(c.acc_limit, opt_.unit), !std::isnan(c.acc_limit)},
  };
  const size_t n = sizeof(regs) / sizeof(regs[0]);
  // send[i]: write this register in this frame.  A used register that already holds the value is kept
  // (counted as written, so clear_sticky() does not reset it) but not sent.
  bool send[n];
  uint32_t written = 0;
  for (size_t i = 0; i < n; i++) {
    send[i] = regs[i].used;
    if (i > 0 && regs[i].used) {
      written |= bit(regs[i].reg);
      if (skip_unchanged && pm_cache_valid_ && is(pm_cache_[i - 1], regs[i].value)) send[i] = false;
    }
  }
  FrameBuilder fb;
  fb.set_mode(mode::kPosition);
  for (size_t i = 0; i < n;) {
    if (!send[i]) {
      i++;
      continue;
    }
    // one write for each run of consecutive registers to send
    std::vector<double> values;
    std::vector<Quantity> quantities;
    const uint16_t start = regs[i].reg;
    while (i < n && send[i] && regs[i].reg == start + values.size()) {
      values.push_back(regs[i].value);
      quantities.push_back(regs[i].q);
      i++;
    }
    fb.write(opt_.type, start, values, quantities);
  }
  clear_sticky(fb, written);  // also invalidates the cache ...
  // ... which now holds exactly what the motor has: used registers = their value, the rest = neutral.
  const double neutral[9] = {0.0, 0.0, 1.0, 1.0, kNaN, kNaN, kNaN, kNaN};
  for (size_t i = 1; i < n; i++) pm_cache_[i - 1] = regs[i].used ? regs[i].value : neutral[i - 1];
  pm_cache_valid_ = opt_.clear_sticky_registers;  // without clearing, unused registers are unknown
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_vfoc_lock(double volt) const {
  // Vendor set_vfoc_lock_int16: always int16 on the wire.
  FrameBuilder fb;
  fb.set_mode(mode::kVfocLock);
  fb.write(DataType::Int16, reg::kVfocVoltage, Quantity::Voltage, {volt});
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_mode(uint8_t m) const {
  FrameBuilder fb;
  fb.set_mode(m);
  add_query(fb);
  return fb;
}

FrameBuilder Motor::command_query() const {
  FrameBuilder fb;
  add_query(fb);
  return fb;
}

MotorState Motor::decode(const CanFrame& reply) const {
  MotorState s;
  parse_reply(reply.id, reply.data.data(), reply.data.size(), s);
  const double k = torque_factor(opt_.model);
  if (s.position) s.position = from_turns(*s.position, opt_.unit);
  if (s.velocity) s.velocity = from_turns(*s.velocity, opt_.unit);
  if (s.torque) s.torque = *s.torque * k;  // tqe_restore()
  return s;
}

std::optional<MotorState> Motor::transact(const CanFrame& f) {
  auto replies = bus_.transact({f}, 1, {id_}, opt_.timeout);
  if (replies.empty()) return std::nullopt;
  last_ = decode(replies.front());
  return last_;
}

std::optional<MotorState> Motor::set_dq_volt(double volt) { return transact(frame(command_dq_volt(volt))); }
std::optional<MotorState> Motor::set_dq_current(double c) { return transact(frame(command_dq_current(c))); }
std::optional<MotorState> Motor::set_pos(double pos) { return transact(frame(command_pos(pos))); }
std::optional<MotorState> Motor::set_vel(double vel) { return transact(frame(command_vel(vel))); }
std::optional<MotorState> Motor::set_tqe(double torque) { return transact(frame(command_tqe(torque))); }

std::optional<MotorState> Motor::set_pos_vel(double pos, double vel) {
  return transact(frame(command_pos_vel_max_tqe(pos, vel, kNaN)));
}

std::optional<MotorState> Motor::set_pos_vel_max_tqe(double pos, double vel, double max_torque) {
  return transact(frame(command_pos_vel_max_tqe(pos, vel, max_torque)));
}

std::optional<MotorState> Motor::set_pos_velmax_acc(double pos, double vel_max, double acc) {
  return transact(frame(command_pos_velmax_acc(pos, vel_max, acc)));
}

std::optional<MotorState> Motor::set_vel_acc(double vel, double acc) {
  return transact(frame(command_vel_acc(vel, acc)));
}

std::optional<MotorState> Motor::set_vel_velmax(double vel, double vel_max) {
  return transact(frame(command_vel_velmax(vel, vel_max)));
}

std::optional<MotorState> Motor::set_pos_vel_tqe_kp_kd(double pos, double vel, double torque, double kp, double kd) {
  if (!saved_kp_kd_) saved_kp_kd_ = read_float_registers(reg::kKp, 2);
  auto s = transact(frame(command_pos_vel_tqe_kp_kd(pos, vel, torque, kp, kd)));
  const bool mode_rejected =
      s && std::any_of(s->errors.begin(), s->errors.end(),
                       [](const MotorState::RegisterError& e) { return e.write && e.reg == reg::kMode; });
  if (mode_rejected && saved_kp_kd_) {
    // The motor stays in its old mode but kept the new kp / kd: put the old values back.
    FrameBuilder fb;
    fb.write(DataType::Float, reg::kKp, Quantity::Raw, {(*saved_kp_kd_)[0], (*saved_kp_kd_)[1]});
    add_query(fb);
    transact(frame(fb));
  }
  return s;
}

std::optional<std::vector<double>> Motor::read_float_registers(uint16_t start_reg, uint32_t count) {
  FrameBuilder fb;
  fb.read(DataType::Float, start_reg, count);
  const auto replies = bus_.transact({frame(fb)}, 1, {id_}, opt_.timeout);
  if (replies.empty()) return std::nullopt;
  // Reply: 0x2C|n (or 0x2C n), the register as varuint, then n floats.  0x31 = read error.
  const auto& d = replies.front().data;
  size_t i = 0;
  if (d.empty() || (d[0] & 0xfc) != 0x2c) return std::nullopt;
  uint32_t n = d[i++] & 0x03;
  if (n == 0 && i < d.size()) n = d[i++];
  while (i < d.size() && (d[i++] & 0x80)) {
  }
  std::vector<double> values;
  for (uint32_t k = 0; k < n && k < count && i + 4 <= d.size(); k++, i += 4) {
    float v;
    std::memcpy(&v, &d[i], 4);
    values.push_back(v);
  }
  if (values.size() != count) return std::nullopt;
  return values;
}

std::optional<MotorState> Motor::set_vfoc_lock(double volt) { return transact(frame(command_vfoc_lock(volt))); }
std::optional<MotorState> Motor::brake() { return transact(frame(command_mode(mode::kBrake))); }
std::optional<MotorState> Motor::stop() { return transact(frame(command_mode(mode::kStop))); }
std::optional<MotorState> Motor::query() { return transact(frame(command_query())); }

std::optional<FirmwareVersion> Motor::get_version() {
  FrameBuilder fb;
  fb.read(DataType::Int16, reg::kFirmwareVersion, 1);  // 0x15 0xB5 0x02
  auto s = transact(frame(fb));
  if (!s) return std::nullopt;
  if (s->version) return s->version;
  // Old firmware does not know this register; the vendor code then assumes 3.9.1.
  return FirmwareVersion{3, 9, 1, false};
}

void Motor::reset() {
  FrameBuilder fb;
  fb.append(diagnostic_command("d reset\n"));
  bus_.send(frame(fb));
  sticky_set_ = 0;  // the reboot clears every register
  pm_cache_valid_ = false;
}

bool Motor::set_timeout_ms(int16_t ms) {
  FrameBuilder fb;
  fb.write_raw(DataType::Int16, reg::kTimeoutMs, {ms});  // 0x05 0x1F lo hi
  // With the reply bit set and no read, the motor answers with an empty frame.
  return transact(frame(fb)).has_value();
}

void Motor::set_timed_return_ms(int16_t ms) {
  FrameBuilder fb;
  fb.write_raw(DataType::Int16, reg::kTimedReturnMs, {ms});  // 0x05 0xB4 0x02 lo hi
  bus_.send(frame(fb));
}

bool Motor::diagnostic_with_ack(const std::string& text) {
  FrameBuilder fb;
  fb.append(diagnostic_command(text));
  const CanFrame f = frame(fb);
  // motor_config_closed_loop(): up to 10 tries, 100 ms each.
  for (int i = 0; i < 10; i++) {
    auto replies = bus_.transact({f}, 1, {id_}, milliseconds(100));
    for (const auto& r : replies) {
      if (decode(r).ack) return true;
    }
  }
  return false;
}

int Motor::rezero_and_save() {
  for (int i = 0; i < 3; i++) reset();
  std::this_thread::sleep_for(milliseconds(100));
  if (!diagnostic_with_ack("d cfg-set-output 0.0\n")) return 1;
  if (!diagnostic_with_ack("conf write\n")) return 2;
  for (int i = 0; i < 3; i++) reset();
  std::this_thread::sleep_for(milliseconds(100));
  return 0;
}

int Motor::conf_write() {
  if (!diagnostic_with_ack("conf write\n")) return 1;
  for (int i = 0; i < 3; i++) reset();
  std::this_thread::sleep_for(milliseconds(100));
  return 0;
}

std::vector<std::optional<MotorState>> cycle(Fdcanusb& bus, const std::vector<std::pair<Motor*, FrameBuilder>>& commands) {
  std::chrono::microseconds timeout{0};
  for (const auto& c : commands) timeout = std::max(timeout, c.first->options().timeout);
  return cycle(bus, commands, timeout);
}

std::vector<std::optional<MotorState>> cycle(Fdcanusb& bus, const std::vector<std::pair<Motor*, FrameBuilder>>& commands,
                                             std::chrono::microseconds timeout) {
  std::vector<CanFrame> tx;
  std::vector<uint8_t> ids;
  for (const auto& [motor, fb] : commands) {
    tx.push_back(motor->frame(fb));
    ids.push_back(motor->id());
  }

  auto replies = bus.transact(tx, tx.size(), ids, timeout);
  std::vector<std::optional<MotorState>> result(commands.size());
  for (const auto& r : replies) {
    for (size_t i = 0; i < commands.size(); i++) {
      if (!result[i] && commands[i].first->id() == can_id_source(r.id)) {
        result[i] = commands[i].first->decode(r);
        break;
      }
    }
  }
  return result;
}

std::string format_state(const MotorState& s, AngleUnit unit) {
  const char* u = unit == AngleUnit::Turns ? "r" : unit == AngleUnit::Radians ? "rad" : "deg";
  char buf[256];
  auto v = [](const std::optional<double>& x) { return x ? *x : kNaN; };
  int n = std::snprintf(buf, sizeof(buf), "ID: %2d, mode: %2d (%s), fault: %2d, pos: %8.4f %s, vel: %8.4f %s/s, tqe: %7.4f Nm",
                        s.id, s.mode ? *s.mode : -1, s.mode ? mode_name(*s.mode) : "-", s.fault ? *s.fault : -1,
                        v(s.position), u, v(s.velocity), u, v(s.torque));
  if (s.voltage && n > 0 && n < static_cast<int>(sizeof(buf))) {
    n += std::snprintf(buf + n, sizeof(buf) - n, ", %.1f V", *s.voltage);
  }
  if (s.temperature && n > 0 && n < static_cast<int>(sizeof(buf))) {
    std::snprintf(buf + n, sizeof(buf) - n, ", %.1f C", *s.temperature);
  }
  std::string out = buf;
  if (s.fault && *s.fault != 0) {
    out += "  <-- fault: ";
    out += fault_name(*s.fault);
  }
  for (const auto& e : s.errors) {
    char err[96];
    std::snprintf(err, sizeof(err), "  <-- motor rejected %s of register 0x%03x (code %u)%s",
                  e.write ? "write" : "read", e.reg, e.code,
                  e.write && e.reg == reg::kMode ? ": mode not supported by this firmware" : "");
    out += err;
  }
  return out;
}

// ---------------------------------------------------------------------------
// MotorMany
// ---------------------------------------------------------------------------

namespace many {
constexpr uint8_t kPosition = 0x80;
constexpr uint8_t kVelocity = 0x81;
constexpr uint8_t kTorque = 0x82;
constexpr uint8_t kVoltage = 0x83;
constexpr uint8_t kCurrent = 0x84;
constexpr uint8_t kTimeOut = 0x85;
constexpr uint8_t kPosVelTqe = 0x90;
constexpr uint8_t kPosVelTqeKpKd = 0x93;
constexpr uint8_t kPosVelKpKd = 0x9E;
constexpr uint8_t kPosVelAcc = 0xAD;
constexpr uint8_t kPosVelTqeKpKd2 = 0xB0;
}  // namespace many

MotorMany::MotorMany(Fdcanusb& bus, int num_motors, MotorModel model, AngleUnit unit)
    : bus_(bus), num_motors_(std::clamp(num_motors, 1, 30)), models_(num_motors_, model), unit_(unit) {}

void MotorMany::set_model(uint8_t id, MotorModel model) {
  if (id >= 1 && id <= num_motors_) models_[id - 1] = model;
}

double MotorMany::k(uint8_t id) const {
  return torque_factor(id >= 1 && id <= num_motors_ ? models_[id - 1] : MotorModel::None);
}

size_t MotorMany::slot_bytes() const {
  switch (mode_) {
    case many::kPosVelTqe:
    case many::kPosVelAcc: return 6;
    case many::kPosVelKpKd: return 8;
    case many::kPosVelTqeKpKd:
    case many::kPosVelTqeKpKd2: return 10;
    case 0: return 0;
    default: return 2;
  }
}

int16_t MotorMany::i16(double value, Quantity q) const {
  if (std::isnan(value)) return std::numeric_limits<int16_t>::min();
  const double scaled = std::round(value * counts_per_unit(q, DataType::Int16));
  return static_cast<int16_t>(std::clamp(scaled, -32767.0, 32767.0));
}

void MotorMany::select_mode(uint8_t m) {
  if (mode_ == m) return;
  mode_ = m;
  const size_t per = slot_bytes() / 2;
  slots_.assign(per * num_motors_, 0);
  // Default slot contents, same as motor_many.c: position = NaN, everything else 0.
  const bool first_is_pos = m == many::kPosition || m == many::kPosVelTqe || m == many::kPosVelAcc ||
                            m == many::kPosVelTqeKpKd || m == many::kPosVelTqeKpKd2 || m == many::kPosVelKpKd;
  if (first_is_pos) {
    for (size_t i = 0; i < slots_.size(); i += per) slots_[i] = std::numeric_limits<int16_t>::min();
  }
}

void MotorMany::put(uint8_t id, std::initializer_list<int16_t> values) {
  if (id < 1 || id > num_motors_) return;
  const size_t per = slot_bytes() / 2;
  size_t i = (id - 1) * per;
  for (int16_t v : values) slots_[i++] = v;
}

void MotorMany::dq_volt(uint8_t id, double volt) {
  select_mode(many::kVoltage);
  put(id, {i16(volt, Quantity::Voltage)});
}

void MotorMany::dq_current(uint8_t id, double current) {
  select_mode(many::kCurrent);
  put(id, {i16(current, Quantity::Current)});
}

void MotorMany::pos(uint8_t id, double p) {
  select_mode(many::kPosition);
  put(id, {i16(to_turns(p, unit_), Quantity::Position)});
}

void MotorMany::vel(uint8_t id, double v) {
  select_mode(many::kVelocity);
  put(id, {i16(to_turns(v, unit_), Quantity::Velocity)});
}

void MotorMany::tqe(uint8_t id, double torque) {
  select_mode(many::kTorque);
  put(id, {i16(torque / k(id), Quantity::Torque)});
}

void MotorMany::time_out(uint8_t id, int16_t ms) {
  select_mode(many::kTimeOut);
  put(id, {ms});
}

void MotorMany::pos_vel(uint8_t id, double p, double v) {
  select_mode(many::kPosVelTqe);
  put(id, {i16(to_turns(p, unit_), Quantity::Position), i16(to_turns(v, unit_), Quantity::Velocity),
           std::numeric_limits<int16_t>::min()});
}

void MotorMany::pos_vel_max_tqe(uint8_t id, double p, double v, double max_torque) {
  select_mode(many::kPosVelTqe);
  put(id, {i16(to_turns(p, unit_), Quantity::Position), i16(to_turns(v, unit_), Quantity::Velocity),
           i16(max_torque / k(id), Quantity::Torque)});
}

void MotorMany::pos_vel_acc(uint8_t id, double p, double v, double acc) {
  select_mode(many::kPosVelAcc);
  put(id, {i16(to_turns(p, unit_), Quantity::Position), i16(to_turns(v, unit_), Quantity::Velocity),
           i16(to_turns(acc, unit_), Quantity::Accel)});
}

void MotorMany::pos_vel_tqe_kp_kd(uint8_t id, double p, double v, double torque, double kp, double kd) {
  select_mode(many::kPosVelTqeKpKd);
  put(id, {i16(to_turns(p, unit_), Quantity::Position), i16(to_turns(v, unit_), Quantity::Velocity),
           i16(torque / k(id), Quantity::Torque), i16(kp / k(id), Quantity::Pid), i16(kd / k(id), Quantity::Pid)});
}

void MotorMany::pos_vel_tqe_kp_kd_2(uint8_t id, double p, double v, double torque, double kp, double kd) {
  select_mode(many::kPosVelTqeKpKd2);
  put(id, {i16(to_turns(p, unit_), Quantity::Position), i16(to_turns(v, unit_), Quantity::Velocity),
           i16(torque / k(id), Quantity::Torque), i16(kp / k(id), Quantity::Pid), i16(kd / k(id), Quantity::Pid)});
}

std::vector<CanFrame> MotorMany::frames(Request request) const {
  static const uint8_t kRequest[3][2] = {{0xFF, 0xFF}, {0xFF, 0xFE}, {0x17, 0x01}};
  const uint8_t* q = kRequest[static_cast<int>(request)];

  std::vector<uint8_t> bytes;
  for (int16_t v : slots_) {
    bytes.push_back(static_cast<uint8_t>(v & 0xFF));
    bytes.push_back(static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF));
  }

  std::vector<CanFrame> out;
  const size_t max_chunk = mode_ == many::kPosVelKpKd ? 56 : 60;
  size_t offset = 0;
  uint8_t id = mode_;
  while (offset < bytes.size()) {
    const size_t chunk = std::min(max_chunk, bytes.size() - offset);
    // get_motor_data_len(): data + 2 query bytes, rounded up to a DLC length.
    const size_t len = round_up_dlc(chunk + 2);
    CanFrame f;
    f.id = kReplyFlag | id;
    f.data.assign(len, 0x00);  // filler is 0, like the vendor's static buffer
    std::copy(bytes.begin() + offset, bytes.begin() + offset + chunk, f.data.begin());
    f.data[len - 2] = q[0];
    f.data[len - 1] = q[1];
    out.push_back(std::move(f));
    offset += chunk;
    id++;
  }
  return out;
}

std::vector<std::optional<MotorState>> MotorMany::send(Request request) {
  std::vector<std::optional<MotorState>> result(num_motors_);
  if (mode_ == 0) return result;

  std::vector<uint8_t> ids;
  for (int i = 1; i <= num_motors_; i++) ids.push_back(static_cast<uint8_t>(i));
  auto replies = bus_.transact(frames(request), ids.size(), ids, timeout_);

  for (const auto& r : replies) {
    MotorState s;
    const bool compact = request != Request::PosVelTqe && r.data.size() == 8;
    if (compact) {
      parse_many_reply(r.id, r.data.data(), r.data.size(), s);
    } else {
      parse_reply(r.id, r.data.data(), r.data.size(), s);
    }
    if (s.id < 1 || s.id > num_motors_) continue;
    if (s.position) s.position = from_turns(*s.position, unit_);
    if (s.velocity) s.velocity = from_turns(*s.velocity, unit_);
    if (s.torque) s.torque = *s.torque * k(s.id);
    result[s.id - 1] = s;
  }
  return result;
}

}  // namespace hightorque
