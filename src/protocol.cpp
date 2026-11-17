#include "hightorque/protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>

namespace hightorque {

// ---------------------------------------------------------------------------
// Motor models
// ---------------------------------------------------------------------------

namespace {

struct ModelInfo {
  MotorModel model;
  const char* name;
  float k;
};

// motor_tqe_adj[] from convert.c (fdcan_h730 v3.1.2).
constexpr ModelInfo kModels[] = {
    {MotorModel::M3536_32, "M3536_32", 0.458100f},
    {MotorModel::M4438_30, "M4438_30", 0.525600f},
    {MotorModel::M4438_32, "M4438_32", 0.558400f},
    {MotorModel::M4538_19, "M4538_19", 0.445000f},
    {MotorModel::M5043_20, "M5043_20", 0.966000f},
    {MotorModel::M5046_20, "M5046_20", 0.528000f},
    {MotorModel::M5047_09, "M5047_09", 0.533000f},
    {MotorModel::M5047_36, "M5047_36", 0.803000f},
    {MotorModel::M6056_36, "M6056_36", 0.677000f},
    {MotorModel::M7256_35, "M7256_35", 0.677000f},
    {MotorModel::M60SG_35, "M60SG_35", 0.794200f},
    {MotorModel::M60BM_35, "M60BM_35", 0.794200f},
    {MotorModel::General, "GENERAL", 0.5f},
    {MotorModel::None, "NONE", 1.0f},
};

}  // namespace

float torque_factor(MotorModel model) {
  for (const auto& m : kModels) {
    if (m.model == model) return m.k;
  }
  return 1.0f;
}

const char* motor_model_name(MotorModel model) {
  for (const auto& m : kModels) {
    if (m.model == model) return m.name;
  }
  return "?";
}

std::optional<MotorModel> parse_motor_model(const std::string& name) {
  std::string upper;
  for (char c : name) upper += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  for (const auto& m : kModels) {
    const std::string full = m.name;
    if (upper == full || (full[0] == 'M' && upper == full.substr(1))) return m.model;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------

constexpr double kTwoPi = 6.283185307179586;

double to_turns(double value, AngleUnit unit) {
  switch (unit) {
    case AngleUnit::Radians: return value / kTwoPi;
    case AngleUnit::Degrees: return value / 360.0;
    case AngleUnit::Turns: break;
  }
  return value;
}

double from_turns(double turns, AngleUnit unit) {
  switch (unit) {
    case AngleUnit::Radians: return turns * kTwoPi;
    case AngleUnit::Degrees: return turns * 360.0;
    case AngleUnit::Turns: break;
  }
  return turns;
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

const char* mode_name(uint8_t m) {
  switch (m) {
    case mode::kStop: return "stop";
    case mode::kFault: return "fault";
    case 2: case 3: case 4: return "preparing";
    case mode::kPwm: return "pwm";
    case mode::kVoltage: return "voltage";
    case mode::kVoltageFoc: return "voltage_foc";
    case mode::kVoltageDq: return "voltage_dq";
    case mode::kCurrentDq: return "current_dq";
    case mode::kPosition: return "position";
    case mode::kTimeout: return "timeout";
    case mode::kZeroVelocity: return "zero_velocity";
    case mode::kStayWithin: return "stay_within";
    case mode::kMeasureInductance: return "measure_inductance";
    case mode::kBrake: return "brake";
    case mode::kVfocLock: return "vfoc_lock";
    case mode::kPdControl2: return "pd_control";
    default: return "unknown";
  }
}

// 《表3-电机报错代码说明》
const char* fault_name(uint8_t f) {
  switch (f) {
    case 0: return "ok";
    case 1: return "dma stream transfer error";
    case 2: return "dma stream fifo error";
    case 3: return "uart overrun";
    case 4: return "uart framing error";
    case 5: return "uart noise error";
    case 6: return "uart buffer overrun";
    case 7: return "uart parity error";
    case 32: return "calibration fault (encoder cannot sense magnet)";
    case 33: return "motor driver fault (usually undervoltage / current limit too low)";
    case 34: return "over voltage";
    case 35: return "encoder fault";
    case 36: return "motor not calibrated";
    case 37: return "pwm cycle overrun";
    case 38: return "over temperature";
    case 39: return "start outside position limit";
    case 40: return "under voltage";
    case 41: return "config changed";
    case 42: return "theta invalid";
    case 43: return "position invalid";
    case 44: return "driver enable fault";
    case 45: return "stop position used with accel/velocity limit";
    case 46: return "timing violation";
    case 47: return "bemf feedforward needs accel limit";
    default: return "unknown";
  }
}

// ---------------------------------------------------------------------------
// Scaling
// ---------------------------------------------------------------------------

double counts_per_unit(Quantity q, DataType t) {
  if (t == DataType::Float) return 1.0;
  // Columns: int8, int16, int32.  From PDF section 2 and convert.c.
  struct Row { double i8, i16, i32; };
  Row r{1, 1, 1};
  switch (q) {
    case Quantity::Raw:         r = {1, 1, 1}; break;
    case Quantity::Current:     r = {1, 10, 1000}; break;
    case Quantity::Voltage:     r = {2, 10, 1000}; break;
    case Quantity::Torque:      r = {2, 100, 1000}; break;
    case Quantity::Temperature: r = {1, 10, 1000}; break;
    case Quantity::Time:        r = {100, 1000, 1000000}; break;
    case Quantity::Position:    r = {100, 10000, 100000}; break;
    case Quantity::Velocity:    r = {100, 4000, 100000}; break;
    case Quantity::Accel:       r = {20, 1000, 100000}; break;
    case Quantity::Pid:         r = {1, 10, 1000}; break;
    case Quantity::Scale:       r = {127, 32767, 2147483647.0}; break;
  }
  switch (t) {
    case DataType::Int8: return r.i8;
    case DataType::Int16: return r.i16;
    case DataType::Int32: return r.i32;
    case DataType::Float: break;
  }
  return 1.0;
}

Quantity register_quantity(uint16_t r) {
  switch (r) {
    case reg::kMode: case reg::kFault: return Quantity::Raw;
    case reg::kPosition: case reg::kPositionCmd: case reg::kStopPosition: return Quantity::Position;
    case reg::kVelocity: case reg::kVelocityCmd: case reg::kVelocityLimit: return Quantity::Velocity;
    case reg::kTorque: case reg::kFeedforwardTorque: case reg::kMaxTorque: return Quantity::Torque;
    case reg::kQCurrent: case reg::kDCurrent: case reg::kQCurrentCmd: case reg::kDCurrentCmd:
      return Quantity::Current;
    case reg::kVoltage: case reg::kVfocVoltage: case reg::kDVoltage: case reg::kQVoltage:
      return Quantity::Voltage;
    case reg::kTemperature: return Quantity::Temperature;
    case reg::kAccelLimit: return Quantity::Accel;
    case reg::kKp: case reg::kKd: case reg::kKi: return Quantity::Pid;
    case reg::kKpScale: case reg::kKdScale: return Quantity::Scale;
    default: return Quantity::Raw;
  }
}

namespace {

size_t type_size(DataType t) {
  switch (t) {
    case DataType::Int8: return 1;
    case DataType::Int16: return 2;
    case DataType::Int32: return 4;
    case DataType::Float: return 4;
  }
  return 1;
}

void append_le(std::vector<uint8_t>& out, uint32_t v, size_t n) {
  for (size_t i = 0; i < n; i++) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

int64_t int_min(DataType t) {
  switch (t) {
    case DataType::Int8: return std::numeric_limits<int8_t>::min();
    case DataType::Int16: return std::numeric_limits<int16_t>::min();
    default: return std::numeric_limits<int32_t>::min();
  }
}

int64_t int_max(DataType t) {
  switch (t) {
    case DataType::Int8: return std::numeric_limits<int8_t>::max();
    case DataType::Int16: return std::numeric_limits<int16_t>::max();
    default: return std::numeric_limits<int32_t>::max();
  }
}

// Encode one integer count.
void append_int(std::vector<uint8_t>& out, DataType t, int64_t counts) {
  append_le(out, static_cast<uint32_t>(counts), type_size(t));
}

// Encode one physical value.  NaN -> "no limit" sentinel.
void append_value(std::vector<uint8_t>& out, DataType t, Quantity q, double value) {
  if (t == DataType::Float) {
    const float f = std::isnan(value) ? std::numeric_limits<float>::quiet_NaN() : static_cast<float>(value);
    uint32_t bits;
    std::memcpy(&bits, &f, 4);
    append_le(out, bits, 4);
    return;
  }
  if (std::isnan(value)) {
    append_int(out, t, int_min(t));  // 0x80 / 0x8000 / 0x80000000
    return;
  }
  // Round to nearest (the vendor C code truncates; rounding is more accurate), and clamp so a
  // large value never turns into the NaN sentinel by accident.
  const double scaled = std::round(value * counts_per_unit(q, t));
  const double lo = static_cast<double>(int_min(t) + 1);
  const double hi = static_cast<double>(int_max(t));
  append_int(out, t, static_cast<int64_t>(std::clamp(scaled, lo, hi)));
}

}  // namespace

size_t round_up_dlc(size_t size) {
  if (size <= 8) return size;
  for (size_t s : {12, 16, 20, 24, 32, 48, 64}) {
    if (size <= s) return s;
  }
  return 64;
}

void append_varuint(std::vector<uint8_t>& out, uint32_t value) {
  do {
    uint8_t b = value & 0x7F;
    value >>= 7;
    if (value) b |= 0x80;
    out.push_back(b);
  } while (value);
}

// ---------------------------------------------------------------------------
// FrameBuilder
// ---------------------------------------------------------------------------

void FrameBuilder::header(uint8_t op, DataType type, uint16_t start_reg, uint32_t count) {
  const uint8_t base = static_cast<uint8_t>((op << 4) | (static_cast<uint8_t>(type) << 2));
  if (count >= 1 && count <= 3) {
    // 模式一: count in the low two bits.
    data_.push_back(base | static_cast<uint8_t>(count));
  } else {
    // 模式二: low bits 00, count in the following byte.
    data_.push_back(base);
    append_varuint(data_, count);
  }
  append_varuint(data_, start_reg);
}

FrameBuilder& FrameBuilder::write(DataType type, uint16_t start_reg, std::initializer_list<double> values,
                                  std::initializer_list<Quantity> quantities) {
  return write(type, start_reg, std::vector<double>(values), std::vector<Quantity>(quantities));
}

FrameBuilder& FrameBuilder::write(DataType type, uint16_t start_reg, const std::vector<double>& values,
                                  const std::vector<Quantity>& quantities) {
  header(0x0, type, start_reg, static_cast<uint32_t>(values.size()));
  for (size_t i = 0; i < values.size(); i++) {
    append_value(data_, type, i < quantities.size() ? quantities[i] : Quantity::Raw, values[i]);
  }
  return *this;
}

FrameBuilder& FrameBuilder::write(DataType type, uint16_t start_reg, Quantity q,
                                  std::initializer_list<double> values) {
  header(0x0, type, start_reg, static_cast<uint32_t>(values.size()));
  for (double v : values) append_value(data_, type, q, v);
  return *this;
}

FrameBuilder& FrameBuilder::write_raw(DataType type, uint16_t start_reg, std::initializer_list<int64_t> values) {
  header(0x0, type, start_reg, static_cast<uint32_t>(values.size()));
  for (int64_t v : values) append_int(data_, type, v);
  return *this;
}

FrameBuilder& FrameBuilder::set_mode(uint8_t m) {
  return write_raw(DataType::Int8, reg::kMode, {m});
}

FrameBuilder& FrameBuilder::read(DataType type, uint16_t start_reg, uint32_t count) {
  header(0x1, type, start_reg, count);
  return *this;
}

FrameBuilder& FrameBuilder::query_state(DataType type) {
  read(type, reg::kMode, 4);           // mode, position, velocity, torque
  read(DataType::Int8, reg::kFault, 1);  // fault
  return *this;
}

FrameBuilder& FrameBuilder::query_state_full(DataType type) {
  read(type, reg::kMode, 4);             // mode, position, velocity, torque
  read(DataType::Int16, reg::kVoltage, 3);  // voltage, temperature, fault
  return *this;
}

FrameBuilder& FrameBuilder::append(const std::vector<uint8_t>& bytes) {
  data_.insert(data_.end(), bytes.begin(), bytes.end());
  return *this;
}

std::vector<uint8_t> FrameBuilder::build() const {
  std::vector<uint8_t> out = data_;
  out.resize(round_up_dlc(out.size()), kPadding);
  return out;
}

std::vector<uint8_t> diagnostic_command(const std::string& text) {
  std::vector<uint8_t> out = {0x40, 0x01, static_cast<uint8_t>(text.size())};
  out.insert(out.end(), text.begin(), text.end());
  return out;
}

// ---------------------------------------------------------------------------
// Reply parsing
// ---------------------------------------------------------------------------

namespace {

bool read_varuint(const uint8_t* data, size_t size, size_t& i, uint32_t& value) {
  value = 0;
  for (int shift = 0; shift < 35; shift += 7) {
    if (i >= size) return false;
    const uint8_t b = data[i++];
    value |= static_cast<uint32_t>(b & 0x7F) << shift;
    if (!(b & 0x80)) return true;
  }
  return false;
}

// Decode a "raw" (mode / fault) register.  For float replies the vendor parser takes the low
// byte (p_data[3]); a moteus-style firmware would send e.g. 10.0f instead.  Accept both.
uint8_t decode_raw_byte(DataType t, uint32_t bits) {
  if (t == DataType::Float) {
    float f;
    std::memcpy(&f, &bits, 4);
    if (std::isfinite(f) && f >= 0.0f && f <= 255.0f && f == std::floor(f)) return static_cast<uint8_t>(f);
  }
  return static_cast<uint8_t>(bits & 0xFF);
}

double decode_value(DataType t, Quantity q, uint32_t bits) {
  switch (t) {
    case DataType::Float: {
      float f;
      std::memcpy(&f, &bits, 4);
      return f;
    }
    case DataType::Int8: {
      const int8_t v = static_cast<int8_t>(bits);
      if (v == std::numeric_limits<int8_t>::min()) return std::nan("");
      return v / counts_per_unit(q, t);
    }
    case DataType::Int16: {
      const int16_t v = static_cast<int16_t>(bits);
      if (v == std::numeric_limits<int16_t>::min()) return std::nan("");
      return v / counts_per_unit(q, t);
    }
    case DataType::Int32: {
      const int32_t v = static_cast<int32_t>(bits);
      if (v == std::numeric_limits<int32_t>::min()) return std::nan("");
      return v / counts_per_unit(q, t);
    }
  }
  return 0.0;
}

void store_register(MotorState& s, uint16_t r, DataType t, uint32_t bits) {
  switch (r) {
    case reg::kMode: s.mode = decode_raw_byte(t, bits); return;
    case reg::kFault: s.fault = decode_raw_byte(t, bits); return;
    case reg::kFirmwareVersion: {
      // 16 bit 0xMmmP: major 4 bit, minor 8 bit, patch 4 bit.
      const uint16_t v = static_cast<uint16_t>(bits);
      s.version = FirmwareVersion{(v >> 12) & 0xF, (v >> 4) & 0xFF, v & 0xF};
      return;
    }
    default: break;
  }
  const double value = decode_value(t, register_quantity(r), bits);
  switch (r) {
    case reg::kPosition: s.position = value; break;
    case reg::kVelocity: s.velocity = value; break;
    case reg::kTorque: s.torque = value; break;
    case reg::kQCurrent: s.q_current = value; break;
    case reg::kDCurrent: s.d_current = value; break;
    case reg::kVoltage: s.voltage = value; break;
    case reg::kTemperature: s.temperature = value; break;
    default: break;  // register we do not track
  }
}

}  // namespace

bool parse_reply(uint32_t can_id, const uint8_t* data, size_t size, MotorState& out) {
  out.id = can_id_source(can_id);
  if (size == 0) {
    out.empty = true;
    return true;
  }

  bool parsed_any = false;
  size_t i = 0;
  while (i < size) {
    const uint8_t cmd = data[i];
    if (cmd == kPadding) {
      i++;
      continue;
    }

    if ((cmd & 0xF0) == 0x20) {
      // Reply sub-frame.
      i++;
      const DataType t = static_cast<DataType>((cmd >> 2) & 0x3);
      uint32_t count = cmd & 0x3;
      if (count == 0 && !read_varuint(data, size, i, count)) break;
      uint32_t start = 0;
      if (!read_varuint(data, size, i, start)) break;
      const size_t n = type_size(t);
      if (i + n * count > size) break;
      for (uint32_t k = 0; k < count; k++) {
        uint32_t bits = 0;
        for (size_t b = 0; b < n; b++) bits |= static_cast<uint32_t>(data[i + b]) << (8 * b);
        i += n;
        store_register(out, static_cast<uint16_t>(start + k), t, bits);
      }
      parsed_any = true;
    } else if (cmd == 0x41) {
      // Diagnostic channel response: 0x41 <channel> <len> <bytes...>
      i++;
      uint32_t channel = 0;
      if (!read_varuint(data, size, i, channel)) break;
      if (i >= size) break;
      const size_t len = data[i++];
      if (i + len > size) break;
      out.diagnostic.append(reinterpret_cast<const char*>(data + i), len);
      i += len;
      if (out.diagnostic.find("OK") != std::string::npos) out.ack = true;
      parsed_any = true;
    } else if (cmd == 0x30 || cmd == 0x31) {
      // Write / read error: <register> <error code>.
      i++;
      uint32_t r = 0, e = 0;
      if (!read_varuint(data, size, i, r) || !read_varuint(data, size, i, e)) break;
      out.errors.push_back({cmd == 0x30, r, e});
      parsed_any = true;
    } else {
      break;  // unknown sub-frame, stop
    }
  }
  return parsed_any;
}

bool parse_many_reply(uint32_t can_id, const uint8_t* data, size_t size, MotorState& out) {
  if (size != 8) return false;
  out.id = can_id_source(can_id);
  const uint8_t type = data[1] >> 6;
  out.fault = data[1] & 0x3F;
  if (type == 0) out.mode = data[0];
  if (type == 1) out.temperature = static_cast<int8_t>(data[0]);
  auto i16 = [&](size_t o) { return static_cast<uint32_t>(data[o] | (data[o + 1] << 8)); };
  out.position = decode_value(DataType::Int16, Quantity::Position, i16(2));
  out.velocity = decode_value(DataType::Int16, Quantity::Velocity, i16(4));
  out.torque = decode_value(DataType::Int16, Quantity::Torque, i16(6));
  return true;
}

std::string to_hex(const uint8_t* data, size_t size) {
  static const char* digits = "0123456789ABCDEF";
  std::string s;
  s.reserve(size * 2);
  for (size_t i = 0; i < size; i++) {
    s += digits[data[i] >> 4];
    s += digits[data[i] & 0xF];
  }
  return s;
}

}  // namespace hightorque
