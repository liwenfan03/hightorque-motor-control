// 高擎 (HighTorque) motor CAN-FD protocol: frame encoding / decoding.
//
// This file is pure data manipulation (no I/O), so it can be unit-tested
// without hardware.  Byte layouts follow livelybot_fdcan.c / motor_many.c from
// the vendor STM32 example (fdcan_h730 v3.1.2) and《02-fdcan 协议解析.pdf》.
//
// Protocol summary (see PDF section 1):
//   * CAN ID (16 bit): bit15 = reply requested, bits[14:8] = source, bits[6:0] = destination.
//     0x7F = broadcast.  Low byte >= 0x80 means "一拖多" (one-to-many) mode.
//   * Payload = sequence of sub-frames.  Sub-frame header byte:
//       cmd[7:4] = 0 write / 1 read / 2 reply
//       cmd[3:2] = 0 int8 / 1 int16 / 2 int32 / 3 float
//       cmd[1:0] = count (1..3), or 0 => next byte is the count ("模式二")
//     followed by the start register, then the values (little endian).
//   * Unused tail bytes are padded with 0x50 (NOP).
//   * "No limit / unset" values: int8 0x80, int16 0x8000, int32 0x80000000, float NaN.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hightorque {

// ---------------------------------------------------------------------------
// Basic enums
// ---------------------------------------------------------------------------

// Wire data type.  Affects precision and range (see PDF section 2).
enum class DataType : uint8_t {
  Int8 = 0,
  Int16 = 1,
  Int32 = 2,
  Float = 3,
};

// Motor model, used for the torque correction factor k (real torque = k * tqe).
// Values from motor_tqe_adj[] in convert.c (v3.1.2), which the PDF says is authoritative.
enum class MotorModel {
  M3536_32,
  M4438_30,
  M4438_32,
  M4538_19,
  M5043_20,
  M5046_20,
  M5047_09,
  M5047_36,
  M6056_36,
  M7256_35,
  M60SG_35,
  M60BM_35,
  General,  // k = 0.5 (vendor "MGENERAL")
  None,     // k = 1, no correction (vendor "MNONE")
};

float torque_factor(MotorModel model);
std::optional<MotorModel> parse_motor_model(const std::string& name);
const char* motor_model_name(MotorModel model);

// Unit for position / velocity / acceleration on the user-facing API.
// The motor itself always works in turns (圈数), like MOTOR_DATA_TYPE_FLAG = TURNS.
enum class AngleUnit {
  Turns,
  Radians,
  Degrees,
};

double to_turns(double value, AngleUnit unit);
double from_turns(double turns, AngleUnit unit);

// Motor running mode (register 0x000), see《表2-电机运行模式》.
namespace mode {
constexpr uint8_t kStop = 0;          // 停止，清除错误
constexpr uint8_t kFault = 1;         // 错误
constexpr uint8_t kPwm = 5;
constexpr uint8_t kVoltage = 6;
constexpr uint8_t kVoltageFoc = 7;
constexpr uint8_t kVoltageDq = 8;     // DQ 电压模式
constexpr uint8_t kCurrentDq = 9;     // DQ 电流模式
constexpr uint8_t kPosition = 10;     // 位置模式
constexpr uint8_t kTimeout = 11;      // 超时模式
constexpr uint8_t kZeroVelocity = 12;
constexpr uint8_t kStayWithin = 13;
constexpr uint8_t kMeasureInductance = 14;
constexpr uint8_t kBrake = 15;        // 刹车模式
constexpr uint8_t kVfocLock = 18;     // 0x12, vendor "vfoc 固定模式"
constexpr uint8_t kPdControl2 = 21;   // 0x15, vendor "真运控模式" (motor_set_pos_vel_tqe_kp_kd_2)
}  // namespace mode

const char* mode_name(uint8_t mode);
const char* fault_name(uint8_t fault);

// Register addresses, see《表1-电机寄存器功能》.
namespace reg {
constexpr uint16_t kMode = 0x000;
constexpr uint16_t kPosition = 0x001;
constexpr uint16_t kVelocity = 0x002;
constexpr uint16_t kTorque = 0x003;
constexpr uint16_t kQCurrent = 0x004;
constexpr uint16_t kDCurrent = 0x005;
constexpr uint16_t kVoltage = 0x00d;
constexpr uint16_t kTemperature = 0x00e;
constexpr uint16_t kFault = 0x00f;
constexpr uint16_t kVfocTheta = 0x018;
constexpr uint16_t kVfocVoltage = 0x019;
constexpr uint16_t kDVoltage = 0x01a;
constexpr uint16_t kQVoltage = 0x01b;
constexpr uint16_t kQCurrentCmd = 0x01c;
constexpr uint16_t kDCurrentCmd = 0x01d;
constexpr uint16_t kTimeoutMs = 0x01f;      // used by set_out_time_int16()
constexpr uint16_t kPositionCmd = 0x020;
constexpr uint16_t kVelocityCmd = 0x021;
constexpr uint16_t kFeedforwardTorque = 0x022;
constexpr uint16_t kKpScale = 0x023;
constexpr uint16_t kKdScale = 0x024;
constexpr uint16_t kMaxTorque = 0x025;
constexpr uint16_t kStopPosition = 0x026;
constexpr uint16_t kVelocityLimit = 0x028;
constexpr uint16_t kAccelLimit = 0x029;
constexpr uint16_t kKp = 0x02b;
constexpr uint16_t kKd = 0x02c;
constexpr uint16_t kKi = 0x02d;
constexpr uint16_t kTimedReturnMs = 0x134;  // timed_return_motor_status_int16(): 0x05 0xB4 0x02
constexpr uint16_t kFirmwareVersion = 0x135;  // read_motor_version_int16(): 0x15 0xB5 0x02
}  // namespace reg

// ---------------------------------------------------------------------------
// Scaling: physical value <-> integer on the wire (PDF section 2, convert.c)
// ---------------------------------------------------------------------------

// Physical quantity kinds; each has its own LSB per data type.
enum class Quantity {
  Raw,          // mode, fault: no scaling
  Current,      // A
  Voltage,      // V
  Torque,       // Nm (register value, before/after k correction is done by caller)
  Temperature,  // degC
  Time,         // s
  Position,     // turns
  Velocity,     // turns/s
  Accel,        // turns/s^2
  Pid,          // Kp / Kd (registers 0x2b/0x2c, the "Kp、Kd 标度" of PDF 2.11)
  Scale,        // rKp / rKd and PWM (registers 0x23/0x24), 1/127, 1/32767 ...
};

// How many integer counts make one physical unit for the given type.
// e.g. counts_per_unit(Position, Int16) = 10000  (LSB = 0.0001 turn).
double counts_per_unit(Quantity q, DataType t);

// Quantity stored in a register (used when decoding replies).
Quantity register_quantity(uint16_t reg);

// ---------------------------------------------------------------------------
// Frame building
// ---------------------------------------------------------------------------

constexpr uint16_t kReplyFlag = 0x8000;
constexpr uint8_t kBroadcastId = 0x7F;
constexpr uint8_t kPadding = 0x50;

// Round a payload size up to the next valid CAN-FD DLC length (0-8,12,16,20,24,32,48,64).
size_t round_up_dlc(size_t size);

// Builds one CAN-FD payload out of sub-frames.
class FrameBuilder {
 public:
  // Write `values` (already in physical units) to consecutive registers starting at `start_reg`.
  // NaN encodes "no limit" (0x80, 0x8000, 0x80000000, NaN).
  FrameBuilder& write(DataType type, uint16_t start_reg, std::initializer_list<double> values,
                      std::initializer_list<Quantity> quantities);
  FrameBuilder& write(DataType type, uint16_t start_reg, const std::vector<double>& values,
                      const std::vector<Quantity>& quantities);
  // Same, every value shares one quantity.
  FrameBuilder& write(DataType type, uint16_t start_reg, Quantity q, std::initializer_list<double> values);
  // Write raw integer counts (no scaling).
  FrameBuilder& write_raw(DataType type, uint16_t start_reg, std::initializer_list<int64_t> values);
  // Write the running mode (int8 at register 0x000).
  FrameBuilder& set_mode(uint8_t mode);
  // Read `count` registers of type `type`, starting at `start_reg`.
  FrameBuilder& read(DataType type, uint16_t start_reg, uint32_t count);
  // Standard state query used by every vendor function:
  //   read <type> x4 from 0x000 (mode, position, velocity, torque) + read int8 x1 from 0x00f (fault)
  FrameBuilder& query_state(DataType type);
  // Optional extra query: voltage, temperature, fault as int16 (0x17 0x0d).  Use instead of the
  // int8 fault read when you also want bus voltage and temperature.
  FrameBuilder& query_state_full(DataType type);
  // Raw bytes (used by text "diagnostic" commands).
  FrameBuilder& append(const std::vector<uint8_t>& bytes);

  // Payload padded with 0x50 to a valid DLC.
  std::vector<uint8_t> build() const;
  const std::vector<uint8_t>& unpadded() const { return data_; }

 private:
  void header(uint8_t op, DataType type, uint16_t start_reg, uint32_t count);
  std::vector<uint8_t> data_;
};

// Diagnostic channel text command (vendor: rezero "d cfg-set-output 0.0\n",
// "conf write\n", "d reset\n").  Produces sub-frame 0x40 0x01 <len> <ascii...>.
std::vector<uint8_t> diagnostic_command(const std::string& text);

// Append a little-endian varuint (register addresses > 0x7F use 2 bytes, e.g. 0x135 -> B5 02).
void append_varuint(std::vector<uint8_t>& out, uint32_t value);

// ---------------------------------------------------------------------------
// Reply decoding
// ---------------------------------------------------------------------------

struct FirmwareVersion {
  int major = 0;
  int minor = 0;
  int patch = 0;
  // false: the motor rejected the version register (old firmware); the vendor code then
  // assumes 3.9.1.  Such firmware predates the one-to-many compact replies (>= 4.2.3).
  bool reported = true;
};

// Everything we can learn from one reply frame.  Fields not present in the frame stay empty.
struct MotorState {
  uint8_t id = 0;  // source address of the reply
  std::optional<uint8_t> mode;
  std::optional<double> position;     // turns
  std::optional<double> velocity;     // turns/s
  std::optional<double> torque;       // Nm.  parse_reply(): raw register value; Motor::decode(): k applied
  std::optional<double> q_current;    // A
  std::optional<double> d_current;    // A
  std::optional<double> voltage;      // V
  std::optional<double> temperature;  // degC
  std::optional<uint8_t> fault;
  std::optional<FirmwareVersion> version;
  std::string diagnostic;             // text from the diagnostic channel (e.g. "OK\r\n")
  bool ack = false;                   // diagnostic channel replied "OK" (rezero / conf write)
  bool empty = false;                 // reply with zero data bytes (reply bit set but nothing read)
  // Write / read errors the motor reported (sub-frames 0x30 / 0x31), e.g. a mode the firmware
  // does not support comes back as {write, reg 0x000, code 3}.
  struct RegisterError {
    bool write = true;
    uint32_t reg = 0;
    uint32_t code = 0;
  };
  std::vector<RegisterError> errors;
};

// Parse a normal (sub-frame based) reply.  `can_id` is the received CAN ID.
// Returns false if the payload could not be parsed at all.
bool parse_reply(uint32_t can_id, const uint8_t* data, size_t size, MotorState& out);

// One-to-many compact reply (8 bytes, answer to the 0xFF 0xFF / 0xFF 0xFE query):
//   [0] mode (type 0) or temperature (type 1)
//   [1] bits[5:0] fault, bits[7:6] type
//   [2..7] int16 position, velocity, torque
bool parse_many_reply(uint32_t can_id, const uint8_t* data, size_t size, MotorState& out);

// Source / destination fields of a CAN ID.
inline uint8_t can_id_source(uint32_t can_id) { return (can_id >> 8) & 0x7F; }
inline uint8_t can_id_destination(uint32_t can_id) { return can_id & 0xFF; }

std::string to_hex(const uint8_t* data, size_t size);
inline std::string to_hex(const std::vector<uint8_t>& v) { return to_hex(v.data(), v.size()); }

}  // namespace hightorque
