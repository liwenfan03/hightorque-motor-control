// Offline test (no hardware): the frames we build must be byte-identical to the arrays in the
// vendor livelybot_fdcan.c / motor_many.c, and the reply parser must decode the examples of
// 《02-fdcan 协议解析.pdf》.
//
// Trailing 0x50 padding is ignored when comparing (some vendor arrays pad more than needed).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <limits>
#include <string>
#include <vector>

#include "hightorque/motor.hpp"
#include "hightorque/control.hpp"
#include "hightorque/zero.hpp"

using namespace hightorque;
using Bytes = std::vector<uint8_t>;

static int g_failed = 0;
static int g_passed = 0;

static Bytes strip_padding(Bytes b) {
  while (!b.empty() && b.back() == kPadding) b.pop_back();
  return b;
}

static void check_bytes(const char* name, const Bytes& got, const Bytes& want) {
  const bool dlc_ok = round_up_dlc(got.size()) == got.size();
  if (strip_padding(got) == strip_padding(want) && dlc_ok) {
    g_passed++;
    return;
  }
  g_failed++;
  std::printf("FAIL %s\n  got  %s\n  want %s\n", name, to_hex(got).c_str(), to_hex(want).c_str());
}

static void check_near(const char* name, double got, double want, double tol = 1e-6) {
  if (std::fabs(got - want) <= tol) {
    g_passed++;
    return;
  }
  g_failed++;
  std::printf("FAIL %s: got %.9g want %.9g\n", name, got, want);
}

static void check(const char* name, bool ok) {
  if (ok) {
    g_passed++;
  } else {
    g_failed++;
    std::printf("FAIL %s\n", name);
  }
}

// memcpy a value into a vendor array, like the vendor functions do.
template <typename T>
static void put(Bytes& b, size_t offset, T value) {
  std::memcpy(&b[offset], &value, sizeof(T));
}

int main() {
  Fdcanusb offline(Fdcanusb::Offline{});

  Motor::Options fopt;
  fopt.model = MotorModel::None;  // k = 1, so values go to the wire unchanged
  fopt.type = DataType::Float;
  fopt.clear_sticky_registers = false;  // golden tests: byte-identical to the vendor frames
  Motor mf(offline, 1, fopt);

  Motor::Options i16opt = fopt;
  i16opt.type = DataType::Int16;
  Motor m16(offline, 1, i16opt);

  Motor::Options i32opt = fopt;
  i32opt.type = DataType::Int32;
  Motor m32(offline, 1, i32opt);

  // ---- CAN ID ----
  check("can id = 0x8001", mf.frame(mf.command_query()).id == 0x8001);

  // ---- set_dq_volt_float / int16 ----
  {
    Bytes want = {0x01, 0x00, 0x08, 0x0E, 0x1a, 0, 0, 0, 0, 0, 0, 0, 0, 0x1C, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put(want, 9, 0.5f);
    check_bytes("set_dq_volt_float", mf.frame(mf.command_dq_volt(0.5)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x08, 0x06, 0x1a, 0, 0, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put<int16_t>(want, 7, 15);  // 1.5 V, LSB 0.1 V
    check_bytes("set_dq_volt_int16", m16.frame(m16.command_dq_volt(1.5)).data, want);
  }

  // ---- set_dq_current_float / int32 ----
  {
    Bytes want = {0x01, 0x00, 0x09, 0x0E, 0x1C, 0, 0, 0, 0, 0, 0, 0, 0, 0x1C, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put(want, 5, 0.2f);
    check_bytes("set_dq_current_float", mf.frame(mf.command_dq_current(0.2)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x09, 0x0A, 0x1C, 0, 0, 0, 0, 0, 0, 0, 0, 0x18, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put<int32_t>(want, 5, 200);  // 0.2 A, LSB 0.001 A
    check_bytes("set_dq_current_int32", m32.frame(m32.command_dq_current(0.2)).data, want);
  }

  // ---- set_torque_float / int16 ----
  {
    Bytes want = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc,
                  0xcc, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f,
                  0x1C, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,
                  0x50, 0x50};
    put(want, 14, 0.5f);
    check_bytes("set_torque_float", mf.frame(mf.command_tqe(0.5)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x80, 0x14, 0x04, 0x00, 0x11, 0x0f, 0x50};
    put<int16_t>(want, 10, 30);  // 0.3 Nm, LSB 0.01 Nm
    check_bytes("set_torque_int16", m16.frame(m16.command_tqe(0.3)).data, want);
  }
  {
    // Torque correction: M4438_30, k = 0.5256 -> register = 0.5 / 0.5256
    Motor::Options o = fopt;
    o.model = MotorModel::M4438_30;
    Motor mk(offline, 1, o);
    Bytes want = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x1C, 0x04, 0x00, 0x11,
                  0x0f};
    put(want, 14, static_cast<float>(0.5 / 0.5256f));
    check_bytes("set_torque_float M4438_30 correction", mk.frame(mk.command_tqe(0.5)).data, want);
  }

  // ---- set_pos_vel_tqe_float / int16 (stop position trick) ----
  {
    Bytes want = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0, 0, 0, 0, 0x0e, 0x25, 0, 0, 0, 0, 0, 0,
                  0, 0, 0x1C, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50, 0x50, 0x50};
    put(want, 9, 0.1f);
    put(want, 15, 0.3f);
    put(want, 19, 0.2f);
    check_bytes("set_pos_vel_tqe_float", mf.frame(mf.command_pos_vel_max_tqe(0.2, 0.1, 0.3)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0a, 0x06, 0x20, 0x00, 0x80, 0, 0, 0x06, 0x25, 0, 0, 0, 0,
                  0x14, 0x04, 0x00, 0x11, 0x0f};
    put<int16_t>(want, 7, 400);                                      // 0.1 r/s, LSB 0.00025
    put<int16_t>(want, 11, static_cast<int16_t>(0x8000));            // NaN max torque
    put<int16_t>(want, 13, 2000);                                    // 0.2 r, LSB 0.0001
    check_bytes("set_pos_vel (int16, NaN max tqe)", m16.frame(m16.command_pos_vel_max_tqe(0.2, 0.1, NAN)).data, want);
  }

  // ---- set_pos_int32 / int16, set_vel_float / int16 ----
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x09, 0x20, 0, 0, 0, 0, 0x18, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put<int32_t>(want, 5, 50000);
    check_bytes("set_pos_int32", m32.frame(m32.command_pos(0.5)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x05, 0x20, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f};
    put<int16_t>(want, 5, 5000);
    check_bytes("set_pos_int16", m16.frame(m16.command_pos(0.5)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0, 0, 0, 0, 0x1C, 0x04, 0x00, 0x11, 0x0f,
                  0x50, 0x50};
    put(want, 9, 0.1f);
    check_bytes("set_vel_float", mf.frame(mf.command_vel(0.1)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50};
    put<int16_t>(want, 7, 400);
    check_bytes("set_vel_int16", m16.frame(m16.command_vel(0.1)).data, want);
  }
  {
    // PDF 1.3.3.1.1 example: position mode, position unlimited, velocity 0.1 r/s, int32
    Bytes want = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x10, 0x27, 0x00, 0x00, 0x50, 0x50, 0x50};
    FrameBuilder fb;
    fb.set_mode(mode::kPosition);
    fb.write(DataType::Int32, reg::kPositionCmd, {NAN, 0.1}, {Quantity::Position, Quantity::Velocity});
    check_bytes("PDF example 1.3.3.1.1", fb.build(), want);
  }

  // ---- 运控模式2 float / int16 ----
  {
    Bytes want = {0x01, 0x00, 0x15, 0x0f, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0e, 0x2b, 0, 0, 0, 0, 0, 0, 0,
                  0, 0x1C, 0x04, 0x00, 0x11, 0x0f};
    put(want, 5, 2.0f);
    put(want, 9, 0.1f);
    put(want, 13, 1.0f);
    put(want, 19, 1.5f);
    put(want, 23, 0.25f);
    check_bytes("set_pos_vel_tqe_kp_kd_float_2", mf.frame(mf.command_pos_vel_tqe_kp_kd(2, 0.1, 1, 1.5, 0.25)).data,
                want);
  }
  {
    Bytes want = {0x01, 0x00, 0x15, 0x07, 0x20, 0, 0, 0, 0, 0, 0, 0x06, 0x2b, 0, 0, 0, 0, 0x14, 0x04, 0x00, 0x11,
                  0x0f, 0x50, 0x50};
    put<int16_t>(want, 5, 20000);  // 2 r
    put<int16_t>(want, 7, 400);    // 0.1 r/s
    put<int16_t>(want, 9, 100);    // 1 Nm
    put<int16_t>(want, 13, 15);    // kp 1.5
    put<int16_t>(want, 15, 2);     // kd 0.2 (rounded; LSB 0.1)
    check_bytes("set_pos_vel_tqe_kp_kd_int16_2", m16.frame(m16.command_pos_vel_tqe_kp_kd(2, 0.1, 1, 1.5, 0.2)).data,
                want);
  }

  // ---- trapezoid, vel/acc, vel/velmax ----
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0, 0, 0, 0, 0x0E, 0x28, 0, 0, 0, 0, 0, 0, 0, 0,
                  0x1C, 0x04, 0x00, 0x11, 0x0f};
    put(want, 5, 0.1f);
    put(want, 11, 0.5f);
    put(want, 15, 0.1f);
    check_bytes("set_pos_velmax_acc_float", mf.frame(mf.command_pos_velmax_acc(0.1, 0.5, 0.1)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0, 0, 0x05, 0x29, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f,
                  0x50, 0x50};
    put<int16_t>(want, 7, 400);
    put<int16_t>(want, 11, 400);  // 0.4 r/s^2, LSB 0.001
    check_bytes("set_vel_acc_int16", m16.frame(m16.command_vel_acc(0.1, 0.4)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0, 0, 0x05, 0x28, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f,
                  0x50, 0x50};
    put<int16_t>(want, 7, 400);
    put<int16_t>(want, 11, 800);
    check_bytes("set_vel_velmax_int16", m16.frame(m16.command_vel_velmax(0.1, 0.2)).data, want);
  }
  {
    Bytes want = {0x01, 0x00, 0x12, 0x05, 0x19, 0, 0, 0x14, 0x04, 0x00, 0x11, 0x0f, 0x50, 0x50, 0x50, 0x50};
    put<int16_t>(want, 5, 5);
    check_bytes("set_vfoc_lock_int16", m16.frame(m16.command_vfoc_lock(0.5)).data, want);
  }

  // ---- stop / brake / query / version ----
  check_bytes("set_motor_stop_float", mf.frame(mf.command_mode(mode::kStop)).data,
              {0x01, 0x00, 0x00, 0x1C, 0x04, 0x00, 0x11, 0x0f});
  check_bytes("set_motor_brake_int16", m16.frame(m16.command_mode(mode::kBrake)).data,
              {0x01, 0x00, 0x0f, 0x14, 0x04, 0x00, 0x11, 0x0f});
  check_bytes("read_motor_state_int32", m32.frame(m32.command_query()).data, {0x18, 0x04, 0x00, 0x11, 0x0f});
  // ---- sticky mode-10 registers are reset to neutral (Options::clear_sticky_registers) ----
  {
    Motor::Options o = fopt;
    o.clear_sticky_registers = true;
    Motor m(offline, 1, o);
    const Bytes nan = {0x00, 0x00, 0xc0, 0x7f}, zero = {0, 0, 0, 0}, one = {0x00, 0x00, 0x80, 0x3f};
    auto cat = [](std::initializer_list<Bytes> parts) {
      Bytes b;
      for (const auto& p : parts) b.insert(b.end(), p.begin(), p.end());
      return b;
    };
    const Bytes mode10 = {0x01, 0x00, 0x0a}, query = {0x1C, 0x04, 0x00, 0x11, 0x0f};
    const Bytes pos_half = {0x0d, 0x20, 0x00, 0x00, 0x00, 0x3f};  // position 0.5
    // A new Motor does not know what an earlier program left behind: clear everything once.
    check_bytes("sticky: first set_pos clears all", m.frame(m.command_pos(0.5)).data,
                cat({mode10, pos_half, {0x0c, 0x06, 0x21}, zero, zero, one, one, nan, nan, {0x0e, 0x28}, nan, nan,
                     query}));
    // Nothing left to clear: the vendor frame.
    check_bytes("sticky: second set_pos = vendor frame", m.frame(m.command_pos(0.5)).data,
                cat({mode10, pos_half, query}));
    // set_pos_vel_max_tqe leaves velocity, max torque and stop position set ...
    m.command_pos_vel_max_tqe(0.2, 0.1, 1.0);
    // ... so the next set_pos resets exactly those three.
    check_bytes("sticky: set_pos after set_pos_vel_max_tqe", m.frame(m.command_pos(0.5)).data,
                cat({mode10, pos_half, {0x0d, 0x21}, zero, {0x0e, 0x25}, nan, nan, query}));
    // set_tqe writes kp/kd scale 0; the next set_vel must restore them to 1 (and clear max torque).
    m.command_tqe(0.5);
    const Bytes vel = cat({{0x0e, 0x20}, nan, {0xcd, 0xcc, 0xcc, 0x3d}});  // position NaN, velocity 0.1
    check_bytes("sticky: set_vel after set_tqe", m.frame(m.command_vel(0.1)).data,
                cat({mode10, vel, {0x0c, 0x04, 0x22}, zero, one, one, nan, query}));
    // stop / brake / query do not touch them.
    m.command_mode(mode::kStop);
    check_bytes("sticky: set_vel again = vendor frame", m.frame(m.command_vel(0.1)).data, cat({mode10, vel, query}));
  }

  {
    FrameBuilder fb;
    fb.read(DataType::Int16, reg::kFirmwareVersion, 1);
    check_bytes("read_motor_version_int16", fb.build(), {0x15, 0xB5, 0x02});
  }

  // ---- diagnostic text commands ----
  {
    FrameBuilder fb;
    fb.append(diagnostic_command("d cfg-set-output 0.0\n"));
    check_bytes("set_pos_rezero", fb.build(),
                {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74,
                 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a});
  }
  {
    FrameBuilder fb;
    fb.append(diagnostic_command("conf write\n"));
    check_bytes("set_conf_write", fb.build(),
                {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50});
  }
  {
    FrameBuilder fb;
    fb.append(diagnostic_command("d reset\n"));
    check_bytes("set_motor_reset_int8", fb.build(),
                {0x40, 0x01, 0x08, 0x64, 0x20, 0x72, 0x65, 0x73, 0x65, 0x74, 0x0A, 0x50});
  }
  {
    FrameBuilder fb;
    fb.write_raw(DataType::Int16, reg::kTimeoutMs, {500});
    check_bytes("set_out_time_int16", fb.build(), {0x05, 0x1f, 0xF4, 0x01});
  }
  {
    FrameBuilder fb;
    fb.write_raw(DataType::Int16, reg::kTimedReturnMs, {10});
    check_bytes("timed_return_motor_status_int16", fb.build(), {0x05, 0xb4, 0x02, 0x0A, 0x00});
  }

  // ---- reply parsing: PDF 1.3.3.2.1 (模式一, int32 x3 from 0x01) ----
  {
    const Bytes r = {0x2B, 0x01, 0x96, 0x1D, 0x04, 0x00, 0x54, 0x40, 0x02, 0x00, 0x86, 0x01, 0x00, 0x00, 0x50, 0x50};
    MotorState s;
    check("parse PDF mode-1 reply", parse_reply(0x0100, r.data(), r.size(), s));
    check("  id", s.id == 1);
    check_near("  position", s.position.value_or(NAN), 2.69718);
    check_near("  velocity", s.velocity.value_or(NAN), 1.4754);
    check_near("  torque (int32 LSB 0.001 Nm)", s.torque.value_or(NAN), 0.390);
  }
  // ---- PDF 1.3.3.2.2 (模式二, int32 x4 from 0x00) ----
  {
    const Bytes r = {0x28, 0x04, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x96, 0x1D, 0x04, 0x00,
                     0x54, 0x40, 0x02, 0x00, 0x86, 0x01, 0x00, 0x00, 0x50};
    MotorState s;
    check("parse PDF mode-2 reply", parse_reply(0x0100, r.data(), r.size(), s));
    check("  mode = 10", s.mode.value_or(255) == 10);
    check_near("  position", s.position.value_or(NAN), 2.69718);
  }
  // ---- standard int16 state reply, as parsed by motor_process_state() ----
  {
    Bytes r = {0x24, 0x04, 0x00, 0x0A, 0x00, 0, 0, 0, 0, 0, 0, 0x21, 0x0F, 33, 0x50, 0x50};
    put<int16_t>(r, 5, -2500);  // -0.25 r
    put<int16_t>(r, 7, 400);    // 0.1 r/s
    put<int16_t>(r, 9, 12);     // 0.12 Nm
    Fdcanusb off(Fdcanusb::Offline{});
    Motor::Options o;
    o.model = MotorModel::M4438_30;
    Motor m(off, 2, o);
    CanFrame f;
    f.id = 0x0200;
    f.data = r;
    const MotorState s = m.decode(f);
    check("int16 reply id", s.id == 2);
    check("int16 reply mode", s.mode.value_or(255) == 10);
    check_near("int16 reply position", s.position.value_or(NAN), -0.25);
    check_near("int16 reply velocity", s.velocity.value_or(NAN), 0.1);
    check_near("int16 reply torque * k", s.torque.value_or(NAN), 0.12 * 0.5256f, 1e-6);
    check("int16 reply fault", s.fault.value_or(0) == 33);
  }
  // ---- float reply with mode sent as a raw byte (vendor parse) and as 10.0f (moteus style) ----
  {
    Bytes r = {0x2C, 0x04, 0x00, 0x0A, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x21, 0x0F, 0x00};
    put(r, 7, 1.25f);
    MotorState s;
    parse_reply(0x0100, r.data(), r.size(), s);
    check("float reply, raw mode byte", s.mode.value_or(255) == 10);
    check_near("float reply position", s.position.value_or(NAN), 1.25);
    put(r, 3, 15.0f);
    MotorState s2;
    parse_reply(0x0100, r.data(), r.size(), s2);
    check("float reply, mode as 15.0f", s2.mode.value_or(255) == 15);
  }
  // ---- write error before the state (real reply of old firmware to mode 21, 2026-11-04) ----
  {
    Bytes r = {0x30, 0x00, 0x03, 0x2C, 0x04, 0x00};  // error, then read-reply header (float x4 from 0x00)
    r.resize(r.size() + 16, 0);                      // mode, position, velocity, torque
    r.insert(r.end(), {0x21, 0x0F, 0x00});           // fault
    put(r, 10, 0.2f);
    MotorState s;
    check("write-error reply parsed", parse_reply(0x0200, r.data(), r.size(), s));
    check("write error recorded", s.errors.size() == 1 && s.errors[0].write && s.errors[0].reg == 0 &&
                                      s.errors[0].code == 3);
    check("state after write error", s.mode.value_or(255) == 0);
    check_near("position after write error", s.position.value_or(NAN), 0.2, 1e-6);
  }
  // ---- version reply 0x25 0xB5 0x02 lo hi ----
  {
    const Bytes r = {0x25, 0xB5, 0x02, 0x53, 0x41};  // 0x4153 -> 4.21.3
    MotorState s;
    parse_reply(0x0100, r.data(), r.size(), s);
    check("version parsed", s.version.has_value());
    if (s.version) check("version = 4.21.3", s.version->major == 4 && s.version->minor == 21 && s.version->patch == 3);
  }
  // ---- rezero / conf write ack ----
  {
    const Bytes r = {0x41, 0x01, 0x04, 'O', 'K', '\r', '\n'};
    MotorState s;
    parse_reply(0x0100, r.data(), r.size(), s);
    check("diagnostic OK ack", s.ack);
  }
  // ---- empty reply ----
  {
    MotorState s;
    check("empty reply", parse_reply(0x0300, nullptr, 0, s) && s.empty && s.id == 3);
  }

  // ---- 一拖多 ----
  {
    MotorMany many(offline, 2, MotorModel::None);
    many.vel(1, 0.1);
    many.vel(2, -0.1);
    auto f = many.frames(MotorMany::Request::TempFaultPosVelTqe);
    check("many vel: one frame", f.size() == 1);
    check("many vel: id 0x8081", f[0].id == 0x8081);
    check_bytes("many vel payload", f[0].data, {0x90, 0x01, 0x70, 0xFE, 0xFF, 0xFE});
  }
  {
    MotorMany many(offline, 2, MotorModel::None);
    many.pos_vel(1, 0.5, 0.1);
    auto f = many.frames(MotorMany::Request::PosVelTqe);
    // Motor 1: pos 5000, vel 400, max tqe NaN.  Motor 2 untouched: pos NaN, vel 0, tqe 0.
    check("many pos_vel: id 0x8090", f.size() == 1 && f[0].id == 0x8090);
    check("many pos_vel: 16 bytes", f[0].data.size() == 16);
    check_bytes("many pos_vel payload", f[0].data,
                {0x88, 0x13, 0x90, 0x01, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x01});
  }
  {
    MotorMany many(offline, 12, MotorModel::None);
    for (int i = 1; i <= 12; i++) many.pos_vel_tqe_kp_kd_2(static_cast<uint8_t>(i), 0, 0, 0, 1, 0.1);
    auto f = many.frames(MotorMany::Request::ModeFaultPosVelTqe);
    // 10 bytes per motor, max 60 data bytes per frame -> 6 motors per frame -> 2 frames.
    check("many kp_kd_2: 2 frames", f.size() == 2);
    check("many kp_kd_2: ids 0xB0, 0xB1", f.size() == 2 && f[0].id == 0x80B0 && f[1].id == 0x80B1);
    check("many kp_kd_2: 64 byte frames", f.size() == 2 && f[0].data.size() == 64);
  }
  {
    // Compact reply: temperature 35 C, type 1, fault 0, pos 0.5 r, vel -0.1 r/s, tqe 0.2 Nm
    Bytes r = {35, 0x40, 0, 0, 0, 0, 0, 0};
    put<int16_t>(r, 2, 5000);
    put<int16_t>(r, 4, -400);
    put<int16_t>(r, 6, 20);
    MotorState s;
    check("many compact reply", parse_many_reply(0x0200, r.data(), r.size(), s));
    check("  temperature", s.temperature.value_or(0) == 35 && !s.mode);
    check_near("  position", s.position.value_or(NAN), 0.5);
    check_near("  velocity", s.velocity.value_or(NAN), -0.1);
    check_near("  torque", s.torque.value_or(NAN), 0.2);
  }

  // ---- scaling edge cases ----
  {
    FrameBuilder fb;
    fb.write(DataType::Int16, reg::kPositionCmd, Quantity::Position, {100.0});  // way out of int16 range
    const Bytes b = fb.unpadded();
    const int16_t v = static_cast<int16_t>(b[2] | (b[3] << 8));
    check("int16 clamps to 32767, never to the NaN value", v == 32767);
  }

  // ---- host-side zero file ----
  {
    char tmpl[] = "/tmp/ht_zero_test_XXXXXX";
    const char* dir = mkdtemp(tmpl);
    check("zero: temp dir", dir != nullptr);
    if (dir) {
      const std::string f = std::string(dir) + "/sub/zero.txt";  // sub/ does not exist yet
      check("zero: load from missing file", !load_zero(f, 2));
      ZeroRecord a{2, 5.302012, "2026-11-05 14:00:00"};
      ZeroRecord b{1, -0.25, "2026-11-05 14:01:00"};
      check("zero: save id 2 (creates dir)", save_zero(f, a));
      check("zero: save id 1", save_zero(f, b));
      auto la = load_zero(f, 2);
      auto lb = load_zero(f, 1);
      check("zero: load id 2", la && std::fabs(la->position - 5.302012) < 1e-9 && la->time == a.time);
      check("zero: load id 1", lb && std::fabs(lb->position + 0.25) < 1e-9);
      check("zero: unknown id", !load_zero(f, 3));
      ZeroRecord a2{2, 0.1234, "2026-11-05 15:00:00"};
      check("zero: overwrite id 2", save_zero(f, a2));
      la = load_zero(f, 2);
      lb = load_zero(f, 1);
      check("zero: id 2 replaced", la && std::fabs(la->position - 0.1234) < 1e-9);
      check("zero: id 1 kept", lb && std::fabs(lb->position + 0.25) < 1e-9);
      std::remove(f.c_str());
      std::remove((std::string(dir) + "/sub").c_str());
      std::remove(dir);
    }
  }

  // ---- angle relative to the zero (whole turns ignored) ----
  {
    auto near = [](double a, double b) { return std::fabs(a - b) < 1e-9; };
    check("wrap: 0.3", near(wrap_turns(0.3), 0.3));
    check("wrap: 0.7 -> -0.3", near(wrap_turns(0.7), -0.3));
    check("wrap: -0.7 -> 0.3", near(wrap_turns(-0.7), 0.3));
    check("wrap: 2.25 -> 0.25", near(wrap_turns(2.25), 0.25));
    check("wrap: range [-0.5, 0.5)", near(wrap_turns(0.5), -0.5) && near(wrap_turns(-0.5), -0.5));
    // Measured on 5047 #1: zero 0.0713; 1.0716 r before a power cycle (extra turn) is the same place.
    check("eff zero: extra turn", near(effective_zero(0.0713, 1.0716), 1.0713));
    check("eff zero: same frame", near(effective_zero(0.0713, -0.0691), 0.0713));
    check("eff zero: zero stored with extra turns", near(effective_zero(1.0713, 0.0716), 0.0713));
    check("eff zero: -2.67725 frame", near(effective_zero(0.32292, -2.67725), -2.67708));
    check("eff zero: offset always within half a turn",
          std::fabs(0.0713 + 0.6 - effective_zero(0.0713, 0.0713 + 0.6)) <= 0.5);
  }

  // ---- hardcoded joint limits ----
  {
    for (int id = 0; id < 128; id++) {
      if (const auto l = joint_limits(static_cast<uint8_t>(id)); l && !l->continuous) {
        check(std::string("limits: strictly inside +-180 deg (ID " + std::to_string(id) + ")").c_str(),
              l->min_offset > -0.5 && l->max_offset < 0.5 && l->min_offset < l->max_offset);
        check(std::string("limits: room for the 3.6 deg margin (ID " + std::to_string(id) + ")").c_str(),
              l->min_offset - 0.01 > -0.5 && l->max_offset + 0.01 < 0.5);
      }
    }
    check("limits: ID 3 not hardcoded", !joint_limits(3));
    const auto l2 = joint_limits(2);
    const auto l1 = joint_limits(1);
    check("limits: ID 2 hardcoded", l2.has_value());
    check("limits: ID 1 hardcoded", l1.has_value());
    if (l2) {
      const double z = 0.07153;  // application zero of 5047 #1 (raw, effective zero in this frame)
      check("limits: ID 2 is +-90 deg", std::fabs(l2->min_offset + 0.25) < 1e-12 && std::fabs(l2->max_offset - 0.25) < 1e-12);
      check("limits: at zero inside", within_limits(*l2, z, z));
      check("limits: +89 deg inside", within_limits(*l2, z, z + 89.0 / 360.0));
      check("limits: -89 deg inside", within_limits(*l2, z, z - 89.0 / 360.0));
      check("limits: -95 deg outside", !within_limits(*l2, z, z - 95.0 / 360.0));
      check("limits: -92 deg inside with 3.6 deg margin", within_limits(*l2, z, z - 92.0 / 360.0, 0.01));
      check("limits: -95 deg outside with 3.6 deg margin", !within_limits(*l2, z, z - 95.0 / 360.0, 0.01));
      // Joint turned to -100 deg while off, reading carries +1 turn: still seen as -100 deg.
      const double pos = z - 100.0 / 360.0 + 1.0;
      check("limits: -100 deg with extra turn is outside", !within_limits(*l2, effective_zero(z, pos), pos, 0.01));
      const double pos2 = z + 30.0 / 360.0 - 1.0;
      check("limits: +30 deg with -1 turn is inside", within_limits(*l2, effective_zero(z, pos2), pos2, 0.01));
      check("limits: clamp high", std::fabs(clamp_to_limits(*l2, z, z + 0.4) - (z + 0.25)) < 1e-12);
      check("limits: clamp low", std::fabs(clamp_to_limits(*l2, z, z - 0.4) - (z - 0.25)) < 1e-12);
      check("limits: clamp inside unchanged", std::fabs(clamp_to_limits(*l2, z, z + 0.1) - (z + 0.1)) < 1e-12);
    }
    if (l1) {
      const double z = 0.2266;
      check("limits: ID 1 is +-170 deg, not continuous", !l1->continuous &&
            std::fabs(l1->max_offset * 360.0 - 170.0) < 1e-9 && std::fabs(l1->min_offset * 360.0 + 170.0) < 1e-9);
      check("limits: ID 1 +165 deg inside", within_limits(*l1, z, z + 165.0 / 360.0));
      check("limits: ID 1 +178 deg outside (no-stop band)", !within_limits(*l1, z, z + 178.0 / 360.0, 0.01));
      // +178 deg reads the same as -182 deg; wrapped it is +178 -> outside either way.
      const double p = z - 182.0 / 360.0;
      check("limits: ID 1 -182 deg seen as +178, outside", !within_limits(*l1, effective_zero(z, p), p, 0.01));
      // Reading carries +1 turn (as after the 2026-11-10 go-zero test): still -0.35 deg, inside.
      const double r = z + 1.0 - 0.001;
      check("limits: ID 1 extra turn ignored", within_limits(*l1, effective_zero(z, r), r));
      // From +160 to -160 the target is 320 deg away the long way, never across 180.
      const double from = z + 160.0 / 360.0, to = clamp_to_limits(*l1, z, z - 160.0 / 360.0);
      check("limits: ID 1 +160 -> -160 goes the long way", std::fabs((to - from) * 360.0 + 320.0) < 1e-9);
    }
  }

  // ---- synchronized multi-motor go-zero: trajectory ----
  {
    check("min_jerk: 0 at start", std::fabs(min_jerk(0.0)) < 1e-12);
    check("min_jerk: 1 at end", std::fabs(min_jerk(1.0) - 1.0) < 1e-12);
    check("min_jerk: 0.5 at half", std::fabs(min_jerk(0.5) - 0.5) < 1e-12);
    check("min_jerk: clamped", std::fabs(min_jerk(-1.0)) < 1e-12 && std::fabs(min_jerk(2.0) - 1.0) < 1e-12);
    bool mono = true;
    for (int k = 1; k <= 100; k++) mono = mono && min_jerk(k / 100.0) >= min_jerk((k - 1) / 100.0);
    check("min_jerk: monotonic", mono);
    // numeric peak velocity / acceleration of s over [0,1] match 1.875 and 5.7735
    double vmax = 0, amax = 0;
    const double h = 1e-4;
    for (int k = 1; k < 10000; k++) {
      const double t = k * h;
      vmax = std::max(vmax, (min_jerk(t + h) - min_jerk(t - h)) / (2 * h));
      amax = std::max(amax, std::fabs(min_jerk(t + h) - 2 * min_jerk(t) + min_jerk(t - h)) / (h * h));
    }
    check("min_jerk: peak velocity 1.875", std::fabs(vmax - 1.875) < 1e-3);
    check("min_jerk: peak acceleration 5.7735", std::fabs(amax - 5.7735) < 2e-3);
    check("duration: 0 for no move", min_jerk_duration(0.0, 0.2, 0.5) == 0.0);
    // 0.16 r at 0.2 r/s, 0.5 r/s^2: velocity bound 1.5 s, acceleration bound sqrt(1.847) = 1.359 s
    check("duration: velocity bound", std::fabs(min_jerk_duration(0.16, 0.2, 0.5) - 1.5) < 1e-9);
    check("duration: acceleration bound", std::fabs(min_jerk_duration(0.01, 0.2, 0.5) - std::sqrt(5.7735 * 0.01 / 0.5)) < 1e-9);
    check("duration: sign ignored", min_jerk_duration(-0.16, 0.2, 0.5) == min_jerk_duration(0.16, 0.2, 0.5));
    // synchronized: with the common T, the shorter move's peaks stay below the limits
    const double T = std::max(min_jerk_duration(0.37, 0.1, 0.3), min_jerk_duration(0.16, 0.1, 0.3));
    check("sync: longer move sets T", std::fabs(T - min_jerk_duration(0.37, 0.1, 0.3)) < 1e-12);
    check("sync: shorter move within limits", 1.875 * 0.16 / T <= 0.1 && 5.7735 * 0.16 / (T * T) <= 0.3);
  }

  // ---- generic position-mode command == the vendor cases ----
  {
    Motor::Options o;
    o.model = MotorModel::M5047_36;
    for (DataType t : {DataType::Float, DataType::Int16}) {
      o.type = t;
      const char* tn = t == DataType::Float ? "float" : "int16";
      auto same = [&](const char* name, auto make_case, PositionModeCommand c) {
        Motor a(offline, 2, o), b(offline, 2, o);  // fresh objects: same sticky state
        check_bytes((std::string("generic = ") + name + " (" + tn + ")").c_str(), b.command_position_mode(c).build(),
                    make_case(a).build());
      };
      PositionModeCommand c2; c2.position = 0.1;
      same("case 2 pos", [](Motor& m) { return m.command_pos(0.1); }, c2);
      PositionModeCommand c3; c3.velocity = 0.2;
      same("case 3 vel", [](Motor& m) { return m.command_vel(0.2); }, c3);
      PositionModeCommand c7; c7.position = 0.1; c7.vel_limit = 0.3; c7.acc_limit = 1.0;
      same("case 7 pos_velmax_acc", [](Motor& m) { return m.command_pos_velmax_acc(0.1, 0.3, 1.0); }, c7);
      PositionModeCommand c6; c6.velocity = 0.1; c6.max_torque = 1.0; c6.stop_position = 0.2;
      same("case 6 pos_vel_max_tqe", [](Motor& m) { return m.command_pos_vel_max_tqe(0.2, 0.1, 1.0); }, c6);
    }
    // sticky: after a case 7 frame, a plain position command resets 0x028 / 0x029
    // (a new Motor clears every sticky register in its first frame, so start both after one plain frame)
    PositionModeCommand c7; c7.position = 0.1; c7.vel_limit = 0.3; c7.acc_limit = 1.0;
    PositionModeCommand c2; c2.position = 0.1;
    Motor m(offline, 2, o), ref(offline, 2, o);
    m.command_position_mode(c2);
    ref.command_position_mode(c2);
    m.command_position_mode(c7);
    ref.command_position_mode(c2);
    check("generic: clears registers of the previous command",
          m.command_position_mode(c2).unpadded().size() > ref.command_position_mode(c2).unpadded().size());
    check("generic: nothing to clear -> position only",
          ref.command_position_mode(c2).unpadded().size() <= 14);  // mode 3 + position 6 + query 5 (float)
  }

  // ---- position mode: send only changed registers ----
  {
    Motor::Options o;
    o.model = MotorModel::M5047_36;
    PositionModeCommand c7; c7.position = 0.1; c7.vel_limit = 0.3; c7.acc_limit = 1.0; c7.max_torque = 1.0;
    PositionModeCommand c2; c2.position = 0.1;
    Motor clean(offline, 2, o);
    clean.command_position_mode(c2);
    const Bytes pos_only = clean.command_position_mode(c2).build();  // mode + position + query

    Motor m(offline, 2, o);
    const Bytes first = m.command_position_mode_changed(c7).build();
    check("changed: first frame sends everything", first.size() > pos_only.size());
    check_bytes("changed: unchanged registers not sent again", m.command_position_mode_changed(c7).build(), pos_only);
    PositionModeCommand c7b = c7; c7b.position = 0.2;
    Motor clean2(offline, 2, o);
    clean2.command_position_mode(c2);
    PositionModeCommand p2; p2.position = 0.2;
    check_bytes("changed: new position, same registers", m.command_position_mode_changed(c7b).build(),
                clean2.command_position_mode(p2).build());
    PositionModeCommand c7c = c7b; c7c.vel_limit = 0.5;
    const size_t one_changed = m.command_position_mode_changed(c7c).unpadded().size();
    check("changed: one changed register is sent", one_changed > pos_only.size() - 0 && one_changed < first.size());
    // a full frame of the same command equals what command_position_mode() sends
    Motor m3(offline, 2, o), m4(offline, 2, o);
    m3.command_position_mode_changed(c7); m4.command_position_mode(c7);
    check_bytes("changed: full frame unchanged", m3.command_position_mode(c7).build(), m4.command_position_mode(c7).build());
    // another command in between forgets the cache
    m.command_tqe(0.0);
    check("changed: cache dropped after another command",
          m.command_position_mode_changed(c7).unpadded().size() > pos_only.size());
    // dropping a register: it is reset (cleared), not silently left at the old value
    PositionModeCommand c_no_acc = c7; c_no_acc.acc_limit = std::numeric_limits<double>::quiet_NaN();
    m.command_position_mode_changed(c7);
    check("changed: removed register is cleared",
          m.command_position_mode_changed(c_no_acc).unpadded().size() > pos_only.size());
    check_bytes("changed: then stable", m.command_position_mode_changed(c_no_acc).build(), pos_only);
  }

  // ---- short torque frame (pipelined torque / impedance) ----
  {
    Motor::Options o;
    o.model = MotorModel::M5047_36;
    Motor a(offline, 2, o), b(offline, 2, o);
    a.command_tqe(0.1);
    b.command_tqe(0.1);
    b.command_tqe_short(0.1);  // clears the 0x025 that command_tqe had written, once
    const Bytes full = a.command_tqe(0.2).build(), shrt = b.command_tqe_short(0.2).build();
    check("tqe_short: 32 byte (float), command_tqe 48", shrt.size() == 32 && full.size() == 48);
    // same values: mode, then one write of 5 registers from 0x020 (full: 6), then the query
    const Bytes sh = strip_padding(shrt);
    check("tqe_short: writes 0x020..0x024", sh.size() >= 6 && sh[3] == 0x0C && sh[4] == 0x05 && sh[5] == 0x20);
    // a max torque left over from a position command is cleared once, then the frame stays short
    PositionModeCommand c; c.position = 0.1; c.max_torque = 1.0;
    Motor m(offline, 2, o);
    m.command_position_mode(c);
    m.command_position_mode(c);
    const size_t first = m.command_tqe_short(0.1).unpadded().size(), second = m.command_tqe_short(0.1).unpadded().size();
    check("tqe_short: leftover 0x025 cleared once", first > second && round_up_dlc(second) == 32);
  }

  // ---- shortest torque frame ----
  {
    Motor::Options o;
    o.model = MotorModel::M5047_36;
    Motor m(offline, 2, o);
    m.command_tqe(0.0);     // as the pipelined warm-up: writes 0x020..0x025
    m.command_tqe_min(0.1);  // clears the leftover velocity / max torque once
    const Bytes f = strip_padding(m.command_tqe_min(0.2).build());
    check("tqe_min: 24 byte (float)", m.command_tqe_min(0.2).build().size() == 24);
    check("tqe_min: one write of 3 float registers from 0x022 (0F 22)", f.size() >= 5 && f[3] == 0x0F && f[4] == 0x22);
    PositionModeCommand c; c.position = 0.1; c.max_torque = 1.0;
    Motor p(offline, 2, o);
    p.command_position_mode(c);
    p.command_position_mode(c);
    const size_t first = p.command_tqe_min(0.1).unpadded().size(), second = p.command_tqe_min(0.1).unpadded().size();
    check("tqe_min: leftover 0x025 cleared once", first > second && round_up_dlc(second) == 24);
  }

  // ---- host setpoint shaper (方案 A) ----
  {
    const double dt = 0.001;
    // 20 deg step, vel_max 0.3 r/s, acc 1 r/s^2: triangular, peak sqrt(d * acc) = 0.236 r/s, T = 2 sqrt(d / acc)
    SetpointShaper sh;
    sh.vel_max = 0.3;
    sh.acc = 1.0;
    sh.reset(0.0);
    const double d = 20.0 / 360.0;
    double vmax = 0, amax = 0, overshoot = 0, prev_v = 0, t_arrive = -1;
    for (int k = 1; k <= 2000; k++) {
      sh.update(d, 0.0, dt);
      vmax = std::max(vmax, std::fabs(sh.vel));
      amax = std::max(amax, std::fabs(sh.vel - prev_v) / dt);
      prev_v = sh.vel;
      overshoot = std::max(overshoot, sh.pos - d);
      if (t_arrive < 0 && sh.pos == d) t_arrive = k * dt;
    }
    check("shaper: step arrives exactly", sh.pos == d && sh.vel == 0.0);
    check_near("shaper: step time = 2 sqrt(d / acc)", t_arrive, 2.0 * std::sqrt(d / 1.0), 0.01);
    check_near("shaper: peak velocity = sqrt(d acc)", vmax, std::sqrt(d * 1.0), 0.005);
    check("shaper: acceleration limited", amax <= 1.0 + 1e-6);
    check("shaper: no overshoot (< 1e-6 r)", overshoot <= 1e-6);
    // long step: cruises at vel_max
    sh.reset(0.0);
    double vm2 = 0;
    for (int k = 0; k < 5000; k++) { sh.update(0.5, 0.0, dt); vm2 = std::max(vm2, std::fabs(sh.vel)); }
    check("shaper: long step cruises at vel_max", std::fabs(vm2 - 0.3) < 1e-9 && sh.pos == 0.5);
    // negative step
    sh.reset(0.0);
    for (int k = 0; k < 2000; k++) sh.update(-d, 0.0, dt);
    check("shaper: negative step arrives", sh.pos == -d && sh.vel == 0.0);
    // sine within the limits (20 deg, 0.25 Hz): followed without lag once converged
    sh.reset(0.0);
    const double A = 20.0 / 360.0, w = 2 * M_PI * 0.25;
    double err = 0;
    for (int k = 1; k <= 8000; k++) {
      const double t = k * dt;
      sh.update(A * std::sin(w * t), A * w * std::cos(w * t), dt);
      if (t > 2.0) err = std::max(err, std::fabs(sh.pos - A * std::sin(w * t)));
    }
    check("shaper: slow sine tracked (< 0.05 deg)", err * 360.0 < 0.05);
    // target faster than vel_max: command speed stays limited
    sh.reset(0.0);
    double vm3 = 0;
    for (int k = 1; k <= 4000; k++) {
      const double t = k * dt;
      sh.update(0.2 * std::sin(2 * M_PI * 2.0 * t), 0.2 * 4 * M_PI * std::cos(2 * M_PI * 2.0 * t), dt);
      vm3 = std::max(vm3, std::fabs(sh.vel));
    }
    check("shaper: fast target -> velocity limited", vm3 <= 0.3 + 1e-9);
    check("shaper: NaN target ignored", (sh.reset(0.1), sh.update(NAN, 0.0, dt) == 0.1));
  }

  // ---- host control loop: impedance / torque law, velocity braking ----
  {
    // spring pulls back toward the setpoint: 2 Nm/turn, 0.05 r away -> 0.1 Nm toward it
    check("impedance: spring sign", std::fabs(impedance_torque(0.05, 0, 0, 0, 0, 2.0, 0, 1.0) + 0.1) < 1e-12);
    check("impedance: spring around setpoint", std::fabs(impedance_torque(0.05, 0, 0.1, 0, 0, 2.0, 0, 1.0) - 0.1) < 1e-12);
    check("impedance: damper opposes velocity", impedance_torque(0, 0.2, 0, 0, 0, 0, 0.5, 1.0) < 0);
    check("impedance: damper tracks setpoint velocity", impedance_torque(0, 0.2, 0, 0.2, 0, 0, 0.5, 1.0) == 0.0);
    check("impedance: feed-forward added", std::fabs(impedance_torque(0, 0, 0, 0, 0.05, 1, 1, 1.0) - 0.05) < 1e-12);
    check("impedance: clamp +", impedance_torque(-1.0, 0, 0, 0, 0, 10.0, 0, 0.3) == 0.3);
    check("impedance: clamp -", impedance_torque(1.0, 0, 0, 0, 0, 10.0, 0, 0.3) == -0.3);
    check("impedance: negative limit treated as magnitude", impedance_torque(1.0, 0, 0, 0, 0, 10.0, 0, -0.3) == -0.3);
    check("impedance: zero gains -> feed-forward only", impedance_torque(0.3, 1.0, 0, 0, 0, 0, 0, 0.3) == 0.0);
    check("impedance: NaN state -> zero torque", impedance_torque(NAN, 0, 0, 0, 0, 1.0, 0.1, 0.3) == 0.0);
    // braking: acc 1 r/s^2, 0.02 r before the upper limit -> |v| <= sqrt(0.04) = 0.2 r/s
    check("brake: far from limit unchanged", brake_limited_velocity(0.1, 0.0, -0.25, 0.25, 1.0) == 0.1);
    check("brake: near upper limit reduced", std::fabs(brake_limited_velocity(0.5, 0.23, -0.25, 0.25, 1.0) - 0.2) < 1e-9);
    check("brake: moving away from the near limit unchanged", brake_limited_velocity(-0.5, 0.23, -0.25, 0.25, 1.0) == -0.5);
    check("brake: near lower limit reduced", std::fabs(brake_limited_velocity(-0.5, -0.23, -0.25, 0.25, 1.0) + 0.2) < 1e-9);
    check("brake: at / beyond the limit -> 0", brake_limited_velocity(0.1, 0.26, -0.25, 0.25, 1.0) == 0.0);
    check("brake: beyond the limit, moving back allowed", brake_limited_velocity(-0.1, 0.26, -0.25, 0.25, 1.0) == -0.1);
    check("brake: NaN -> 0", brake_limited_velocity(NAN, 0.0, -0.25, 0.25, 1.0) == 0.0);
  }

  std::printf("%d passed, %d failed\n", g_passed, g_failed);
  return g_failed == 0 ? 0 : 1;
}
