// C++ / host-PC version of the vendor test_motor.c (test_motor_control).
//
//   ./test_motor <mode> [options]
//
// <mode> is the same "case" number as in the vendor example:
//    0  DQ voltage          motor_set_dq_vlot          --volt
//    1  DQ current          motor_set_dq_current       --cur
//    2  position            motor_set_pos              --pos        (max speed, violent!)
//    3  velocity            motor_set_vel              --vel
//    4  torque              motor_set_tqe              --tqe
//    5  position + velocity motor_set_pos_vel          --pos --vel
//    6  pos + vel + max tqe motor_set_pos_vel_MAXtqe   --pos --vel --tqe
//    7  trapezoid           motor_set_pos_velmax_acc   --pos --vel --acc
//    8  PD control (运控2)  motor_set_pos_vel_tqe_kp_kd_2  --pos --vel --tqe --kp --kd
//    9  brake               motor_set_brake
//   10  stop                motor_set_stop
//   11  query state         motor_get_state_send
//   12  firmware version    motor_get_version
//
// Options:
//   --id N          motor ID (default 1)
//   --type T        float | int32 | int16 (default float, like the vendor test)
//   --model M       motor model for torque correction (default M5047_36 = HTDW-5047-36-NE)
//   --unit U        turns | rad | deg (default turns)
//   --rate HZ       command rate (default 100; the vendor STM32 loop runs at 10 Hz)
//   --rt            real-time thread priority (needed for a clean 1 kHz loop on macOS)
//   --duration S    stop after S seconds (default: run until Ctrl-C)
//   --port PATH     adapter serial port (default: auto-detect /dev/cu.usbmodem*)
//   --full          also read bus voltage and temperature
//   --no-brs        data phase at 1 Mbps instead of 5 Mbps
//   --adapter-timing  keep the adapter's own sample points (by default every program sets the
//                     motors' 0.75 / 0.75 in adapter RAM, see Fdcanusb::set_sample_points)
//   --vendor-frames  do not reset leftover mode-10 registers (byte-identical vendor frames)
//   --debug         print every line exchanged with the adapter
//
// Ctrl-C always sends "stop" before exiting.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "cli_common.hpp"
#include "hightorque/motor.hpp"

using namespace hightorque;

namespace {

void usage() {
  std::cerr << "usage: test_motor <mode 0-12> [--id 1] [--type float|int32|int16] [--model M5047_36]\n"
               "                  [--rate 100] [--rt] [--duration S] [--port /dev/cu.usbmodemXXX] [--full] [--no-brs] [--debug]\n"
               "                  [--pos P] [--vel V] [--tqe T] [--acc A] [--kp KP] [--kd KD] [--volt U] [--cur I]\n"
               "see the comment at the top of examples/test_motor.cpp for the mode list\n";
}

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  if (args.positional().empty() || args.has("help")) {
    usage();
    return 2;
  }
  const int mode = std::atoi(args.positional()[0].c_str());
  if (mode < 0 || mode > 12) {
    usage();
    return 2;
  }

  // Command parameters.  Defaults are the vendor test values, except where noted (safer).
  const double pos = args.num("pos", mode == 8 ? 0.5 : (mode == 6 ? 0.2 : 0.1));
  const double vel = args.num("vel", mode == 7 ? 0.5 : (mode == 8 ? 0.0 : 0.1));
  const double tqe = args.num("tqe", mode == 6 ? NAN : (mode == 4 ? 0.1 : 0.0));  // vendor: 0.5 Nm (case 4), 1 Nm (case 8)
  const double acc = args.num("acc", 0.1);
  const double kp = args.num("kp", 1.0);
  const double kd = args.num("kd", 0.1);   // vendor: 1
  const double volt = args.num("volt", 0.5);  // vendor: 1.5 V
  const double cur = args.num("cur", 0.2);

  const double rate_hz = args.num("rate", 100.0);
  const double duration = args.num("duration", 0.0);
  const int id = static_cast<int>(args.num("id", 1));

  cli::install_ctrl_c();

  try {
    Fdcanusb bus(args.str("port"));
    cli::configure_bus(bus, args);

    Motor::Options opt;
    opt.type = cli::parse_type(args.str("type"));
    opt.model = cli::parse_model(args.str("model"));
    opt.unit = cli::parse_unit(args.str("unit"));
    opt.query_full = args.has("full");
    opt.clear_sticky_registers = !args.has("vendor-frames");  // README "register 残留"
    Motor motor(bus, static_cast<uint8_t>(id), opt);

    std::cout << "adapter: " << bus.path() << ", motor ID " << id << ", model " << motor_model_name(opt.model)
              << " (k = " << torque_factor(opt.model) << ")\n";

    if (mode == 12) {
      auto v = motor.get_version();
      if (!v) {
        cli::explain_no_reply(bus, id);
        return 1;
      }
      std::printf("ID: %d, firmware version = %d.%d.%d%s\n", id, v->major, v->minor, v->patch,
                  v->reported ? "" : " (not reported by the motor: old firmware, vendor default value)");
      return 0;
    }

    // Like moteus set_stop(): leave any fault / timeout mode first.
    auto first = motor.stop();
    if (!first) {
      cli::explain_no_reply(bus, id);
      return 1;
    }
    std::cout << "initial: " << format_state(*first, opt.unit) << "\n";

    cli::Rate rate(rate_hz);
    if (args.has("rt")) cli::make_realtime(rate_hz);
    const auto t0 = std::chrono::steady_clock::now();
    auto last_print = t0;
    uint64_t sent = 0, missed = 0, over = 0;
    const double period_us = 1e6 / rate_hz;
    std::vector<double> lats;  // round-trip time of every command (send + reply), us
    lats.reserve(static_cast<size_t>(rate_hz * (duration > 0 ? duration + 1 : 60)));

    while (!cli::g_stop) {
      std::optional<MotorState> s;
      const auto t_cmd = std::chrono::steady_clock::now();
      switch (mode) {
        case 0: s = motor.set_dq_volt(volt); break;
        case 1: s = motor.set_dq_current(cur); break;
        case 2: s = motor.set_pos(pos); break;
        case 3: s = motor.set_vel(vel); break;
        case 4: s = motor.set_tqe(tqe); break;
        case 5: s = motor.set_pos_vel(pos, vel); break;
        case 6: s = motor.set_pos_vel_max_tqe(pos, vel, tqe); break;
        case 7: s = motor.set_pos_velmax_acc(pos, vel, acc); break;
        case 8: s = motor.set_pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd); break;
        case 9: s = motor.brake(); break;
        case 10: s = motor.stop(); break;
        case 11: s = motor.query(); break;
      }
      sent++;
      if (!s) missed++;
      const double lat_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t_cmd).count();
      lats.push_back(lat_us);
      if (lat_us > period_us) over++;

      const auto now = std::chrono::steady_clock::now();
      if (now - last_print >= std::chrono::seconds(1)) {
        last_print = now;
        if (s) {
          std::cout << format_state(*s, opt.unit);
        } else {
          std::cout << "no reply";
        }
        std::cout << "   [sent " << sent << ", missed " << missed << ", >" << period_us << "us " << over << "]\n";
      }
      if (duration > 0 && now - t0 >= std::chrono::duration<double>(duration)) break;
      rate.sleep();
    }

    if (!lats.empty()) {
      std::sort(lats.begin(), lats.end());
      auto pct = [&](double p) { return lats[std::min(lats.size() - 1, static_cast<size_t>(p * lats.size()))]; };
      std::printf("loop: %zu cycles at %.0f Hz, missed %llu, %llu over %.0f us (%.3f%%), round trip p50 %.0f / p99 %.0f / p99.9 %.0f / max %.0f us\n",
                  lats.size(), rate_hz, static_cast<unsigned long long>(missed), static_cast<unsigned long long>(over),
                  period_us, 100.0 * over / lats.size(), pct(0.5), pct(0.99), pct(0.999), lats.back());
    }
    auto s = motor.stop();
    std::cout << "stopped" << (s ? ": " + format_state(*s, opt.unit) : std::string(" (no reply)")) << "\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
