// C++ / host-PC version of the vendor test_motor_many.c (一拖多模式, one frame drives all motors).
//
//   ./test_motor_many <mode> --num 2 [options]
//
// <mode> is the same "case" number as in the vendor example (all motors get the same command):
//    0  DQ voltage            motor_many_dq_volt              --volt
//    1  DQ current            motor_many_dq_current           --cur
//    2  position              motor_many_pos                  --pos   (max speed, violent!)
//    3  velocity              motor_many_vel                  --vel
//    4  torque                motor_many_tqe                  --tqe
//    5  position + velocity   motor_many_pos_vel              --pos --vel
//    6  pos + vel + max tqe   motor_many_pos_vel_MAXtqe       --pos --vel --tqe
//    7  pos + vel + acc       motor_many_pos_vel_acc          --pos --vel --acc
//    8  PD control (old)      motor_many_pos_vel_tqe_kp_kd    --pos --vel --tqe --kp --kd
//    9  PD control 2          motor_many_pos_vel_tqe_kp_kd_2  --pos --vel --tqe --kp --kd
//
// Options: --num N (motors with IDs 1..N, default 2), --model, --rate HZ (default 100),
//          --duration S, --port PATH, --request mode|temp|basic (what the motors report back), --debug
//
// Motor IDs must be 1..N.  The int16 position range is only +-3.27 turns.
#include <cmath>
#include <iostream>

#include "cli_common.hpp"
#include "hightorque/motor.hpp"

using namespace hightorque;

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  if (args.positional().empty()) {
    std::cerr << "usage: test_motor_many <mode 0-9> [--num 2] [--rate 100] [--duration S] [--port PATH]\n"
                 "see the comment at the top of examples/test_motor_many.cpp for the mode list\n";
    return 2;
  }
  const int mode = std::atoi(args.positional()[0].c_str());
  const int num = static_cast<int>(args.num("num", 2));

  // Vendor test defaults, except torque (vendor 1 Nm) and voltage (vendor 1 V), lowered for safety.
  const double pos = args.num("pos", mode == 6 ? 0.0 : 1.0);
  const double vel = args.num("vel", 0.1);
  const double tqe = args.num("tqe", mode == 6 ? NAN : (mode == 4 ? 0.1 : 0.0));
  const double acc = args.num("acc", 0.2);
  const double kp = args.num("kp", 1.0);
  const double kd = args.num("kd", 1.0);
  const double volt = args.num("volt", 0.5);
  const double cur = args.num("cur", 0.2);

  auto request = MotorMany::Request::TempFaultPosVelTqe;
  if (args.str("request") == "mode") request = MotorMany::Request::ModeFaultPosVelTqe;
  if (args.str("request") == "basic") request = MotorMany::Request::PosVelTqe;

  cli::install_ctrl_c();
  try {
    Fdcanusb bus(args.str("port"));
    cli::configure_bus(bus, args);
    const MotorModel model = cli::parse_model(args.str("model"));
    MotorMany many(bus, num, model);

    // Clear faults first with a normal stop command to every motor.
    for (int id = 1; id <= num; id++) {
      Motor m(bus, static_cast<uint8_t>(id), Motor::Options{model});
      auto s = m.stop();
      std::cout << (s ? format_state(*s) : "motor " + std::to_string(id) + ": no reply") << "\n";
    }

    cli::Rate rate(args.num("rate", 100.0));
    const double duration = args.num("duration", 0.0);
    const auto t0 = std::chrono::steady_clock::now();
    auto last_print = t0;

    while (!cli::g_stop) {
      for (int i = 1; i <= num; i++) {
        const uint8_t id = static_cast<uint8_t>(i);
        switch (mode) {
          case 0: many.dq_volt(id, volt); break;
          case 1: many.dq_current(id, cur); break;
          case 2: many.pos(id, pos); break;
          case 3: many.vel(id, vel); break;
          case 4: many.tqe(id, tqe); break;
          case 5: many.pos_vel(id, pos, vel); break;
          case 6: many.pos_vel_max_tqe(id, pos, vel, tqe); break;
          case 7: many.pos_vel_acc(id, pos, vel, acc); break;
          case 8: many.pos_vel_tqe_kp_kd(id, pos, vel, tqe, kp, kd); break;
          case 9: many.pos_vel_tqe_kp_kd_2(id, pos, vel, tqe, kp, kd); break;
          default: std::cerr << "mode must be 0-9\n"; return 2;
        }
      }
      auto states = many.send(request);

      const auto now = std::chrono::steady_clock::now();
      if (now - last_print >= std::chrono::seconds(1)) {
        last_print = now;
        for (int i = 0; i < num; i++) {
          std::cout << (states[i] ? format_state(*states[i]) : "ID: " + std::to_string(i + 1) + " no reply") << "\n";
        }
        std::cout << "\n";
      }
      if (duration > 0 && now - t0 >= std::chrono::duration<double>(duration)) break;
      rate.sleep();
    }

    for (int id = 1; id <= num; id++) Motor(bus, static_cast<uint8_t>(id)).stop();
    std::cout << "stopped\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
