// Find out which part of a command frame costs round-trip time.
//
// Runs several frame variants back to back, each at a fixed rate (default 1 kHz) for a few
// seconds, and prints the round-trip latency (send -> reply received) of each.  Two neighbouring
// variants differ in one thing only.  The motor never gets more than "velocity 0" (mode 10: powered,
// holds still) or "current 0 A" (mode 9: no torque), and is stopped at the end.
//
//   ./latency_probe --id 2 [--type float|int16] [--seconds 5] [--rate 1000] [--rt]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <limits>

#include "cli_common.hpp"
#include "hightorque/motor.hpp"

using namespace hightorque;

namespace {

struct Variant {
  const char* name;
  int enter_mode;  // 0: stop, 10: mode 10 with vel 0, 9: mode 9 with 0 A (no torque) before this variant
  std::function<FrameBuilder()> frame;
};

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  cli::install_ctrl_c();
  const uint8_t id = static_cast<uint8_t>(args.num("id", 1));
  const double seconds = args.num("seconds", 5.0);
  const double rate_hz = args.num("rate", 1000.0);
  const double kNaN = std::numeric_limits<double>::quiet_NaN();

  try {
    Fdcanusb bus(args.str("port"));
    cli::configure_bus(bus, args);
    Motor::Options opt;
    opt.type = cli::parse_type(args.str("type"));
    opt.model = cli::parse_model(args.str("model"));
    Motor motor(bus, id, opt);
    const DataType t = opt.type;

    auto first = motor.stop();
    if (!first) {
      cli::explain_no_reply(bus, id);
      return 1;
    }
    if (args.has("rt")) cli::make_realtime(rate_hz);

    // Size of the "current" command frame, so the padded query can match it.
    FrameBuilder full;
    full.set_mode(mode::kPosition).write(t, reg::kPositionCmd, {kNaN, 0.0}, {Quantity::Position, Quantity::Velocity});
    full.query_state(t);
    FrameBuilder q_only;
    q_only.query_state(t);
    const size_t pad = full.unpadded().size() > q_only.unpadded().size()
                           ? full.unpadded().size() - q_only.unpadded().size()
                           : 0;

    const std::vector<Variant> variants = {
        {"stop:   query only", 0, [&] { FrameBuilder f; f.query_state(t); return f; }},
        {"stop:   NOP padding + query (same size as command)", 0,
         [&] { FrameBuilder f; f.append(std::vector<uint8_t>(pad, kPadding)); f.query_state(t); return f; }},
        {"stop:   write mode 0 + query", 0, [&] { FrameBuilder f; f.set_mode(mode::kStop); f.query_state(t); return f; }},
        {"mode10: query only", 10, [&] { FrameBuilder f; f.query_state(t); return f; }},
        {"mode10: write vel 0 (0x021) + query", 10,
         [&] { FrameBuilder f; f.write(t, reg::kVelocityCmd, Quantity::Velocity, {0.0}); f.query_state(t); return f; }},
        {"mode10: write mode 10 + query", 10,
         [&] { FrameBuilder f; f.set_mode(mode::kPosition); f.query_state(t); return f; }},
        {"mode10: mode 10 + pos NaN + vel 0 + query (current)", 10,
         [&] {
           FrameBuilder f;
           f.set_mode(mode::kPosition).write(t, reg::kPositionCmd, {kNaN, 0.0}, {Quantity::Position, Quantity::Velocity});
           f.query_state(t);
           return f;
         }},
        {"mode9:  query only", 9, [&] { FrameBuilder f; f.query_state(t); return f; }},
        {"mode9:  mode 9 + current 0 A + query (case 1 frame)", 9, [&] { return motor.command_dq_current(0.0); }},
    };

    std::printf("%.0f Hz, %s, %.0f s per variant, motor ID %u\n", rate_hz, args.str("type", "float").c_str(), seconds, id);
    std::printf("%-54s %5s %7s %6s %6s %6s %6s %6s\n", "variant", "bytes", "cycles", ">1ms%", "p50", "p90", "p99",
                "max");
    const double period_us = 1e6 / rate_hz;

    for (const auto& v : variants) {
      if (cli::g_stop) break;
      if (v.enter_mode == 10) {
        if (!motor.set_vel(0.0)) std::cerr << "warning: no reply to set_vel(0)\n";
      } else if (v.enter_mode == 9) {
        motor.stop();
        if (!motor.set_dq_current(0.0)) std::cerr << "warning: no reply to set_dq_current(0)\n";
      } else {
        motor.stop();
      }
      const FrameBuilder fb = v.frame();
      std::vector<double> lats;
      uint64_t missed = 0, over = 0;
      cli::Rate rate(rate_hz);
      const auto t_begin = std::chrono::steady_clock::now();
      while (!cli::g_stop && std::chrono::steady_clock::now() - t_begin < std::chrono::duration<double>(seconds)) {
        const auto t0 = std::chrono::steady_clock::now();
        const auto states = cycle(bus, {{&motor, fb}});
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        if (!states[0]) missed++;
        if (us > period_us) over++;
        lats.push_back(us);
        rate.sleep();
      }
      std::sort(lats.begin(), lats.end());
      auto pct = [&](double p) { return lats[std::min(lats.size() - 1, static_cast<size_t>(p * lats.size()))]; };
      std::printf("%-54s %5zu %7zu %5.1f%% %6.0f %6.0f %6.0f %6.0f", v.name, round_up_dlc(fb.unpadded().size()),
                  lats.size(), 100.0 * over / lats.size(), pct(0.5), pct(0.9), pct(0.99), lats.back());
      if (missed) std::printf("   missed %llu", static_cast<unsigned long long>(missed));
      std::printf("\n");
      std::fflush(stdout);
    }
    auto s = motor.stop();
    std::cout << "stopped" << (s ? ": " + format_state(*s) : std::string(" (no reply)")) << "\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
