// Measure the real control rate (command sent + reply received per second).
// C++ version of rate_test.py.  The motor holds velocity 0 (it does not move).
//
//   ./rate_test [--ids 1,2] [--type float|int32|int16] [--many] [--query] [--seconds 10]
//
//   --ids 1,2   several motors on the bus: frames for all motors are written back to back
//               and all replies are collected (pipelined), one "cycle" per loop
//   --many      use the 一拖多 mode instead: ONE frame for all motors (int16 only)
//   --sequential  with several --ids: one motor at a time (send, wait, next), replies never overlap
//   --pad N     with --query: put N NOP bytes (0x50) before the query, to make frames longer
//               (stress test for the 5 Mbps data phase; 59 => 64-byte frames)
//   --query     only query the state, do not command anything
//   --rate HZ   run at a fixed rate instead of as fast as possible (e.g. --rate 1000)
//   --hist      print a latency histogram (100 us bins) at the end
//   --rt        real-time thread priority (macOS time-constraint policy / Linux SCHED_FIFO)
//   --trace FILE every frame sent / received with a timestamp (tools/check_link.py --trace FILE)
//   --timeout-us N  wait at most N us for the replies of one cycle (default 20000).  With a lost
//               reply the cycle takes this long, so use about one period for a fixed --rate.
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <sstream>

#include "cli_common.hpp"
#include "hightorque/motor.hpp"

using namespace hightorque;

int main(int argc, char** argv) {
  cli::Args args(argc, argv);
  cli::install_ctrl_c();

  std::vector<uint8_t> ids;
  {
    std::stringstream ss(args.str("ids", "1"));
    std::string item;
    while (std::getline(ss, item, ',')) ids.push_back(static_cast<uint8_t>(std::atoi(item.c_str())));
  }
  const bool use_many = args.has("many");
  const bool query_only = args.has("query");
  const double seconds = args.num("seconds", 0.0);
  const double fixed_rate = args.num("rate", 0.0);

  try {
    Fdcanusb bus(args.str("port"));
    cli::configure_bus(bus, args);

    Motor::Options opt;
    opt.type = cli::parse_type(args.str("type"));
    opt.model = cli::parse_model(args.str("model"));
    if (args.has("timeout-us")) opt.timeout = std::chrono::microseconds(static_cast<int64_t>(args.num("timeout-us", 20000)));
    std::vector<Motor> motors;
    for (uint8_t id : ids) motors.emplace_back(bus, id, opt);

    for (auto& m : motors) {
      // long timeout here: --timeout-us is for the test loop only
      auto st = cycle(bus, {{&m, m.command_mode(mode::kStop)}}, std::chrono::milliseconds(20));
      auto s = st[0];
      if (!s) {
        cli::explain_no_reply(bus, m.id());
        return 1;
      }
      std::cout << format_state(*s) << "\n";
    }

    const int max_id = *std::max_element(ids.begin(), ids.end());
    MotorMany many(bus, max_id, opt.model);

    std::cout << "running " << (use_many ? "one-to-many" : "normal") << " mode, " << ids.size() << " motor(s), "
              << (use_many ? "int16" : args.str("type", "float")) << (fixed_rate > 0 ? "" : ", as fast as possible")
              << ". Ctrl-C to stop.\n";

    const size_t pad = static_cast<size_t>(args.num("pad", 0));
    auto padded_query = [&](Motor& m) {
      if (pad == 0) return m.command_query();
      FrameBuilder f;
      f.append(std::vector<uint8_t>(pad, kPadding));
      f.query_state(opt.type);
      return f;
    };
    if (pad > 0) {
      FrameBuilder f;
      f.append(std::vector<uint8_t>(pad, kPadding));
      f.query_state(opt.type);
      std::cout << "padded query frame: " << round_up_dlc(f.unpadded().size()) << " bytes\n";
    }

    using Clock = std::chrono::steady_clock;
    std::optional<cli::Rate> rate;
    if (fixed_rate > 0) rate.emplace(fixed_rate);
    if (args.has("rt")) cli::make_realtime(fixed_rate > 0 ? fixed_rate : 1000.0);

    std::FILE* trace = nullptr;
    if (args.has("trace")) {
      trace = std::fopen(args.str("trace").c_str(), "w");
      if (!trace) throw std::runtime_error("cannot write --trace file");
      std::setvbuf(trace, nullptr, _IOFBF, 1 << 20);
      std::fprintf(trace, "# rate_test trace: rate_hz=%.1f\n", fixed_rate > 0 ? fixed_rate : 0.0);
      bus.set_trace(trace);
    }
    std::vector<uint64_t> req0, rep0;
    for (uint8_t id : ids) {
      req0.push_back(bus.requests_sent(id));
      rep0.push_back(bus.replies_received(id));
    }
    const auto t_begin = Clock::now();
    auto t_window = t_begin;
    uint64_t count = 0, missed = 0;
    double lat_sum = 0, lat_max = 0, lat_min = 1e9;
    // Deadline: one period at a fixed rate, else 1 ms.  "over" counts cycles slower than that.
    const double deadline_us = fixed_rate > 0 ? 1e6 / fixed_rate : 1000.0;
    uint64_t over = 0, total = 0, total_over = 0;
    std::vector<double> lats, all_lats;
    std::vector<uint64_t> per_motor_missed(ids.size(), 0);

    while (!cli::g_stop) {
      const auto t0 = Clock::now();
      bool ok = true;
      if (use_many) {
        for (uint8_t id : ids) many.vel(id, 0.0);
        auto states = many.send(MotorMany::Request::ModeFaultPosVelTqe);
        for (uint8_t id : ids) ok = ok && states[id - 1].has_value();
      } else {
        if (args.has("sequential")) {
          // One motor at a time: send, wait for its reply, then the next one (replies never overlap).
          for (size_t i = 0; i < motors.size(); i++) {
            auto& m = motors[i];
            const auto st = cycle(bus, {{&m, query_only ? padded_query(m) : m.command_vel(0.0)}});
            if (!st[0]) { ok = false; per_motor_missed[i]++; }
          }
        } else {
          std::vector<std::pair<Motor*, FrameBuilder>> cmds;
          for (auto& m : motors) cmds.emplace_back(&m, query_only ? padded_query(m) : m.command_vel(0.0));
          const auto st = cycle(bus, cmds);
          for (size_t i = 0; i < st.size(); i++) {
            if (!st[i]) { ok = false; per_motor_missed[i]++; }
          }
        }
      }
      const double lat_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
      count++;
      if (!ok) missed++;
      lat_sum += lat_us;
      lat_max = std::max(lat_max, lat_us);
      lat_min = std::min(lat_min, lat_us);
      lats.push_back(lat_us);
      if (lat_us > deadline_us) over++;

      const auto now = Clock::now();
      const double window = std::chrono::duration<double>(now - t_window).count();
      if (window >= 1.0) {
        std::sort(lats.begin(), lats.end());
        auto pct = [&](double p) { return lats[std::min(lats.size() - 1, static_cast<size_t>(p * lats.size()))]; };
        std::printf("%6.0f Hz   latency min/avg/p99/p99.9/max %5.0f / %5.0f / %5.0f / %5.0f / %5.0f us   >%.0fus %llu   missed %llu\n",
                    count / window, lat_min, lat_sum / count, pct(0.99), pct(0.999), lat_max, deadline_us,
                    static_cast<unsigned long long>(over), static_cast<unsigned long long>(missed));
        std::fflush(stdout);
        t_window = now;
        total += count;
        total_over += over;
        all_lats.insert(all_lats.end(), lats.begin(), lats.end());
        lats.clear();
        count = missed = over = 0;
        lat_sum = lat_max = 0;
        lat_min = 1e9;
      }
      if (seconds > 0 && now - t_begin >= std::chrono::duration<double>(seconds)) break;
      if (rate) rate->sleep();
    }

    if (total > 0) {
      std::sort(all_lats.begin(), all_lats.end());
      auto pct = [&](double p) { return all_lats[std::min(all_lats.size() - 1, static_cast<size_t>(p * all_lats.size()))]; };
      std::printf("total: %llu cycles, %llu over %.0f us (%.3f%%), p50 %.0f / p99 %.0f / p99.9 %.0f / max %.0f us\n",
                  static_cast<unsigned long long>(total), static_cast<unsigned long long>(total_over), deadline_us,
                  100.0 * total_over / total, pct(0.5), pct(0.99), pct(0.999), all_lats.back());
    }
    if (ids.size() > 1 && !use_many) {
      std::printf("missed per motor:");
      for (size_t i = 0; i < ids.size(); i++) std::printf("  ID %u: %llu", ids[i], static_cast<unsigned long long>(per_motor_missed[i]));
      std::printf("\n");
    }
    if (args.has("hist") && !all_lats.empty()) {
      // all_lats is sorted.  100 us bins; everything >= 3 ms goes into the last bin.
      std::vector<uint64_t> bins(31, 0);
      for (double v : all_lats) bins[std::min<size_t>(30, static_cast<size_t>(v / 100.0))]++;
      const uint64_t peak = *std::max_element(bins.begin(), bins.end());
      std::printf("latency histogram (100 us bins):\n");
      for (size_t i = 0; i < bins.size(); i++) {
        if (bins[i] == 0) continue;
        const int bar = static_cast<int>(50.0 * bins[i] / peak + 0.5);
        if (i < 30) std::printf("  %4zu-%4zu us %7llu  %s\n", i * 100, i * 100 + 100, static_cast<unsigned long long>(bins[i]), std::string(std::max(bar, 1), '#').c_str());
        else std::printf("     >=3000 us %7llu  %s\n", static_cast<unsigned long long>(bins[i]), std::string(std::max(bar, 1), '#').c_str());
      }
    }
    for (auto& m : motors) m.stop();
    bus.drain(std::chrono::milliseconds(100));
    for (size_t i = 0; i < ids.size(); i++) {
      const long long rq = static_cast<long long>(bus.requests_sent(ids[i]) - req0[i]);
      const long long rp = static_cast<long long>(bus.replies_received(ids[i]) - rep0[i]);
      std::printf("ID %u: %lld requests, %lld replies (+100 ms wait) -> %lld never arrived, late (dropped) %llu\n", ids[i],
                  rq, rp, rq - rp, static_cast<unsigned long long>(bus.late_replies(ids[i])));
    }
    if (trace) {
      bus.set_trace(nullptr);
      std::fclose(trace);
    }
    std::string can_status;
    bus.bus_off(&can_status);
    std::printf("adapter can status: %s\n", can_status.c_str());
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
