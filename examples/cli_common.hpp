// Small helpers shared by the example programs: argument parsing, Ctrl-C handling, loop timing.
#pragma once

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#endif

#include "hightorque/fdcanusb.hpp"
#include "hightorque/protocol.hpp"

namespace cli {

inline std::atomic<bool> g_stop{false};

inline void install_ctrl_c() {
  std::signal(SIGINT, [](int) { g_stop = true; });
  std::signal(SIGTERM, [](int) { g_stop = true; });
  std::signal(SIGHUP, [](int) { g_stop = true; });  // terminal closed
  // If stdout is a pipe that closes early (e.g. `| head`), the default SIGPIPE kills the program
  // before it can send "stop", and the motor keeps running.  Ignore it: printing then just fails,
  // and the loop still finishes and stops the motor.
  std::signal(SIGPIPE, SIG_IGN);
}

// "--key value" and "--flag" arguments, plus positional arguments.
class Args {
 public:
  Args(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
      std::string a = argv[i];
      if (a.rfind("--", 0) == 0) {
        std::string key = a.substr(2);
        if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
          kv_[key] = argv[++i];
        } else {
          kv_[key] = "";
        }
      } else {
        positional_.push_back(a);
      }
    }
  }

  bool has(const std::string& k) const { return kv_.count(k) > 0; }
  std::string str(const std::string& k, const std::string& def = "") const {
    auto it = kv_.find(k);
    return it == kv_.end() ? def : it->second;
  }
  double num(const std::string& k, double def) const {
    auto it = kv_.find(k);
    return it == kv_.end() || it->second.empty() ? def : std::atof(it->second.c_str());
  }
  const std::vector<std::string>& positional() const { return positional_; }

 private:
  std::map<std::string, std::string> kv_;
  std::vector<std::string> positional_;
};

inline hightorque::DataType parse_type(const std::string& s) {
  if (s == "int16") return hightorque::DataType::Int16;
  if (s == "int32") return hightorque::DataType::Int32;
  if (s == "float" || s.empty()) return hightorque::DataType::Float;
  std::cerr << "unknown --type " << s << " (use float, int32 or int16)\n";
  std::exit(2);
}

inline hightorque::MotorModel parse_model(const std::string& s) {
  if (s.empty()) return hightorque::MotorModel::M5047_36;
  auto m = hightorque::parse_motor_model(s);
  if (!m) {
    std::cerr << "unknown --model " << s << " (e.g. M4438_30, M5047_36, NONE)\n";
    std::exit(2);
  }
  return *m;
}

inline hightorque::AngleUnit parse_unit(const std::string& s) {
  if (s == "rad") return hightorque::AngleUnit::Radians;
  if (s == "deg") return hightorque::AngleUnit::Degrees;
  return hightorque::AngleUnit::Turns;
}

// Apply --debug / --no-brs, and restart the adapter's CAN controller if an earlier run left it
// in bus-off (the adapter ships with can.autorecover = 0, so it would stay dead otherwise).
inline void configure_bus(hightorque::Fdcanusb& bus, const Args& args) {
  bus.set_debug(args.has("debug"));
  bus.set_disable_brs(args.has("no-brs"));
  if (args.has("sample-points")) {
    // "--sample-points 0.75,0.75": nominal, data.  For experiments; RAM only like the default.
    const std::string v = args.str("sample-points");
    const auto comma = v.find(',');
    if (comma == std::string::npos) throw std::runtime_error("--sample-points expects NOMINAL,DATA, e.g. 0.75,0.75");
    bus.set_sample_points(std::stod(v.substr(0, comma)), std::stod(v.substr(comma + 1)));
    return;
  }
  if (!args.has("adapter-timing")) {
    // Match the motor's sample points (RAM only); this also restarts CAN, which clears bus-off.
    bus.set_sample_points();
    return;
  }
  std::string status;
  if (bus.bus_off(&status)) {
    std::cerr << "adapter was in CAN bus-off (" << status << "), restarting its CAN controller\n";
    bus.restart_can();
  }
}

// Explain a missing reply.  Checks the adapter's CAN state first, because bus-off is the most
// common cause and looks exactly like "motor not answering".
inline void explain_no_reply(hightorque::Fdcanusb& bus, int id) {
  std::string status;
  if (bus.bus_off(&status)) {
    std::cerr << "No reply from motor " << id << ": the adapter went CAN bus-off (" << status << ").\n"
              << "  dlec != 0/7 means errors in the 5 Mbps data phase. Usually the bus has only one 120 ohm\n"
              << "  terminator (should read ~60 ohm between CAN_H and CAN_L with power off).\n"
              << "  Fix the termination, or run with --no-brs (data phase at 1 Mbps) until then.\n";
    bus.restart_can();
    return;
  }
  std::cerr << "No reply from motor " << id << " via " << bus.path() << ". Check:\n"
            << "  * motor power supply is on (24 V) and the motor LED is lit\n"
            << "  * the motor ID (--id); `motor_tool scan` lists the motors that answer\n"
            << "  * CAN_H / CAN_L wiring and termination (~60 ohm between H and L with power off)\n"
            << "  * run with --debug to see what the adapter sends back\n";
}

// Ask the OS to treat the calling thread as a real-time thread (--rt).
//   macOS: THREAD_TIME_CONSTRAINT_POLICY (the policy CoreAudio uses), no root needed.
//   Linux: SCHED_FIFO priority 80 + mlockall (needs root or CAP_SYS_NICE / rtprio limit).
// Returns false (with a message) if the OS refused.
inline bool make_realtime(double hz) {
#if defined(__APPLE__)
  mach_timebase_info_data_t tb;
  mach_timebase_info(&tb);
  const double ticks_per_ns = static_cast<double>(tb.denom) / tb.numer;
  const double period_ns = 1e9 / hz;
  thread_time_constraint_policy_data_t pol;
  pol.period = static_cast<uint32_t>(period_ns * ticks_per_ns);
  pol.computation = static_cast<uint32_t>(0.5 * period_ns * ticks_per_ns);  // CPU time we need per period
  pol.constraint = static_cast<uint32_t>(period_ns * ticks_per_ns);         // must be done within one period
  pol.preemptible = 1;
  const kern_return_t kr = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
                                             reinterpret_cast<thread_policy_t>(&pol),
                                             THREAD_TIME_CONSTRAINT_POLICY_COUNT);
  if (kr != KERN_SUCCESS) {
    std::cerr << "warning: thread_policy_set failed (" << kr << ")\n";
    return false;
  }
  std::cerr << "real-time: macOS time-constraint policy, period " << period_ns / 1000 << " us\n";
  return true;
#elif defined(__linux__)
  (void)hz;
  if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) std::cerr << "warning: mlockall failed\n";
  sched_param sp{};
  sp.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    std::cerr << "warning: SCHED_FIFO failed (run as root or set rtprio in /etc/security/limits.conf)\n";
    return false;
  }
  std::cerr << "real-time: SCHED_FIFO priority 80\n";
  return true;
#else
  (void)hz;
  std::cerr << "warning: --rt not supported on this OS\n";
  return false;
#endif
}

// Fixed-rate loop: sleep until the next period, then busy-wait the last 200 us for low jitter.
class Rate {
 public:
  explicit Rate(double hz)
      : period_(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz))), next_(Clock::now() + period_) {}

  void sleep() {
    const auto spin = std::chrono::microseconds(200);
    if (next_ - Clock::now() > spin) std::this_thread::sleep_until(next_ - spin);
    while (Clock::now() < next_) {
    }
    next_ += period_;
    // If we fell far behind (e.g. the process was paused), do not try to catch up.
    if (Clock::now() > next_ + 10 * period_) next_ = Clock::now() + period_;
  }

 private:
  using Clock = std::chrono::steady_clock;
  std::chrono::nanoseconds period_;
  Clock::time_point next_;
};

}  // namespace cli
