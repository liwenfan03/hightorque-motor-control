// Transport for the mjbots mjcanfd-usb-1x / fdcanusb adapter on macOS / Linux.
//
// The adapter shows up as a USB CDC serial port (/dev/cu.usbmodem* on macOS) and speaks a
// line based text protocol (same as moteus' Python fdcanusb_device.py):
//
//   host -> adapter:  "can send 8001 01000A0D20....  BF\n"   (B = bit rate switch, F = CAN-FD)
//   adapter -> host:  "OK"                                      (frame queued)
//                     "rcv 100 2C0400... E B F"                  (frame received from the bus)
//                     "ERR ..."                                  (error)
//
// The adapter sets the bus timing itself (1 Mbps arbitration / 5 Mbps data by default), which
// matches the motor (see fdcan.c of the vendor example).  Nothing needs to be configured here.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace hightorque {

struct CanFrame {
  uint32_t id = 0;
  std::vector<uint8_t> data;
  bool brs = true;  // bit rate switch (data phase at 5 Mbps)
  bool fd = true;   // CAN-FD frame
  std::chrono::steady_clock::time_point rx_time{};  // when the host parsed it (received frames only)
};

class Fdcanusb {
 public:
  using Clock = std::chrono::steady_clock;

  // `path` empty => auto-detect the first /dev/cu.usbmodem* (macOS) or /dev/ttyACM* (Linux).
  explicit Fdcanusb(const std::string& path = "");
  // Not connected to anything; only for building frames offline (unit tests).  Any I/O throws.
  struct Offline {};
  explicit Fdcanusb(Offline) : path_("(offline)") {}
  ~Fdcanusb();

  Fdcanusb(const Fdcanusb&) = delete;
  Fdcanusb& operator=(const Fdcanusb&) = delete;

  static std::vector<std::string> detect();
  const std::string& path() const { return path_; }

  // Send frames and wait for the adapter's "OK" for each one, plus `expected_replies` received
  // frames whose source address is in `reply_sources` (empty = accept any source).  All frames
  // are written with a single write() so they go out in as few USB packets as possible.
  // Returns the received frames (may be fewer than expected on timeout).
  std::vector<CanFrame> transact(const std::vector<CanFrame>& tx, size_t expected_replies,
                                 const std::vector<uint8_t>& reply_sources,
                                 std::chrono::microseconds timeout);

  // Send without waiting for any reply frame (still waits for "OK").
  void send(const CanFrame& frame, std::chrono::microseconds timeout = std::chrono::milliseconds(50));

  // Return one received frame (from the queue, or wait up to `timeout`).
  bool receive(CanFrame& out, std::chrono::microseconds timeout);

  // Send a raw text command to the adapter (e.g. "conf get can.termination") and collect the
  // lines it prints until "OK" / "ERR".
  std::vector<std::string> command(const std::string& line,
                                   std::chrono::microseconds timeout = std::chrono::milliseconds(500));

  // Print every line sent / received to stderr.
  void set_debug(bool debug) { debug_ = debug; }

  // Send every frame without bit rate switch, i.e. the data phase also runs at 1 Mbps.
  // Slower, but much more tolerant of bad termination / long wires.
  void set_disable_brs(bool disable) { disable_brs_ = disable; }

  // Ask the adapter for its CAN controller state ("can status") and return true if it is in
  // bus-off.  `status_line` receives the raw text, e.g. "lec=7 dlec=4 err=1 warn=1 busoff=1 ...".
  bool bus_off(std::string* status_line = nullptr);
  // Restart the adapter's CAN controller ("can off" + "can on"), e.g. after bus-off.
  void restart_can();
  // Set the adapter's nominal (1 Mbps) and data (5 Mbps) sample points in RAM only (no
  // "conf write", so unplugging restores the adapter's own values), then restart CAN so they
  // take effect.  The adapter default 0.666 / 0.666 loses most 5 Mbps frames (README "根本原因").
  // Default 0.75 / 0.75 works for BOTH lab motors on one bus (measured 2026-11-11, README "CAN 丢包"):
  //   nominal  5047 (HTDW-5047-36): 0.725-0.80 OK     4438 (HTDW-4438-32): 0.725-0.775 OK, 0.79 marginal,
  //            0.80 fails (the vendor STM32 value 0.80 made the 4438 lose 20-75% of its replies)
  //   data     0.60-0.80 OK for both
  // A new motor (or new firmware) must be checked against these values before changing them.
  static constexpr double kMotorSamplePoint = 0.75;
  static constexpr double kMotorDataSamplePoint = 0.75;
  void set_sample_points(double nominal = kMotorSamplePoint, double data = kMotorDataSamplePoint);

  // Statistics.
  uint64_t timeouts() const { return timeouts_; }
  // Late replies: frames from a motor we are about to talk to that belong to an EARLIER request (it
  // timed out).  transact() drops them instead of taking them as the answer to the new request:
  //   * frames already waiting before the new request is written, and
  //   * frames parsed less than kMinReplyUs after the write (no real answer can be that fast: the
  //     shortest measured round trip is about 500 us).
  // Counted per source ID (the motor ID).  A reply that is late is not lost: on a healthy bus
  // "missed" (no reply in time) minus "late" should be 0.
  static constexpr int kMinReplyUs = 200;
  // Exact counts, per motor ID, since the adapter was opened: frames sent that ask for a reply
  // (reply flag 0x8000), and frames received from that ID (whether used, late or dropped).  Over a
  // whole run, after waiting for stragglers (drain()), requests - replies = replies that never arrived.
  uint64_t requests_sent(uint8_t dest) const { return requests_[dest & 0x7F]; }
  uint64_t replies_received(uint8_t source) const { return received_[source & 0x7F]; }
  // ---- non-blocking use (pipelining): write without waiting for "OK" or replies, collect later ----
  // Write the frames in one go and return at once (requests are counted and traced as in transact()).
  void write_frames(const std::vector<CanFrame>& tx);
  // Move every received CAN frame into `out`; if none is queued, wait for one until `deadline`
  // (microsecond resolution; other adapter lines such as "OK" do not end the wait).  Returns false
  // only when the deadline passed with no frame.
  bool wait_frames(std::vector<CanFrame>& out, Clock::time_point deadline);
  // "ERR" lines from the adapter since it was opened (e.g. a rejected "can send").
  uint64_t adapter_errors() const { return err_total_; }
  // Read and drop everything that arrives during `t` (e.g. late replies after the last command).
  void drain(std::chrono::microseconds t);
  // Write one line per CAN frame to `f` (nullptr = off):  "tx <us> <dest id>" when written to the
  // adapter, "rx <us> <source id>" when the host parsed it; us = steady clock, from set_trace().
  void set_trace(std::FILE* f) {
    trace_ = f;
    trace_t0_ = Clock::now();
  }
  uint64_t late_replies(uint8_t source) const { return late_[source & 0x7F]; }
  uint64_t late_replies() const {
    uint64_t s = 0;
    for (auto v : late_) s += v;
    return s;
  }

 private:
  std::string format_send(const CanFrame& frame) const;
  void write_all(const std::string& s);
  // Read whatever is available, waiting at most until `deadline`.  Returns false on timeout.
  bool pump(Clock::time_point deadline);
  // Same with microsecond resolution (select); used by wait_frames().
  bool pump_us(Clock::time_point deadline);
  // Pull complete lines out of rx_buffer_ and dispatch them.
  void process_lines();
  void handle_line(const std::string& line);

  std::string path_;
  int fd_ = -1;
  bool debug_ = false;
  bool disable_brs_ = false;
  bool checksum_ = false;  // enabled automatically if the adapter asks for it

  std::string rx_buffer_;
  std::deque<CanFrame> frames_;           // received, not yet consumed
  std::vector<std::string> other_lines_;  // non rcv / OK lines (for command())
  size_t ok_count_ = 0;
  size_t err_count_ = 0;
  std::string last_error_;
  uint64_t timeouts_ = 0;
  std::array<uint64_t, 128> late_{};
  std::array<uint64_t, 128> requests_{};
  std::array<uint64_t, 128> received_{};
  std::FILE* trace_ = nullptr;
  uint64_t err_total_ = 0;
  Clock::time_point trace_t0_{};
};

}  // namespace hightorque
