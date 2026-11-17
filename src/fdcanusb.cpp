#include "hightorque/fdcanusb.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "hightorque/protocol.hpp"

namespace hightorque {

namespace {

std::vector<std::string> glob_paths(const char* pattern) {
  std::vector<std::string> result;
  glob_t g;
  if (glob(pattern, 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; i++) result.emplace_back(g.gl_pathv[i]);
  }
  globfree(&g);
  return result;
}

// CRC-8, polynomial 0x97, nybble table.  Same as moteus fdcanusb_device.py.  Only used if the
// adapter replies "ERR checksum" (UART-style links); a normal mjcanfd-usb-1x does not need it.
uint8_t crc8(const std::string& s) {
  static const uint8_t table[16] = {0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
                                    0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96};
  uint8_t crc = 0;
  for (unsigned char b : s) {
    crc = table[((crc >> 4) ^ (b >> 4)) & 0x0f] ^ static_cast<uint8_t>(crc << 4);
    crc = table[((crc >> 4) ^ (b & 0x0f)) & 0x0f] ^ static_cast<uint8_t>(crc << 4);
  }
  return crc;
}

int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && s[i] == ' ') i++;
    size_t j = i;
    while (j < s.size() && s[j] != ' ') j++;
    if (j > i) out.push_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

}  // namespace

std::vector<std::string> Fdcanusb::detect() {
  std::vector<std::string> result;
  for (const char* pattern : {"/dev/serial/by-id/*fdcanusb*", "/dev/cu.usbmodem*", "/dev/ttyACM*"}) {
    for (auto& p : glob_paths(pattern)) result.push_back(p);
  }
  return result;
}

Fdcanusb::Fdcanusb(const std::string& path) : path_(path) {
  if (path_.empty()) {
    const auto found = detect();
    if (found.empty()) {
      throw std::runtime_error("no CAN-FD USB adapter found (looked for /dev/cu.usbmodem*). Is it plugged in?");
    }
    path_ = found.front();
  }

  fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + path_ + ": " + std::strerror(errno));
  }
  // Exclusive access, so a second program (or moteus_tool) cannot interleave lines.
  ::ioctl(fd_, TIOCEXCL);

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    ::close(fd_);
    throw std::runtime_error("tcgetattr failed on " + path_);
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetspeed(&tio, B115200);  // USB CDC: the adapter ignores the baud rate
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    ::close(fd_);
    throw std::runtime_error("tcsetattr failed on " + path_);
  }
  ::tcflush(fd_, TCIOFLUSH);
}

Fdcanusb::~Fdcanusb() {
  if (fd_ >= 0) ::close(fd_);
}

std::string Fdcanusb::format_send(const CanFrame& frame) const {
  char id[16];
  std::snprintf(id, sizeof(id), "%04x", frame.id);
  // Pad to a valid DLC with 0x50 (NOP), like moteus does.
  std::vector<uint8_t> data = frame.data;
  data.resize(round_up_dlc(data.size()), kPadding);

  std::string line = "can send ";
  line += id;
  line += ' ';
  line += to_hex(data);
  line += ' ';
  line += (frame.brs && !disable_brs_) ? 'B' : 'b';
  if (frame.fd) line += 'F';
  if (checksum_) {
    line += ' ';
    char buf[8];
    std::snprintf(buf, sizeof(buf), "*%02X", crc8(line));
    line += buf;
  }
  line += '\n';
  return line;
}

void Fdcanusb::write_all(const std::string& s) {
  if (fd_ < 0) throw std::runtime_error("adapter not open");
  if (debug_) std::cerr << "> " << s;
  size_t done = 0;
  while (done < s.size()) {
    const ssize_t n = ::write(fd_, s.data() + done, s.size() - done);
    if (n > 0) {
      done += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      throw std::runtime_error(std::string("write to adapter failed: ") + std::strerror(errno));
    }
    pollfd p{fd_, POLLOUT, 0};
    ::poll(&p, 1, 10);
  }
}

bool Fdcanusb::pump(Clock::time_point deadline) {
  if (fd_ < 0) throw std::runtime_error("adapter not open");
  const auto now = Clock::now();
  int timeout_ms = 0;
  if (deadline > now) {
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    timeout_ms = static_cast<int>((us + 999) / 1000);
  }
  pollfd p{fd_, POLLIN, 0};
  const int r = ::poll(&p, 1, timeout_ms);
  if (r < 0) {
    if (errno == EINTR) return true;
    throw std::runtime_error(std::string("poll failed: ") + std::strerror(errno));
  }
  if (r == 0) return false;
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("adapter disconnected (" + path_ + ")");
  }

  char buf[4096];
  for (;;) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      rx_buffer_.append(buf, static_cast<size_t>(n));
      if (static_cast<size_t>(n) < sizeof(buf)) break;
    } else {
      break;
    }
  }
  process_lines();
  return true;
}

void Fdcanusb::process_lines() {
  size_t start = 0;
  for (size_t i = 0; i < rx_buffer_.size(); i++) {
    const char c = rx_buffer_[i];
    if (c == '\r' || c == '\n') {
      if (i > start) handle_line(rx_buffer_.substr(start, i - start));
      start = i + 1;
    }
  }
  rx_buffer_.erase(0, start);
  // Protect against a garbled stream that never sends a newline.
  if (rx_buffer_.size() > 4096) rx_buffer_.clear();
}

void Fdcanusb::handle_line(const std::string& raw) {
  if (debug_) std::cerr << "< " << raw << "\n";
  std::string line = raw;
  // Strip an optional " *XX" checksum.
  if (line.size() >= 4 && line[line.size() - 3] == '*') {
    line.erase(line.size() - 3);
    while (!line.empty() && line.back() == ' ') line.pop_back();
  }

  if (line.rfind("rcv", 0) == 0) {
    const auto fields = split(line);
    if (fields.size() < 3) return;
    CanFrame f;
    f.id = static_cast<uint32_t>(std::strtoul(fields[1].c_str(), nullptr, 16));
    const std::string& hex = fields[2];
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
      const int hi = hex_nibble(hex[i]);
      const int lo = hex_nibble(hex[i + 1]);
      if (hi < 0 || lo < 0) return;
      f.data.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    f.brs = false;
    f.fd = false;
    for (size_t i = 3; i < fields.size(); i++) {
      if (fields[i] == "B") f.brs = true;
      if (fields[i] == "F") f.fd = true;
    }
    f.rx_time = Clock::now();
    received_[can_id_source(f.id) & 0x7F]++;
    if (trace_) {
      std::fprintf(trace_, "rx %lld %u\n",
                   static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(f.rx_time - trace_t0_).count()),
                   can_id_source(f.id));
    }
    frames_.push_back(std::move(f));
    // Bound the queue (e.g. timed-return frames nobody reads).
    while (frames_.size() > 1024) frames_.pop_front();
  } else if (line.rfind("OK", 0) == 0) {
    ok_count_++;
  } else if (line.rfind("ERR", 0) == 0) {
    err_count_++;
    err_total_++;
    last_error_ = line;
    if (line.find("checksum") != std::string::npos) checksum_ = true;
    other_lines_.push_back(line);
  } else {
    other_lines_.push_back(line);
  }
}

std::vector<CanFrame> Fdcanusb::transact(const std::vector<CanFrame>& tx, size_t expected_replies,
                                         const std::vector<uint8_t>& reply_sources,
                                         std::chrono::microseconds timeout) {
  auto wanted = [&](const CanFrame& f) {
    if (reply_sources.empty()) return true;
    return std::find(reply_sources.begin(), reply_sources.end(), can_id_source(f.id)) != reply_sources.end();
  };

  // Read whatever is already waiting and drop stale replies from the motors we are about to
  // talk to (for example late answers to an earlier, timed-out request).
  pump(Clock::now());
  if (expected_replies > 0) {
    frames_.erase(std::remove_if(frames_.begin(), frames_.end(),
                                 [&](const CanFrame& f) {
                                   if (!wanted(f)) return false;
                                   late_[can_id_source(f.id) & 0x7F]++;
                                   return true;
                                 }),
                  frames_.end());
  }

  std::string out;
  for (const auto& f : tx) out += format_send(f);
  ok_count_ = 0;
  err_count_ = 0;
  const auto before_write = Clock::now();
  write_all(out);
  const auto written = Clock::now();
  for (const auto& f : tx) {
    if (f.id & kReplyFlag) {
      requests_[can_id_destination(f.id) & 0x7F]++;
      if (trace_) {
        std::fprintf(trace_, "tx %lld %u\n",
                     static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(written - trace_t0_).count()),
                     can_id_destination(f.id));
      }
    }
  }
  // From the START of the write: no answer to this request can be parsed earlier than that + kMinReplyUs,
  // however long the write itself took.
  const auto too_early = before_write + std::chrono::microseconds(kMinReplyUs);

  std::vector<CanFrame> replies;
  const auto deadline = written + timeout;
  for (;;) {
    for (auto it = frames_.begin(); it != frames_.end() && replies.size() < expected_replies;) {
      if (wanted(*it) && expected_replies > 0 && it->rx_time < too_early) {
        late_[can_id_source(it->id) & 0x7F]++;  // answer to an earlier request, not to this one
        it = frames_.erase(it);
      } else if (wanted(*it)) {
        replies.push_back(std::move(*it));
        it = frames_.erase(it);
      } else {
        ++it;
      }
    }
    if (ok_count_ >= tx.size() && replies.size() >= expected_replies) break;
    if (err_count_ > 0) {
      if (debug_) std::cerr << "adapter error: " << last_error_ << "\n";
      break;
    }
    if (!pump(deadline)) {
      timeouts_++;
      break;
    }
  }
  return replies;
}

void Fdcanusb::write_frames(const std::vector<CanFrame>& tx) {
  std::string out;
  for (const auto& f : tx) out += format_send(f);
  write_all(out);
  const auto written = Clock::now();
  for (const auto& f : tx) {
    if (f.id & kReplyFlag) {
      requests_[can_id_destination(f.id) & 0x7F]++;
      if (trace_) {
        std::fprintf(trace_, "tx %lld %u\n",
                     static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(written - trace_t0_).count()),
                     can_id_destination(f.id));
      }
    }
  }
}

bool Fdcanusb::pump_us(Clock::time_point deadline) {
  if (fd_ < 0) throw std::runtime_error("adapter not open");
  const auto now = Clock::now();
  const long long us = deadline > now ? std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count() : 0;
  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(fd_, &rd);
  timeval tv{static_cast<decltype(tv.tv_sec)>(us / 1000000), static_cast<decltype(tv.tv_usec)>(us % 1000000)};
  const int r = ::select(fd_ + 1, &rd, nullptr, nullptr, &tv);
  if (r < 0) {
    if (errno == EINTR) return true;
    throw std::runtime_error(std::string("select failed: ") + std::strerror(errno));
  }
  if (r == 0) return false;
  char buf[4096];
  for (;;) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      rx_buffer_.append(buf, static_cast<size_t>(n));
      if (static_cast<size_t>(n) < sizeof(buf)) break;
    } else {
      if (n == 0) throw std::runtime_error("adapter disconnected (" + path_ + ")");
      break;
    }
  }
  process_lines();
  return true;
}

bool Fdcanusb::wait_frames(std::vector<CanFrame>& out, Clock::time_point deadline) {
  // Keep reading until a CAN frame is queued or the deadline passes: adapter lines like "OK" (one per
  // "can send") also wake us up but are not frames.
  while (frames_.empty() && Clock::now() < deadline) pump_us(deadline);
  if (frames_.empty()) return false;
  for (auto& f : frames_) out.push_back(std::move(f));
  frames_.clear();
  return true;
}

void Fdcanusb::drain(std::chrono::microseconds t) {
  const auto deadline = Clock::now() + t;
  while (pump(deadline)) {
  }
  frames_.clear();
}

void Fdcanusb::send(const CanFrame& frame, std::chrono::microseconds timeout) {
  transact({frame}, 0, {}, timeout);
}

bool Fdcanusb::receive(CanFrame& out, std::chrono::microseconds timeout) {
  const auto deadline = Clock::now() + timeout;
  for (;;) {
    if (!frames_.empty()) {
      out = std::move(frames_.front());
      frames_.pop_front();
      return true;
    }
    if (!pump(deadline)) return false;
  }
}

std::vector<std::string> Fdcanusb::command(const std::string& line, std::chrono::microseconds timeout) {
  pump(Clock::now());
  other_lines_.clear();
  ok_count_ = 0;
  err_count_ = 0;
  write_all(line + "\n");
  const auto deadline = Clock::now() + timeout;
  while (ok_count_ == 0 && err_count_ == 0) {
    if (!pump(deadline)) break;
  }
  auto result = other_lines_;
  other_lines_.clear();
  return result;
}

bool Fdcanusb::bus_off(std::string* status_line) {
  for (const auto& line : command("can status")) {
    if (line.find("busoff=") == std::string::npos) continue;
    if (status_line) *status_line = line;
    return line.find("busoff=1") != std::string::npos;
  }
  return false;
}

void Fdcanusb::restart_can() {
  command("can off");
  command("can on");
  // The controller needs a moment to rejoin the bus before "can status" reports it as on.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void Fdcanusb::set_sample_points(double nominal, double data) {
  for (const auto& [key, value] : {std::pair<const char*, double>{"can.sample_point", nominal},
                                   {"can.fd_sample_point", data}}) {
    char line[64];
    std::snprintf(line, sizeof(line), "conf set %s %.3f", key, value);
    for (const auto& reply : command(line)) {
      if (reply.rfind("ERR", 0) == 0) throw std::runtime_error(std::string(line) + " failed: " + reply);
    }
  }
  restart_can();
}

}  // namespace hightorque
