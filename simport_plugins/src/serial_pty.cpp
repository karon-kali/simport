// Copyright 2026 simport contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simport_plugins/serial_pty.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace simport::plugins
{

namespace
{

std::string errno_text(int err) {return std::error_code(err, std::generic_category()).message();}

/// Nominal termios speed for informational purposes; PTYs ignore it.
speed_t to_speed(std::uint32_t baud)
{
  switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default: return B38400;
  }
}

std::uint32_t from_speed(speed_t speed)
{
  for (const std::uint32_t baud : {1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u,
      115200u, 230400u, 460800u, 500000u, 921600u, 1000000u, 2000000u, 3000000u, 4000000u})
  {
    if (to_speed(baud) == speed) {
      return baud;
    }
  }
  return 0;
}

}  // namespace

SerialFraming SerialFraming::parse(const std::string & text)
{
  if (text.size() != 3 || text[0] < '5' || text[0] > '8' ||
    (text[1] != 'N' && text[1] != 'E' && text[1] != 'O') || (text[2] != '1' && text[2] != '2'))
  {
    throw std::invalid_argument("expected <data bits 5-8><parity N|E|O><stop bits 1|2>");
  }
  return SerialFraming{text[0] - '0', text[1], text[2] - '0'};
}

SerialPty::~SerialPty()
{
  close();
}

void SerialPty::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "serial", "baud", "framing", "throttle", "max_delay"});
  context_.emplace(context);
  link_ = config.resolve_path(config.get<std::string>("serial"));
  baud_ = config.get<std::uint32_t>("baud", 115200);
  throttle_ = config.get<bool>("throttle", true);
  if (baud_ == 0) {
    config["baud"].error("baud must be positive");
  }
  const Config framing = config["framing"];
  try {
    framing_ = SerialFraming::parse(config.get<std::string>("framing", "8N1"));
  } catch (const std::invalid_argument & e) {
    framing.error(e.what());
  }
}

void SerialPty::set_receive_callback(ReceiveCallback callback)
{
  callback_ = std::move(callback);
}

void SerialPty::open()
{
  if (is_open()) {
    return;
  }
  int master = -1;
  int slave = -1;
  if (::openpty(&master, &slave, nullptr, nullptr, nullptr) != 0) {
    throw TransportError("openpty failed: " + errno_text(errno));
  }
  const char * name = ::ttyname(slave);
  slave_name_ = name != nullptr ? name : "";

  // Raw mode, so the line discipline neither echoes nor rewrites bytes. Settings persist after
  // we close our slave fd; drivers usually configure the port themselves anyway.
  termios tio{};
  ::tcgetattr(slave, &tio);
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | PARODD | CSTOPB);
  tio.c_cflag |= framing_.data_bits == 5 ? CS5 : framing_.data_bits == 6 ? CS6 :
    framing_.data_bits == 7 ? CS7 : CS8;
  if (framing_.parity != 'N') {
    tio.c_cflag |= PARENB | (framing_.parity == 'O' ? PARODD : 0);
  }
  if (framing_.stop_bits == 2) {
    tio.c_cflag |= CSTOPB;
  }
  ::cfsetspeed(&tio, to_speed(baud_));
  ::tcsetattr(slave, TCSANOW, &tio);
  // Not keeping the slave open lets poll() report POLLHUP while no driver is connected.
  ::close(slave);
  ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);

  std::error_code ec;
  fs::create_directories(link_.parent_path(), ec);
  const auto status = fs::symlink_status(link_, ec);
  if (fs::exists(status)) {
    if (!fs::is_symlink(status)) {
      ::close(master);
      throw TransportError(
              "refusing to replace '" + link_.string() + "': it exists and is not a symlink");
    }
    fs::remove(link_, ec);  // stale link from a previous run
  }
  fs::create_symlink(slave_name_, link_, ec);
  if (ec) {
    ::close(master);
    throw TransportError("cannot create symlink '" + link_.string() + "': " + ec.message());
  }

  master_fd_ = master;
  connected_ = false;
  rx_thread_ = std::jthread([this](std::stop_token stop) {rx_loop(stop);});
  context_->logger().info("serial port ready at {} ({})", link_.string(), slave_name_);
}

void SerialPty::close()
{
  if (!is_open()) {
    return;
  }
  rx_thread_.request_stop();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    ::close(master_fd_);
    master_fd_ = -1;
    connected_ = false;
  }
  std::error_code ec;
  if (fs::is_symlink(link_, ec) && fs::read_symlink(link_, ec) == slave_name_) {
    fs::remove(link_, ec);
  }
  slave_name_.clear();
}

void SerialPty::set_connected(bool connected)
{
  if (connected_ == connected) {
    return;
  }
  if (connected) {
    // Never flush here: the driver may already have written its first command.
    connected_ = true;
    line_warned_ = false;
    next_line_check_ = std::chrono::steady_clock::now() + 200ms;  // let the driver configure
  } else {
    // Under the write lock, so no write can slip in between the flag and the flush. Anything
    // written just before the driver closed is discarded, so a driver that reopens the port
    // never sees stale data. Nobody has the slave open, so nothing of the driver's is lost.
    std::lock_guard<std::mutex> lock(write_mutex_);
    connected_ = false;
    ::tcflush(master_fd_, TCIOFLUSH);
  }
  context_->logger().info("driver {} {}", connected ? "opened" : "closed", link_.string());
}

void SerialPty::rx_loop(std::stop_token stop)
{
  std::array<std::uint8_t, 4096> buffer{};
  while (!stop.stop_requested()) {
    pollfd pfd{master_fd_, POLLIN, 0};
    const int n = ::poll(&pfd, 1, 50);
    if (n < 0) {
      if (errno != EINTR) {
        ++rx_errors_;
        std::this_thread::sleep_for(10ms);
      }
      continue;
    }
    if (pfd.revents & POLLHUP) {
      // POLLHUP is level-triggered while nobody has the slave open; back off.
      set_connected(false);
      std::this_thread::sleep_for(20ms);
      continue;
    }
    set_connected(true);
    check_line_settings();
    if (n == 0 || !(pfd.revents & POLLIN)) {
      continue;
    }
    const ssize_t got = ::read(master_fd_, buffer.data(), buffer.size());
    if (got > 0) {
      ++frames_rx_;
      bytes_rx_ += static_cast<std::uint64_t>(got);
      if (callback_) {
        callback_(
          ByteView(buffer.data(), static_cast<std::size_t>(got)), FrameMeta{},
          context_->clock().now());
      }
    } else if (got < 0 && errno != EAGAIN && errno != EINTR && errno != EIO) {
      ++rx_errors_;
    }
  }
}

bool SerialPty::write(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (master_fd_ < 0 || !connected_) {
    ++tx_dropped_;
    return false;
  }
  std::size_t done = 0;
  while (done < frame.data.size()) {
    const ssize_t n = ::write(master_fd_, frame.data.data() + done, frame.data.size() - done);
    if (n > 0) {
      done += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    // EAGAIN: the driver isn't reading fast enough and the pty buffer is full. A real line
    // would overrun too; drop the rest of the frame.
    ++tx_dropped_;
    return false;
  }
  ++frames_tx_;
  bytes_tx_ += done;
  return true;
}

void SerialPty::check_line_settings()
{
  // The pty master reports the termios the driver applied on its end. A real device at the
  // configured baud would look like noise to a driver set differently, so tell the user.
  const auto now = std::chrono::steady_clock::now();
  if (line_warned_ || now < next_line_check_) {
    return;
  }
  next_line_check_ = now + 500ms;
  termios tio{};
  if (::tcgetattr(master_fd_, &tio) != 0) {
    return;
  }
  const speed_t speed = ::cfgetospeed(&tio);
  const std::uint32_t driver_baud = from_speed(speed);
  const int driver_bits = (tio.c_cflag & CSIZE) == CS5 ? 5 : (tio.c_cflag & CSIZE) == CS6 ? 6 :
    (tio.c_cflag & CSIZE) == CS7 ? 7 : 8;
  const char driver_parity = !(tio.c_cflag & PARENB) ? 'N' : (tio.c_cflag & PARODD) ? 'O' : 'E';
  const int driver_stop = (tio.c_cflag & CSTOPB) ? 2 : 1;
  const bool baud_ok = driver_baud == 0 || driver_baud == baud_;  // 0: unknown/nonstandard
  const bool framing_ok = driver_bits == framing_.data_bits &&
    driver_parity == framing_.parity && driver_stop == framing_.stop_bits;
  if (!baud_ok || !framing_ok) {
    line_warned_ = true;
    context_->logger().warn(
      "driver configured {} for {} baud {}{}{}, but the device is {} baud {}{}{}; a real "
      "device would look like garbage to it", link_.string(), driver_baud, driver_bits,
      driver_parity, driver_stop, baud_, framing_.data_bits, framing_.parity, framing_.stop_bits);
  }
}

TransportCaps SerialPty::caps() const
{
  TransportCaps caps;
  caps.stream = true;
  if (throttle_) {
    caps.line_rate_bps = static_cast<double>(baud_);
  }
  caps.bits_per_byte = framing_.bits_per_byte();
  return caps;
}

TransportStats SerialPty::stats() const
{
  return TransportStats{frames_tx_, bytes_tx_, frames_rx_, bytes_rx_, tx_dropped_, rx_errors_,
    is_open() ? std::optional<bool>(connected_.load()) : std::nullopt};
}

std::string SerialPty::describe() const
{
  return link_.string() + (slave_name_.empty() ? "" : " -> " + slave_name_);
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::SerialPty, simport::Transport)
