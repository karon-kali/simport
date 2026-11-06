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

#include "simport_plugins/socket_can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "simport_plugins/net.hpp"

namespace simport::plugins
{

SocketCan::~SocketCan()
{
  close();
}

void SocketCan::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "can", "fd", "bps", "max_delay"});
  context_.emplace(context);
  interface_ = config.get<std::string>("can");
  if (interface_.empty() || interface_.size() >= IFNAMSIZ) {
    config["can"].error("expected a network interface name such as vcan0");
  }
  fd_frames_ = config.get<bool>("fd", false);
}

void SocketCan::open()
{
  if (is_open()) {
    return;
  }
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    throw TransportError("cannot create a CAN socket: " + errno_text(errno));
  }
  const unsigned index = ::if_nametoindex(interface_.c_str());
  if (index == 0) {
    ::close(fd);
    throw TransportError(
            "no CAN interface '" + interface_ + "'. Create a virtual one once with: sudo modprobe "
            "vcan && sudo ip link add dev " + interface_ + " type vcan mtu 72 && sudo ip link "
            "set up " + interface_);
  }
  if (fd_frames_) {
    const int one = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &one, sizeof(one)) != 0) {
      ::close(fd);
      throw TransportError("cannot enable CAN-FD frames on " + interface_ + ": " +
              errno_text(errno));
    }
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(index);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    const int err = errno;
    ::close(fd);
    throw TransportError("cannot bind to CAN interface " + interface_ + ": " + errno_text(err));
  }
  fd_ = fd;
  rx_thread_ = std::jthread([this](std::stop_token stop) {rx_loop(stop);});
  context_->logger().info("CAN device ready: {}", describe());
}

void SocketCan::close()
{
  if (!is_open()) {
    return;
  }
  rx_thread_.request_stop();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  ::close(fd_);
  fd_ = -1;
}

void SocketCan::rx_loop(std::stop_token stop)
{
  canfd_frame frame{};
  while (!stop.stop_requested()) {
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, 50) <= 0 || !(pfd.revents & POLLIN)) {
      continue;
    }
    const ssize_t n = ::read(fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
      if (n < 0 && errno != EAGAIN && errno != EINTR) {
        ++rx_errors_;
      }
      continue;
    }
    if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) {
      continue;  // bus errors and remote requests carry no data for a device
    }
    ++frames_rx_;
    bytes_rx_ += frame.len;
    if (callback_) {
      FrameMeta meta;
      const bool extended = frame.can_id & CAN_EFF_FLAG;
      meta.can_id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      if (extended) {
        meta.attributes[kCanExtendedAttribute] = "1";
      }
      if (n == static_cast<ssize_t>(CANFD_MTU)) {
        meta.attributes[kCanFdAttribute] = "1";
      }
      callback_(ByteView(frame.data, frame.len), meta, context_->clock().now());
    }
  }
}

bool SocketCan::write(const Frame & frame)
{
  const bool fd = frame.data.size() > CAN_MAX_DLEN ||
    frame.meta.attributes.contains(kCanFdAttribute);
  if (!is_open() || !frame.meta.can_id || frame.data.size() > CANFD_MAX_DLEN ||
    (fd && !fd_frames_))
  {
    ++tx_dropped_;
    return false;
  }
  canfd_frame out{};
  out.can_id = *frame.meta.can_id;
  if (frame.meta.attributes.contains(kCanExtendedAttribute) || out.can_id > CAN_SFF_MASK) {
    out.can_id |= CAN_EFF_FLAG;
  }
  out.len = static_cast<std::uint8_t>(frame.data.size());
  std::memcpy(out.data, frame.data.data(), frame.data.size());
  const std::size_t size = fd ? CANFD_MTU : CAN_MTU;
  if (::write(fd_, &out, size) != static_cast<ssize_t>(size)) {
    ++tx_dropped_;  // e.g. ENOBUFS when the bus queue is full
    return false;
  }
  ++frames_tx_;
  bytes_tx_ += frame.data.size();
  return true;
}

TransportCaps SocketCan::caps() const
{
  TransportCaps caps;
  caps.stream = false;
  caps.max_frame_size = fd_frames_ ? CANFD_MAX_DLEN : CAN_MAX_DLEN;
  return caps;
}

TransportStats SocketCan::stats() const
{
  return TransportStats{frames_tx_, bytes_tx_, frames_rx_, bytes_rx_, tx_dropped_, rx_errors_, {}};
}

std::string SocketCan::describe() const
{
  return "can " + interface_ + (fd_frames_ ? " (CAN-FD)" : "");
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::SocketCan, simport::Transport)
