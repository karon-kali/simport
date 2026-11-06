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

#include "simport_plugins/udp.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <mutex>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

namespace
{
constexpr std::size_t kMaxDatagram = 65507;
}  // namespace

std::string errno_text(int err) {return std::error_code(err, std::generic_category()).message();}

sockaddr_in to_sockaddr(const Endpoint & e)
{
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(e.port);
  if (::inet_pton(AF_INET, e.host.c_str(), &addr.sin_addr) == 1) {
    return addr;
  }
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo * result = nullptr;
  if (::getaddrinfo(e.host.c_str(), nullptr, &hints, &result) != 0 || result == nullptr) {
    throw std::invalid_argument("cannot resolve '" + e.host + "' to an IPv4 address");
  }
  addr.sin_addr = reinterpret_cast<sockaddr_in *>(result->ai_addr)->sin_addr;
  ::freeaddrinfo(result);
  return addr;
}

Endpoint from_sockaddr(const sockaddr_in & addr)
{
  char buf[INET_ADDRSTRLEN] = {};
  ::inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf));
  return Endpoint{buf, ntohs(addr.sin_port)};
}

std::string to_string(const Endpoint & e) {return e.host + ":" + std::to_string(e.port);}

Endpoint parse_endpoint(const Config & node)
{
  Endpoint e;
  if (node.is_map()) {
    node.expect_keys({"host", "port"});
    e.host = node.get<std::string>("host");
    e.port = node.get<std::uint16_t>("port");
  } else {
    const std::string text = node.as<std::string>();
    const auto colon = text.rfind(':');
    if (colon == std::string::npos || colon == 0) {
      node.error("expected <host>:<port> or { host: ..., port: ... }");
    }
    e.host = text.substr(0, colon);
    try {
      e.port = parse_integer<std::uint16_t>(text.substr(colon + 1));
    } catch (const std::exception &) {
      node.error("invalid port in '" + text + "'");
    }
  }
  try {
    to_sockaddr(e);
  } catch (const std::invalid_argument & err) {
    node.error(err.what());
  }
  return e;
}

Udp::~Udp()
{
  close();
}

void Udp::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "udp", "to", "broadcast", "bps", "max_delay"});
  context_.emplace(context);
  if (config.has("udp")) {
    bind_ = parse_endpoint(config["udp"]);
  }
  if (config.has("to")) {
    const Config d = config["to"];
    if (d.is_scalar() && d.scalar() == "sender") {
      learn_destination_ = true;
    } else {
      destination_ = parse_endpoint(d);
    }
  }
  broadcast_ = config.get<bool>("broadcast", false);
}

void Udp::open()
{
  if (is_open()) {
    return;
  }
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw TransportError("cannot create UDP socket: " + errno_text(errno));
  }
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (broadcast_) {
    ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  }
  const sockaddr_in addr = to_sockaddr(bind_);
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
    const int err = errno;
    ::close(fd);
    throw TransportError("cannot bind UDP " + to_string(bind_) + ": " + errno_text(err));
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &len);
  local_port_ = ntohs(bound.sin_port);
  fd_ = fd;
  rx_thread_ = std::jthread([this](std::stop_token stop) {rx_loop(stop);});
  context_->logger().info("UDP device ready: {}", describe());
}

void Udp::close()
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
  local_port_ = 0;
}

void Udp::rx_loop(std::stop_token stop)
{
  std::array<std::uint8_t, kMaxDatagram> buffer{};
  while (!stop.stop_requested()) {
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, 50) <= 0 || !(pfd.revents & POLLIN)) {
      continue;
    }
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    const ssize_t n = ::recvfrom(
      fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&from), &len);
    if (n < 0) {
      if (errno != EAGAIN && errno != EINTR) {
        ++rx_errors_;
      }
      continue;
    }
    ++frames_rx_;
    bytes_rx_ += static_cast<std::uint64_t>(n);
    if (learn_destination_) {
      std::lock_guard<std::mutex> lock(destination_mutex_);
      destination_ = from_sockaddr(from);
    }
    if (callback_) {
      FrameMeta meta;
      meta.endpoint = from_sockaddr(from);
      callback_(ByteView(buffer.data(), static_cast<std::size_t>(n)), meta,
        context_->clock().now());
    }
  }
}

bool Udp::write(const Frame & frame)
{
  std::optional<Endpoint> target = frame.meta.endpoint;
  if (!target) {
    std::lock_guard<std::mutex> lock(destination_mutex_);
    target = destination_;
  }
  if (!is_open() || !target || frame.data.size() > kMaxDatagram) {
    ++tx_dropped_;
    return false;
  }
  sockaddr_in addr;
  try {
    addr = to_sockaddr(*target);
  } catch (const std::invalid_argument &) {
    ++tx_dropped_;
    return false;
  }
  const ssize_t n = ::sendto(
    fd_, frame.data.data(), frame.data.size(), 0, reinterpret_cast<const sockaddr *>(&addr),
    sizeof(addr));
  if (n != static_cast<ssize_t>(frame.data.size())) {
    ++tx_dropped_;  // e.g. ECONNREFUSED from a previous send, or a full socket buffer
    return false;
  }
  ++frames_tx_;
  bytes_tx_ += frame.data.size();
  return true;
}

TransportCaps Udp::caps() const
{
  TransportCaps caps;
  caps.stream = false;
  caps.max_frame_size = kMaxDatagram;
  return caps;
}

TransportStats Udp::stats() const
{
  TransportStats s{frames_tx_, bytes_tx_, frames_rx_, bytes_rx_, tx_dropped_, rx_errors_, {}};
  return s;
}

std::string Udp::describe() const
{
  Endpoint local = bind_;
  if (local_port_ != 0) {
    local.port = local_port_;
  }
  std::lock_guard<std::mutex> lock(destination_mutex_);
  const std::string to = learn_destination_ ? " -> last sender" :
    destination_ ? " -> " + to_string(*destination_) : std::string(" (replies only)");
  return "udp " + to_string(local) + to;
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::Udp, simport::Transport)
