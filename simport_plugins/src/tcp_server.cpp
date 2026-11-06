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

#include "simport_plugins/tcp_server.hpp"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "simport_core/input_channel.hpp"
#include "simport_plugins/net.hpp"

namespace simport::plugins
{

namespace
{
constexpr int kPollMs = 50;
constexpr auto kSendTimeout = std::chrono::milliseconds(500);
}  // namespace

TcpServer::~TcpServer()
{
  close();
}

void TcpServer::configure(const Config & config, Context & context)
{
  config.expect_keys({"plugin", "tcp", "max_clients", "bps", "max_delay"});
  context_.emplace(context);
  listen_ = parse_endpoint(config.required("tcp"));
  max_clients_ = config.get<std::uint32_t>("max_clients", 8);
  if (max_clients_ == 0) {
    config["max_clients"].error("max_clients must be at least 1");
  }
}

void TcpServer::open()
{
  if (is_open()) {
    return;
  }
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw TransportError("cannot create TCP socket: " + errno_text(errno));
  }
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  const sockaddr_in addr = to_sockaddr(listen_);
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0) {
    const int err = errno;
    ::close(fd);
    std::string hint;
    if (err == EACCES && listen_.port < 1024) {
      hint = " (ports below 1024 need privileges: use a higher port if the driver allows it, "
        "or run once: sudo sysctl net.ipv4.ip_unprivileged_port_start=" +
        std::to_string(listen_.port) + ")";
    }
    throw TransportError(
            "cannot listen on TCP " + to_string(listen_) + ": " + errno_text(err) + hint);
  }
  if (::listen(fd, 16) != 0) {
    const int err = errno;
    ::close(fd);
    throw TransportError("cannot listen on TCP " + to_string(listen_) + ": " + errno_text(err));
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &len);
  local_port_ = ntohs(bound.sin_port);
  listen_fd_ = fd;
  io_thread_ = std::jthread([this](std::stop_token stop) {io_loop(stop);});
  context_->logger().info("TCP device ready: {}", describe());
}

void TcpServer::close()
{
  if (!is_open()) {
    return;
  }
  io_thread_.request_stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  std::lock_guard<std::mutex> lock(clients_mutex_);
  for (auto & entry : clients_) {
    ::close(entry.first);
  }
  clients_.clear();
  ::close(listen_fd_);
  listen_fd_ = -1;
  local_port_ = 0;
}

void TcpServer::accept_client()
{
  sockaddr_in from{};
  socklen_t len = sizeof(from);
  const int fd = ::accept4(
    listen_fd_, reinterpret_cast<sockaddr *>(&from), &len, SOCK_NONBLOCK | SOCK_CLOEXEC);
  if (fd < 0) {
    return;
  }
  const Endpoint peer = from_sockaddr(from);
  std::lock_guard<std::mutex> lock(clients_mutex_);
  if (clients_.size() >= max_clients_) {
    ::close(fd);
    context_->logger().warn("refused TCP client {}: already {} clients", to_string(peer),
      clients_.size());
    return;
  }
  const int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  clients_[fd] = Client{fd, peer};
  context_->logger().debug("TCP client {} connected", to_string(peer));
}

void TcpServer::drop_client(int fd)
{
  Endpoint peer;
  {
    // Not while write() may be sending on this descriptor.
    std::lock_guard<std::mutex> write_lock(write_mutex_);
    std::lock_guard<std::mutex> lock(clients_mutex_);
    const auto it = clients_.find(fd);
    if (it == clients_.end()) {
      return;
    }
    peer = it->second.peer;
    ::close(fd);
    clients_.erase(it);
  }
  context_->logger().debug("TCP client {} disconnected", to_string(peer));
  if (callback_) {
    // Lets the device discard this client's partial frame.
    FrameMeta meta;
    meta.endpoint = peer;
    meta.attributes[kDisconnectedAttribute] = "1";
    callback_(ByteView(), meta, context_->clock().now());
  }
}

void TcpServer::io_loop(std::stop_token stop)
{
  std::array<std::uint8_t, 65536> buffer{};
  std::vector<pollfd> fds;
  std::vector<Endpoint> peers;
  while (!stop.stop_requested()) {
    fds.assign(1, pollfd{listen_fd_, POLLIN, 0});
    peers.assign(1, Endpoint{});
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      for (const auto & [fd, client] : clients_) {
        fds.push_back(pollfd{fd, POLLIN, 0});
        peers.push_back(client.peer);
      }
    }
    if (::poll(fds.data(), fds.size(), kPollMs) <= 0) {
      continue;
    }
    if (fds[0].revents & POLLIN) {
      accept_client();
    }
    for (std::size_t i = 1; i < fds.size(); ++i) {
      if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) {
        continue;
      }
      const ssize_t n = ::recv(fds[i].fd, buffer.data(), buffer.size(), 0);
      if (n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR)) {
        if (n < 0) {
          ++rx_errors_;
        }
        drop_client(fds[i].fd);
        continue;
      }
      if (n < 0) {
        continue;
      }
      ++frames_rx_;
      bytes_rx_ += static_cast<std::uint64_t>(n);
      if (callback_) {
        FrameMeta meta;
        meta.endpoint = peers[i];
        callback_(ByteView(buffer.data(), static_cast<std::size_t>(n)), meta,
          context_->clock().now());
      }
    }
  }
}

bool TcpServer::send_all(int fd, const Bytes & data)
{
  std::size_t sent = 0;
  const auto deadline = std::chrono::steady_clock::now() + kSendTimeout;
  while (sent < data.size()) {
    const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    }
    // Socket buffer full: wait for the client to read, within limits.
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      return false;
    }
    pollfd pfd{fd, POLLOUT, 0};
    ::poll(&pfd, 1, static_cast<int>(left));
  }
  return true;
}

bool TcpServer::write(const Frame & frame)
{
  std::vector<int> targets;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (const auto & [fd, client] : clients_) {
      if (!frame.meta.endpoint || client.peer == *frame.meta.endpoint) {
        targets.push_back(fd);
      }
    }
  }
  if (targets.empty()) {
    ++tx_dropped_;  // nobody connected, or the asking client is gone
    return false;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  bool ok = true;
  for (const int fd : targets) {
    // A failed or partial send leaves the stream unusable, as it would for a real device:
    // shut the connection down; the receive thread then drops it.
    if (!send_all(fd, frame.data)) {
      ::shutdown(fd, SHUT_RDWR);
      ok = false;
    }
  }
  if (!ok) {
    ++tx_dropped_;
    return false;
  }
  ++frames_tx_;
  bytes_tx_ += frame.data.size();
  return true;
}

TransportCaps TcpServer::caps() const
{
  TransportCaps caps;
  caps.stream = true;
  return caps;
}

TransportStats TcpServer::stats() const
{
  TransportStats s{frames_tx_, bytes_tx_, frames_rx_, bytes_rx_, tx_dropped_, rx_errors_, {}};
  std::lock_guard<std::mutex> lock(clients_mutex_);
  s.peer_connected = !clients_.empty();
  return s;
}

std::string TcpServer::describe() const
{
  Endpoint local = listen_;
  if (local_port_ != 0) {
    local.port = local_port_;
  }
  return "tcp " + to_string(local);
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::TcpServer, simport::Transport)
