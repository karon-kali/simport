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

#ifndef SIMPORT_PLUGINS__TCP_SERVER_HPP_
#define SIMPORT_PLUGINS__TCP_SERVER_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "simport_core/transport.hpp"

namespace simport::plugins
{

/// Emulated network device accepting TCP connections, such as a sensor's configuration or
/// command port.
///
/// ```yaml
/// port:
///   tcp: 127.0.0.5:8080        # the device's address and port (ports below 1024 need
///                              # privileges, see docs/configuration.md)
///   max_clients: 8             # further connections are refused
/// ```
///
/// Each client is a separate byte stream: received data carries the client's endpoint, so the
/// device frames each client separately and replies go to the client that asked. Frames without
/// an endpoint (streams) go to every connected client.
class TcpServer : public Transport
{
public:
  ~TcpServer() override;

  void configure(const Config & config, Context & context) override;
  void set_receive_callback(ReceiveCallback callback) override {callback_ = std::move(callback);}
  void open() override;
  void close() override;
  bool is_open() const override {return listen_fd_ >= 0;}
  TransportCaps caps() const override;
  bool write(const Frame & frame) override;
  TransportStats stats() const override;
  std::string describe() const override;

  /// Bound port (useful when listening on port 0). 0 when closed.
  std::uint16_t local_port() const {return local_port_;}

private:
  struct Client
  {
    int fd = -1;
    Endpoint peer;
  };

  void io_loop(std::stop_token stop);
  void accept_client();
  void drop_client(int fd);
  bool send_all(int fd, const Bytes & data);

  std::optional<Context> context_;
  Endpoint listen_{"127.0.0.1", 0};
  std::size_t max_clients_ = 8;

  ReceiveCallback callback_;
  int listen_fd_ = -1;
  std::uint16_t local_port_ = 0;
  std::jthread io_thread_;
  mutable std::mutex clients_mutex_;
  std::map<int, Client> clients_;  // by fd; guarded by clients_mutex_
  std::mutex write_mutex_;         // one writer at a time, so frames don't interleave

  std::atomic<std::uint64_t> frames_tx_{0}, bytes_tx_{0}, frames_rx_{0}, bytes_rx_{0};
  std::atomic<std::uint64_t> tx_dropped_{0}, rx_errors_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__TCP_SERVER_HPP_
