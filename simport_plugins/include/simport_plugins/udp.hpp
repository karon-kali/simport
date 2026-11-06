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

#ifndef SIMPORT_PLUGINS__UDP_HPP_
#define SIMPORT_PLUGINS__UDP_HPP_

#include <atomic>
#include <mutex>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "simport_core/transport.hpp"
#include "simport_plugins/net.hpp"

namespace simport::plugins
{

/// Emulated network device speaking UDP.
///
/// ```yaml
/// port:
///   udp: 127.0.0.2:2369     # the device's own address; any 127.x.y.z works on Linux without
///                           # setup, so drivers that filter on the sensor's source IP work
///   to: 127.0.0.1:2368      # where sent packets go; `sender` = whoever sent the last
///                           # datagram (e.g. the driver that sent "start streaming");
///                           # omitted = replies only
///   broadcast: false        # allow a broadcast destination
///   bps: 100000000          # optional: pace packets at this line rate
/// ```
///
/// Each datagram is one packet, so receiving needs no framer. Replies go back to the sender of
/// the request instead of `to`.
class Udp : public Transport
{
public:
  ~Udp() override;

  void configure(const Config & config, Context & context) override;
  void set_receive_callback(ReceiveCallback callback) override {callback_ = std::move(callback);}
  void open() override;
  void close() override;
  bool is_open() const override {return fd_ >= 0;}
  TransportCaps caps() const override;
  bool write(const Frame & frame) override;
  TransportStats stats() const override;
  std::string describe() const override;

  /// Bound port (useful when binding to port 0). 0 when closed.
  std::uint16_t local_port() const {return local_port_;}

private:
  void rx_loop(std::stop_token stop);

  std::optional<Context> context_;
  Endpoint bind_{"0.0.0.0", 0};
  std::optional<Endpoint> destination_;  // guarded by destination_mutex_
  bool learn_destination_ = false;
  mutable std::mutex destination_mutex_;
  bool broadcast_ = false;

  ReceiveCallback callback_;
  int fd_ = -1;
  std::uint16_t local_port_ = 0;
  std::jthread rx_thread_;

  std::atomic<std::uint64_t> frames_tx_{0}, bytes_tx_{0}, frames_rx_{0}, bytes_rx_{0};
  std::atomic<std::uint64_t> tx_dropped_{0}, rx_errors_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__UDP_HPP_
