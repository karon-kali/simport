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

#ifndef SIMPORT_PLUGINS__SOCKET_CAN_HPP_
#define SIMPORT_PLUGINS__SOCKET_CAN_HPP_

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "simport_core/transport.hpp"

namespace simport::plugins
{

/// Frame metadata attribute marking a CAN-FD frame ("1").
inline constexpr const char * kCanFdAttribute = "fd";
/// Frame metadata attribute marking an extended (29-bit) identifier ("1").
inline constexpr const char * kCanExtendedAttribute = "extended";

/// Emulated devices on a CAN bus, through Linux SocketCAN (usually a virtual bus).
///
/// ```yaml
/// port:
///   can: vcan0            # one-time setup: sudo modprobe vcan
///                         #   sudo ip link add dev vcan0 type vcan mtu 72   (72: CAN-FD)
///                         #   sudo ip link set up vcan0
///   fd: true              # also receive and send CAN-FD frames
///   bps: 1000000          # optional: model the bus bit rate
/// ```
///
/// Each CAN frame is one packet: the data bytes, with the identifier in `FrameMeta::can_id`
/// (a `can_id: meta` field) and the attributes `fd` and `extended`. Outgoing packets need a
/// `can_id`; they are sent as CAN-FD when marked `fd` (as replies to CAN-FD requests are) or
/// longer than 8 bytes.
class SocketCan : public Transport
{
public:
  ~SocketCan() override;

  void configure(const Config & config, Context & context) override;
  void set_receive_callback(ReceiveCallback callback) override {callback_ = std::move(callback);}
  void open() override;
  void close() override;
  bool is_open() const override {return fd_ >= 0;}
  TransportCaps caps() const override;
  bool write(const Frame & frame) override;
  TransportStats stats() const override;
  std::string describe() const override;

private:
  void rx_loop(std::stop_token stop);

  std::optional<Context> context_;
  std::string interface_;
  bool fd_frames_ = false;

  ReceiveCallback callback_;
  int fd_ = -1;
  std::jthread rx_thread_;

  std::atomic<std::uint64_t> frames_tx_{0}, bytes_tx_{0}, frames_rx_{0}, bytes_rx_{0};
  std::atomic<std::uint64_t> tx_dropped_{0}, rx_errors_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__SOCKET_CAN_HPP_
