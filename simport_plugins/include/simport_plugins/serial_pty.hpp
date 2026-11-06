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

#ifndef SIMPORT_PLUGINS__SERIAL_PTY_HPP_
#define SIMPORT_PLUGINS__SERIAL_PTY_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "simport_core/transport.hpp"

namespace simport::plugins
{

/// Serial line framing such as `8N1`.
struct SerialFraming
{
  int data_bits = 8;
  char parity = 'N';  ///< N, E or O
  int stop_bits = 1;

  /// Bits on the wire per byte: start + data + parity + stop.
  double bits_per_byte() const {return 1.0 + data_bits + (parity == 'N' ? 0 : 1) + stop_bits;}

  /// Parse `8N1`, `7E2`, ... Throws std::invalid_argument.
  static SerialFraming parse(const std::string & text);
};

/// Emulated serial port: a pseudo-terminal whose slave end is exposed at a stable path.
///
/// ```yaml
/// port:
///   serial: /tmp/simport/imu0 # symlink to the pty slave; the driver opens this
///   baud: 115200              # line rate: output is paced and capped to what fits
///   framing: 8N1
///   throttle: true            # false: ignore the line rate
///   max_delay: 0.1            # s a packet may wait for the line before it is dropped
/// ```
///
/// Like a real port with nothing attached, frames written while no driver has the port open
/// are dropped (counted in tx_dropped) instead of being buffered and delivered stale later.
/// If the driver configures the port for a different baud rate or framing than the device,
/// a warning is logged: on real hardware that combination would only produce garbage.
class SerialPty : public Transport
{
public:
  ~SerialPty() override;

  void configure(const Config & config, Context & context) override;
  void set_receive_callback(ReceiveCallback callback) override;
  void open() override;
  void close() override;
  bool is_open() const override {return master_fd_ >= 0;}
  TransportCaps caps() const override;
  bool write(const Frame & frame) override;
  TransportStats stats() const override;
  std::string describe() const override;

  /// True while a driver has the slave end open.
  bool peer_connected() const {return connected_;}
  /// Path of the pty slave device, e.g. `/dev/pts/7`. Empty when closed.
  const std::string & slave_name() const {return slave_name_;}

private:
  void rx_loop(std::stop_token stop);
  void set_connected(bool connected);
  void check_line_settings();

  std::optional<Context> context_;
  std::filesystem::path link_;
  std::uint32_t baud_ = 115200;
  bool throttle_ = true;
  SerialFraming framing_;

  ReceiveCallback callback_;
  int master_fd_ = -1;
  std::string slave_name_;
  std::jthread rx_thread_;
  std::mutex write_mutex_;
  std::atomic<bool> connected_{false};
  bool line_warned_ = false;  // rx thread only
  std::chrono::steady_clock::time_point next_line_check_{};

  std::atomic<std::uint64_t> frames_tx_{0}, bytes_tx_{0}, frames_rx_{0}, bytes_rx_{0};
  std::atomic<std::uint64_t> tx_dropped_{0}, rx_errors_{0};
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__SERIAL_PTY_HPP_
