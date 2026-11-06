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

#ifndef SIMPORT_CORE__TRANSPORT_HPP_
#define SIMPORT_CORE__TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/frame.hpp"
#include "simport_core/types.hpp"

namespace simport
{

/// Static properties of a transport, used to assemble the pipeline around it.
struct TransportCaps
{
  /// True for byte streams (serial, TCP): frame boundaries are not preserved, so inbound
  /// channels need a Framer. False for message-oriented transports (UDP, CAN).
  bool stream = true;
  /// Largest frame write() accepts; 0 means unlimited.
  std::size_t max_frame_size = 0;
  /// Modelled line rate in bits/s, if any (e.g. serial baud). When set, the device adds a
  /// Bandwidth filter in front of the transport.
  std::optional<double> line_rate_bps;
  /// Bits on the wire per payload byte at `line_rate_bps` (10 for 8N1 serial).
  double bits_per_byte = 8.0;
};

/// Counters; a snapshot is returned by Transport::stats().
struct TransportStats
{
  std::uint64_t frames_tx = 0;
  std::uint64_t bytes_tx = 0;
  std::uint64_t frames_rx = 0;
  std::uint64_t bytes_rx = 0;
  /// Frames not delivered: no peer connected, buffer full, OS error.
  std::uint64_t tx_dropped = 0;
  std::uint64_t rx_errors = 0;
  /// Whether a driver is attached, for transports that can tell (e.g. a pty with its slave
  /// open); unset otherwise.
  std::optional<bool> peer_connected;
};

/// Thrown by Transport::open() when the port cannot be created.
class TransportError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Called by the transport's I/O thread for received data. For stream transports `data` is an
/// arbitrary chunk of the byte stream; for message transports it is one complete message.
using ReceiveCallback =
  std::function<void (ByteView data, const FrameMeta & meta, Timestamp stamp)>;

/// The emulated port: where bytes go and come from.
///
/// Lifecycle: configure() → set_receive_callback() → open() → write()* → close().
class Transport
{
public:
  virtual ~Transport() = default;

  /// Parse and validate configuration. Must not acquire OS resources.
  virtual void configure(const Config & config, Context & context) = 0;

  /// Install the inbound data handler. Called before open(); may be left unset.
  virtual void set_receive_callback(ReceiveCallback callback) = 0;

  /// Create the port (pty, socket, ...) and start receiving. Throws TransportError.
  virtual void open() = 0;

  /// Destroy the port and stop the I/O thread. Idempotent. After return, the receive callback
  /// is no longer invoked.
  virtual void close() = 0;

  virtual bool is_open() const = 0;

  /// Valid after configure().
  virtual TransportCaps caps() const = 0;

  /// Send one frame. Thread-safe and non-throwing; returns false if the frame was dropped.
  /// Must not block for longer than the OS write itself.
  virtual bool write(const Frame & frame) = 0;

  virtual TransportStats stats() const = 0;

  /// Where a driver should connect, e.g. `/tmp/simport/imu0 (pty /dev/pts/7)`.
  virtual std::string describe() const = 0;
};

using TransportPtr = std::shared_ptr<Transport>;

}  // namespace simport

#endif  // SIMPORT_CORE__TRANSPORT_HPP_
