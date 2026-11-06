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

#ifndef SIMPORT_CORE__FRAME_HPP_
#define SIMPORT_CORE__FRAME_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "simport_core/types.hpp"

namespace simport
{

/// Network endpoint for datagram transports.
struct Endpoint
{
  std::string host;
  std::uint16_t port = 0;

  bool operator==(const Endpoint &) const = default;
};

/// Transport-level metadata attached to a frame. Transports ignore fields they don't use.
struct FrameMeta
{
  /// Datagram transports: destination (outbound) or sender (inbound). Unset means "default".
  std::optional<Endpoint> endpoint;
  /// Bus transports (CAN): arbitration id.
  std::optional<std::uint32_t> can_id;
  /// Free-form extension point for transport-specific metadata.
  std::map<std::string, std::string> attributes;

  bool operator==(const FrameMeta &) const = default;
};

/// One unit of bytes on the wire: a packet, a datagram, a CAN frame, a line of text, ...
struct Frame
{
  Bytes data;
  FrameMeta meta;
  /// When the frame was produced (outbound) or received (inbound), on the device clock.
  Timestamp stamp{};
};

}  // namespace simport

#endif  // SIMPORT_CORE__FRAME_HPP_
