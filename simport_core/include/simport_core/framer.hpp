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

#ifndef SIMPORT_CORE__FRAMER_HPP_
#define SIMPORT_CORE__FRAMER_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/frame.hpp"
#include "simport_core/types.hpp"

namespace simport
{

struct FramerStats
{
  std::uint64_t frames = 0;
  /// Bytes skipped while searching for a frame start, or dropped from oversized frames.
  std::uint64_t discarded_bytes = 0;
};

using FrameCallback = std::function<void (Frame && frame)>;

/// Splits an inbound byte stream into frames. Only used with stream transports; for
/// message-oriented transports each received message is already a frame.
///
/// A framer belongs to one inbound channel and is called from the transport's I/O thread only.
class Framer
{
public:
  virtual ~Framer() = default;

  virtual void configure(const Config & config, Context & context) = 0;

  /// Consume a chunk of the stream, calling `on_frame` for each completed frame. Incomplete
  /// trailing data is retained for the next call. `stamp` is the receive time of this chunk.
  virtual void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) = 0;

  /// Drop any buffered partial frame.
  virtual void reset() = 0;

  virtual FramerStats stats() const = 0;
};

using FramerPtr = std::shared_ptr<Framer>;

}  // namespace simport

#endif  // SIMPORT_CORE__FRAMER_HPP_
