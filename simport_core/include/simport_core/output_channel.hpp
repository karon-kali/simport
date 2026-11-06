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

#ifndef SIMPORT_CORE__OUTPUT_CHANNEL_HPP_
#define SIMPORT_CORE__OUTPUT_CHANNEL_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/codec.hpp"
#include "simport_core/context.hpp"
#include "simport_core/frame_filter.hpp"
#include "simport_core/mapping.hpp"
#include "simport_core/transport.hpp"
#include "simport_core/trigger.hpp"

namespace simport
{

/// Counters of an OutputChannel.
struct OutputChannelStats
{
  std::uint64_t records_in = 0;    ///< records pushed by the source
  std::uint64_t emitted = 0;       ///< records encoded (trigger emits and requests)
  std::uint64_t requests = 0;      ///< sends requested as replies
  std::uint64_t frames = 0;        ///< frames handed to the transport
  std::uint64_t bytes = 0;
  std::uint64_t dropped = 0;       ///< frames the transport did not accept
  std::uint64_t encode_errors = 0;
};

/// Field values to set in a requested send, by schema index.
using FieldOverrides = std::vector<std::pair<std::size_t, Value>>;

/// The outbound pipeline of one `send:` message, minus the source:
///
///   push() → Trigger → [builtins] → Codec::encode → FrameFilter chain → Transport
///
/// request() bypasses the trigger, sending once immediately (replies from ROS).
/// ROS-agnostic; the host owns the source and calls push(). All plugins must already be
/// configured; the transport must outlive the channel.
class OutputChannel
{
public:
  OutputChannel(
    std::string name, Context context, MappingSpec mapping, TriggerPtr trigger, CodecPtr codec,
    std::vector<FrameFilterPtr> filters, TransportPtr transport);
  ~OutputChannel();

  OutputChannel(const OutputChannel &) = delete;
  OutputChannel & operator=(const OutputChannel &) = delete;

  const std::string & name() const {return name_;}
  const MappingSpec & mapping() const {return mapping_;}
  const SchemaPtr & schema() const {return mapping_.schema();}
  const CodecPtr & codec() const {return codec_;}

  /// Reset the codec, start filters and trigger.
  void start();

  /// Stop trigger and filters. After return nothing is written. Idempotent.
  void stop();

  bool running() const {return running_;}

  /// Feed a record from the source. Thread-safe. Ignored while stopped.
  void push(RecordPtr record);

  /// Send once now, from the latest source record (or defaults if there is none yet) with
  /// `overrides` applied. Frames carry `meta` (e.g. the requester's endpoint).
  /// Thread-safe; ignored while stopped.
  void request(const FieldOverrides & overrides, const FrameMeta & meta = {});

  /// Observe every frame handed to the transport (e.g. for a debug topic). Set before start().
  void set_frame_tap(std::function<void(const Frame &)> tap) {tap_ = std::move(tap);}

  OutputChannelStats stats() const;

private:
  void emit(const RecordPtr & record, const FieldOverrides * overrides, const FrameMeta * meta);
  void write(Frame && frame);

  std::string name_;
  Context context_;
  MappingSpec mapping_;
  TriggerPtr trigger_;
  CodecPtr codec_;
  std::vector<FrameFilterPtr> filters_;
  TransportPtr transport_;
  /// sinks_[i] feeds filters_[i]; sinks_.back() writes to the transport.
  std::vector<FrameSink> sinks_;
  std::function<void(const Frame &)> tap_;

  std::atomic<bool> running_{false};
  std::mutex emit_mutex_;  // serializes trigger emits and requests (codec state, scratch)
  std::vector<Frame> scratch_frames_;
  std::mutex latest_mutex_;
  RecordPtr latest_;

  std::atomic<std::uint64_t> records_in_{0};
  std::atomic<std::uint64_t> emitted_{0};
  std::atomic<std::uint64_t> requests_{0};
  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::uint64_t> bytes_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> encode_errors_{0};
};

}  // namespace simport

#endif  // SIMPORT_CORE__OUTPUT_CHANNEL_HPP_
