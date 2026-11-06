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

#ifndef SIMPORT_CORE__INPUT_CHANNEL_HPP_
#define SIMPORT_CORE__INPUT_CHANNEL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/codec.hpp"
#include "simport_core/context.hpp"
#include "simport_core/framer.hpp"
#include "simport_core/output_channel.hpp"

namespace simport
{

/// A decoded field that must equal a value (or one of several) for a receive to claim a
/// frame.
struct FieldCondition
{
  std::size_t field;            ///< index in the codec's schema
  std::vector<Value> expected;  ///< any of these

  bool matches(const Value & actual) const;
};

/// True if `actual` equals `expected`: numerically for numeric kinds (exact for integers),
/// exactly otherwise.
bool values_equal(const Value & actual, const Value & expected);

/// Parse a `match:` block (`field: value` or `field: [value, ...]`) against a codec schema.
/// Throws ConfigError.
std::vector<FieldCondition> parse_conditions(const Config & match, const Schema & schema);

/// Receives a decoded record together with the frame it came from (for its metadata, e.g.
/// the sender to reply to).
using RecordHandler = std::function<void (const RecordPtr & record, const Frame & frame)>;

struct InputChannelStats
{
  std::uint64_t frames = 0;  ///< frames this input claimed
};

/// One `receive:` message: recognises one kind of frame sent by the driver and hands the
/// decoded record to its handler (a ROS topic or service). It never answers by itself.
class InputChannel
{
public:
  InputChannel(
    std::string name, Context context, CodecPtr codec, std::vector<FieldCondition> match,
    RecordHandler handler);

  const std::string & name() const {return name_;}
  const CodecPtr & codec() const {return codec_;}

  /// Claim and handle `frame` if it decodes with this input's codec and meets the match
  /// conditions. Called from the transport's receive thread.
  bool handle(const Frame & frame);

  InputChannelStats stats() const {return InputChannelStats{frames_};}

private:
  std::string name_;
  Context context_;
  CodecPtr codec_;
  std::vector<FieldCondition> match_;
  RecordHandler handler_;
  std::atomic<std::uint64_t> frames_{0};
};

struct InboundStats
{
  std::uint64_t frames = 0;     ///< frames after framing
  std::uint64_t unmatched = 0;  ///< frames no input claimed
};

/// Creates a framer for one inbound stream.
using FramerFactory = std::function<FramerPtr()>;

/// Metadata attribute a stream transport sets, with empty data, when a peer disconnects.
inline constexpr const char * kDisconnectedAttribute = "disconnected";

/// Device-level receive path: transport bytes → Framer (stream transports) → first InputChannel
/// that claims the frame. Ignores data while inactive.
///
/// Stream transports with several peers (TCP) tag received data with the peer's endpoint;
/// each peer then gets its own framer, so interleaved streams are framed independently.
class InboundRouter
{
public:
  /// `make_framer` may be empty for message-oriented transports.
  InboundRouter(Context context, FramerFactory make_framer, std::vector<InputChannel *> inputs);

  /// One framer instance shared by all peers (for transports with a single stream).
  InboundRouter(Context context, FramerPtr framer, std::vector<InputChannel *> inputs);

  /// Start (resetting the framer) or stop handling received data.
  void set_active(bool active);

  /// Observe every received frame, matched or not (e.g. for a debug topic).
  void set_frame_tap(std::function<void(const Frame &)> tap) {tap_ = std::move(tap);}

  /// The transport's receive callback.
  void on_receive(ByteView data, const FrameMeta & meta, Timestamp stamp);

  InboundStats stats() const {return InboundStats{frames_, unmatched_};}

private:
  void dispatch(Frame && frame);

  Context context_;
  FramerFactory make_framer_;
  std::map<std::string, FramerPtr> framers_;  ///< by peer; used by the receive thread only
  std::vector<InputChannel *> inputs_;
  std::function<void(const Frame &)> tap_;
  std::atomic<bool> active_{false};
  std::atomic<bool> reset_pending_{false};
  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::uint64_t> unmatched_{0};
};

}  // namespace simport

#endif  // SIMPORT_CORE__INPUT_CHANNEL_HPP_
