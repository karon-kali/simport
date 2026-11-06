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

#include "simport_core/input_channel.hpp"

#include <algorithm>
#include <format>
#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace simport
{

bool values_equal(const Value & actual, const Value & expected)
{
  const ValueKind a = kind_of(actual);
  const ValueKind e = kind_of(expected);
  if (is_numeric(a) && is_numeric(e)) {
    const auto as_int = [](const Value & v) -> std::optional<std::int64_t> {
        if (kind_of(v) == ValueKind::kInt) {return std::get<std::int64_t>(v);}
        if (kind_of(v) == ValueKind::kBool) {return std::get<bool>(v) ? 1 : 0;}
        return std::nullopt;
      };
    const auto as_uint = [](const Value & v) -> std::optional<std::uint64_t> {
        return kind_of(v) == ValueKind::kUint ? std::optional(std::get<std::uint64_t>(v)) :
               std::nullopt;
      };
    // Exact comparison between integers of either signedness.
    if (as_int(actual) && as_int(expected)) {return *as_int(actual) == *as_int(expected);}
    if (as_uint(actual) && as_uint(expected)) {return *as_uint(actual) == *as_uint(expected);}
    if (as_int(actual) && as_uint(expected)) {
      return std::cmp_equal(*as_int(actual), *as_uint(expected));
    }
    if (as_uint(actual) && as_int(expected)) {
      return std::cmp_equal(*as_uint(actual), *as_int(expected));
    }
    return to_double(actual) == to_double(expected);
  }
  return actual == expected;
}

bool FieldCondition::matches(const Value & actual) const
{
  return std::any_of(expected.begin(), expected.end(),
           [&](const Value & e) {return values_equal(actual, e);});
}

std::vector<FieldCondition> parse_conditions(const Config & match, const Schema & schema)
{
  std::vector<FieldCondition> out;
  for (const auto & [name, node] : match.entries()) {
    const auto index = schema.find(name);
    if (!index) {
      std::string fields;
      for (const auto & n : schema.names()) {
        fields += " " + n;
      }
      node.error("the input codec has no field '" + name + "'; fields:" + fields);
    }
    FieldCondition condition{*index, {}};
    const std::vector<Config> values = node.is_sequence() ? node.items() :
      std::vector<Config>{node};
    if (values.empty()) {
      node.error("give at least one value to match");
    }
    for (const Config & v : values) {
      switch (schema.field(*index).kind) {
        case ValueKind::kBool: condition.expected.emplace_back(v.as<bool>()); break;
        case ValueKind::kInt: condition.expected.emplace_back(v.as<std::int64_t>()); break;
        case ValueKind::kUint: condition.expected.emplace_back(v.as<std::uint64_t>()); break;
        case ValueKind::kFloat: condition.expected.emplace_back(v.as<double>()); break;
        case ValueKind::kString: condition.expected.emplace_back(v.as<std::string>()); break;
        default: node.error("matching is supported for numbers, booleans and strings only");
      }
    }
    out.push_back(std::move(condition));
  }
  return out;
}

// -- InputChannel ------------------------------------------------------------------------------

InputChannel::InputChannel(
  std::string name, Context context, CodecPtr codec, std::vector<FieldCondition> match,
  RecordHandler handler)
: name_(std::move(name)), context_(std::move(context)), codec_(std::move(codec)),
  match_(std::move(match)), handler_(std::move(handler)) {}

bool InputChannel::handle(const Frame & frame)
{
  auto record = std::make_shared<Record>(Record::make(codec_->schema()));
  if (!codec_->decode(frame, *record).ok()) {
    return false;
  }
  for (const FieldCondition & c : match_) {
    if (!c.matches((*record)[c.field])) {
      return false;
    }
  }
  ++frames_;
  context_.logger().debug("received {}", name_);
  if (handler_) {
    handler_(record, frame);
  }
  return true;
}

// -- InboundRouter -----------------------------------------------------------------------------

InboundRouter::InboundRouter(
  Context context, FramerFactory make_framer, std::vector<InputChannel *> inputs)
: context_(std::move(context)), make_framer_(std::move(make_framer)),
  inputs_(std::move(inputs)) {}

InboundRouter::InboundRouter(
  Context context, FramerPtr framer, std::vector<InputChannel *> inputs)
: InboundRouter(
    std::move(context),
    framer ? FramerFactory([framer] {return framer;}) : FramerFactory(), std::move(inputs)) {}

void InboundRouter::set_active(bool active)
{
  if (active) {
    reset_pending_ = true;  // reset on the receive thread, which owns the framers
  }
  active_ = active;
}

void InboundRouter::on_receive(ByteView data, const FrameMeta & meta, Timestamp stamp)
{
  const std::string peer =
    meta.endpoint ? meta.endpoint->host + ":" + std::to_string(meta.endpoint->port) : "";
  if (meta.attributes.contains(kDisconnectedAttribute)) {
    framers_.erase(peer);
    return;
  }
  if (!active_) {
    return;
  }
  if (!make_framer_) {
    dispatch(Frame{Bytes(data.begin(), data.end()), meta, stamp});
    return;
  }
  if (reset_pending_.exchange(false)) {
    for (auto & entry : framers_) {
      entry.second->reset();
    }
  }
  FramerPtr & framer = framers_[peer];
  if (!framer) {
    framer = make_framer_();
    framer->reset();
  }
  framer->feed(
    data, stamp, [&](Frame && frame) {
      frame.meta = meta;
      dispatch(std::move(frame));
    });
}

void InboundRouter::dispatch(Frame && frame)
{
  ++frames_;
  if (tap_) {
    tap_(frame);
  }
  for (InputChannel * input : inputs_) {
    if (input->handle(frame)) {
      return;
    }
  }
  if (unmatched_++ == 0) {
    std::string hex;
    for (std::size_t i = 0; i < std::min<std::size_t>(frame.data.size(), 32); ++i) {
      hex += std::format("{:02x} ", frame.data[i]);
    }
    context_.logger().warn(
      "received a frame that no input matches ({} bytes: {}{}); further ones are counted "
      "silently", frame.data.size(), hex, frame.data.size() > 32 ? "..." : "");
  }
}

}  // namespace simport
