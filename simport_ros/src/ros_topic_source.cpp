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

#include "simport_ros/ros_topic_source.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace simport
{

namespace
{

std::optional<FieldAccessor> try_bind(const MessageMembers & members, const char * path)
{
  try {
    return FieldAccessor::bind(members, parse_field_path(path));
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
}

bool compatible(ValueKind field, ValueKind target)
{
  if (is_numeric(field) && (is_numeric(target) || target == ValueKind::kFloatArray)) {
    return true;  // a single number fills every element of an array field
  }
  return field == target;
}

}  // namespace

RosTopicSource::~RosTopicSource()
{
  stop();
}

void RosTopicSource::configure(
  const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
  const MappingSpec & mapping)
{
  config.expect_keys({"plugin", "topic", "type", "reliability", "depth", "stamp"});
  context_.emplace(context);
  node_ = &node;
  topic_ = config.get<std::string>("topic");
  if (topic_.empty()) {
    config["topic"].error("topic must not be empty");
  }

  const Config type = config.required("type");
  try {
    message_ = std::make_unique<DynamicMessage>(type.as<std::string>());
  } catch (const std::invalid_argument & e) {
    type.error(e.what());
  }

  qos_ = rclcpp::QoS(config.get<std::uint32_t>("depth", 10));
  const std::string reliability = config.get<std::string>("reliability", "best_effort");
  if (reliability == "best_effort") {
    qos_.best_effort();
  } else if (reliability == "reliable") {
    qos_.reliable();
  } else {
    config["reliability"].error("expected 'best_effort' or 'reliable'");
  }

  // Bind every path mapping against the message type.
  const Schema & schema = *mapping.schema();
  bindings_.clear();
  for (const FieldMapping & m : mapping.paths()) {
    const Config where = m.config.is_map() ? m.config["from"] : m.config;
    Binding b{m.target, FieldAccessor{}, m.transform};
    try {
      b.accessor = FieldAccessor::bind(message_->members(), m.path_elements);
    } catch (const std::invalid_argument & e) {
      where.error(e.what());
    }
    const ValueKind target = schema.field(m.target).kind;
    if (!compatible(b.accessor.kind(), target)) {
      where.error(
        "'" + m.path + "' is " + std::string(to_string(b.accessor.kind())) + " but codec field '" +
        m.target_name + "' is " + std::string(to_string(target)));
    }
    const ValueKind source = b.accessor.kind();
    if (!m.transform.identity() && !is_numeric(source) && source != ValueKind::kFloatArray) {
      where.error("'scale'/'offset' need a numeric source field, but '" + m.path + "' is " +
        std::string(to_string(source)));
    }
    if (m.transform.array_ops() && source != ValueKind::kFloatArray) {
      where.error("'transpose', 'reverse', 'step' and 'resize' need an array, but '" + m.path +
        "' is " + std::string(to_string(source)));
    }
    if (m.transform.ramp && source == ValueKind::kFloatArray) {
      where.error("'ramp' makes an array from a number, but '" + m.path + "' is already an array");
    }
    bindings_.push_back(std::move(b));
  }
  prototype_ = mapping.make_record();

  const Config stamp = config["stamp"];
  const std::string stamp_mode = config.get<std::string>("stamp", "auto");
  if (stamp_mode != "auto" && stamp_mode != "header" && stamp_mode != "receive") {
    stamp.error("expected 'auto', 'header' or 'receive'");
  }
  stamp_sec_.reset();
  stamp_nanosec_.reset();
  if (stamp_mode != "receive") {
    stamp_sec_ = try_bind(message_->members(), "header.stamp.sec");
    stamp_nanosec_ = try_bind(message_->members(), "header.stamp.nanosec");
    if ((!stamp_sec_ || !stamp_nanosec_)) {
      stamp_sec_.reset();
      stamp_nanosec_.reset();
      if (stamp_mode == "header") {
        stamp.error(message_->type() + " has no header.stamp");
      }
    }
  }
}

void RosTopicSource::start(RecordCallback callback)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
  }
  subscription_ = node_->create_generic_subscription(
    topic_, message_->type(), qos_,
    [this](std::shared_ptr<rclcpp::SerializedMessage> message) {on_message(*message);});
}

void RosTopicSource::stop()
{
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = nullptr;
}

void RosTopicSource::on_message(const rclcpp::SerializedMessage & serialized)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!callback_) {
    return;
  }
  ++messages_;
  try {
    message_->deserialize(serialized);
  } catch (const std::exception & e) {
    if (deserialize_errors_++ == 0) {
      context_->logger().error("cannot deserialize message on {}: {}", topic_, e.what());
    }
    return;
  }
  const void * msg = message_->data();
  auto record = std::make_shared<Record>(prototype_);
  for (const Binding & b : bindings_) {
    std::optional<Value> v = b.accessor.read(msg);
    if (!v) {
      if (missing_values_++ == 0) {
        context_->logger().warn(
          "array index out of range in a message on {}; field left at its default "
          "(further occurrences counted silently)", topic_);
      }
      continue;
    }
    (*record)[b.target] = b.transform.apply(*v);
  }
  record->stamp = stamp_sec_ ? header_stamp(msg) : context_->clock().now();
  callback_(std::move(record));
}

Timestamp RosTopicSource::header_stamp(const void * message) const
{
  const auto sec = saturate_cast<std::int64_t>(*stamp_sec_->read(message));
  const auto nsec = saturate_cast<std::int64_t>(*stamp_nanosec_->read(message));
  return Timestamp(sec * 1'000'000'000 + nsec);
}

std::string RosTopicSource::describe() const
{
  return topic_ + " [" + (message_ ? message_->type() : std::string("?")) + "]";
}

RosTopicSource::Stats RosTopicSource::stats() const
{
  return Stats{messages_, deserialize_errors_, missing_values_};
}

}  // namespace simport

PLUGINLIB_EXPORT_CLASS(simport::RosTopicSource, simport::Source)
