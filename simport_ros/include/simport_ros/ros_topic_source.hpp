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

#ifndef SIMPORT_ROS__ROS_TOPIC_SOURCE_HPP_
#define SIMPORT_ROS__ROS_TOPIC_SOURCE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/qos.hpp"
#include "simport_ros/dynamic_message.hpp"
#include "simport_ros/source.hpp"

namespace simport
{

/// Subscribes to a topic of any type and maps message fields to codec fields.
///
/// ```yaml
/// source:
///   plugin: simport/RosTopicSource
///   topic: /imu
///   type: sensor_msgs/msg/Imu
///   reliability: best_effort   # best_effort (default; matches any publisher) | reliable
///   depth: 10
///   stamp: auto                # auto (header.stamp if present, else receive time)
///                              # | header | receive
/// ```
class RosTopicSource : public Source
{
public:
  ~RosTopicSource() override;

  void configure(
    const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
    const MappingSpec & mapping) override;
  void start(RecordCallback callback) override;
  void stop() override;
  std::string describe() const override;

  struct Stats
  {
    std::uint64_t messages = 0;
    std::uint64_t deserialize_errors = 0;
    std::uint64_t missing_values = 0;  ///< array index out of range in a message
  };
  Stats stats() const;

private:
  struct Binding
  {
    std::size_t target;
    FieldAccessor accessor;
    ValueTransform transform;
  };

  void on_message(const rclcpp::SerializedMessage & message);
  Timestamp header_stamp(const void * message) const;

  std::optional<Context> context_;
  rclcpp_lifecycle::LifecycleNode * node_ = nullptr;
  std::string topic_;
  rclcpp::QoS qos_{10};
  std::unique_ptr<DynamicMessage> message_;
  std::vector<Binding> bindings_;
  Record prototype_;
  std::optional<FieldAccessor> stamp_sec_;
  std::optional<FieldAccessor> stamp_nanosec_;

  std::mutex mutex_;  // guards callback_ and message_ against stop() and concurrent callbacks
  RecordCallback callback_;
  rclcpp::GenericSubscription::SharedPtr subscription_;

  std::atomic<std::uint64_t> messages_{0};
  std::atomic<std::uint64_t> deserialize_errors_{0};
  std::atomic<std::uint64_t> missing_values_{0};
};

}  // namespace simport

#endif  // SIMPORT_ROS__ROS_TOPIC_SOURCE_HPP_
