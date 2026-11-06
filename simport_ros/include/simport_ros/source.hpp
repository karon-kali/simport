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

#ifndef SIMPORT_ROS__SOURCE_HPP_
#define SIMPORT_ROS__SOURCE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/mapping.hpp"
#include "simport_core/schema.hpp"

namespace simport
{

using RecordCallback = std::function<void (RecordPtr record)>;

/// Where a send message's data comes from: subscribes to something and turns each incoming
/// sample into a Record following the codec schema, using the path mappings of `mapping`.
/// Constants are pre-filled by MappingSpec::make_record(); builtins are applied later by the
/// channel.
///
/// Lifecycle: configure() → start() → stop() (start/stop may repeat).
class Source
{
public:
  virtual ~Source() = default;

  /// Resolve the mapping against the source's data type. Report problems with
  /// `mapping.paths()[i].config.error(...)` or `config.error(...)`. `node` outlives the source.
  virtual void configure(
    const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
    const MappingSpec & mapping) = 0;

  /// Begin delivering records to `callback` (from any thread, one at a time).
  virtual void start(RecordCallback callback) = 0;

  /// After return, `callback` is not running and will not be called again.
  virtual void stop() = 0;

  /// e.g. `/imu [sensor_msgs/msg/Imu]`.
  virtual std::string describe() const = 0;
};

using SourcePtr = std::shared_ptr<Source>;

}  // namespace simport

#endif  // SIMPORT_ROS__SOURCE_HPP_
