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

#ifndef SIMPORT_ROS__BEHAVIOR_HPP_
#define SIMPORT_ROS__BEHAVIOR_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/output_channel.hpp"
#include "simport_core/schema.hpp"

namespace simport
{

/// A send or receive message of the device, as a behavior sees it.
struct MessageInfo
{
  std::string name;
  SchemaPtr schema;
};

/// The device's messages, in file order. Indices into these identify messages in
/// Behavior::handle() and Answer.
struct DeviceMessages
{
  std::vector<MessageInfo> sends;
  std::vector<MessageInfo> receives;

  /// Index of the message named `name`; throws ConfigError at `where` if there is none.
  std::size_t send(const std::string & name, const Config & where) const;
  std::size_t receive(const std::string & name, const Config & where) const;
};

/// A packet to send in answer: the send message (index into DeviceMessages::sends) and the
/// field values to set (by index into that message's schema). Constants, counters, lengths
/// and checksums are filled in by the codec as usual.
struct Answer
{
  std::size_t send = 0;
  FieldOverrides values;
};

/// What a device decides, in the device's own process: the fast alternative to a service.
///
/// ```yaml
/// behavior: { plugin: my_pkg/MyBus, ... }    # this plugin's own keys
/// receive:
///   - name: command
///     fields: [...]
///     to: behavior                           # handle() gets every decoded packet
/// ```
///
/// Lifecycle: configure() → start() → handle()* → stop() (start/stop may repeat).
class Behavior
{
public:
  virtual ~Behavior() = default;

  /// Read the `behavior:` block and bind message and field names to indices. `node` is the
  /// device's node, for the plugin's own subscriptions and publishers (e.g. simulated joints).
  virtual void configure(
    const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
    const DeviceMessages & messages) = 0;

  /// Data flow starts / stops.
  virtual void start() {}
  virtual void stop() {}

  /// A packet decoded by receive `receive` (index into DeviceMessages::receives). Append the
  /// packets to send back to `answers` (it arrives empty); they go to whoever sent the packet.
  /// Called from the port's receive thread, one packet at a time; ROS callbacks of the plugin
  /// run on another thread, so guard shared state.
  virtual void handle(
    std::size_t receive, const Record & packet,
    std::vector<Answer> & answers) = 0;

  /// One line for logs, e.g. "Damiao bus: 7 motors".
  virtual std::string describe() const {return {};}
};

using BehaviorPtr = std::shared_ptr<Behavior>;

}  // namespace simport

#endif  // SIMPORT_ROS__BEHAVIOR_HPP_
