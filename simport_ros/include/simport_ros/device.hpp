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

#ifndef SIMPORT_ROS__DEVICE_HPP_
#define SIMPORT_ROS__DEVICE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "simport_core/clock.hpp"
#include "simport_core/device_config.hpp"
#include "simport_core/framer.hpp"
#include "simport_core/input_channel.hpp"
#include "simport_core/logger.hpp"
#include "simport_core/output_channel.hpp"
#include "simport_core/transport.hpp"
#include "simport_msgs/msg/record.hpp"
#include "simport_msgs/srv/exchange.hpp"
#include "simport_ros/behavior.hpp"
#include "simport_ros/source.hpp"

namespace simport
{

/// One emulated device assembled from a DeviceConfig: the port, its `send` messages (ROS ->
/// driver) and `receive` messages (driver -> ROS topic, service, or in-process behavior). Used
/// by DeviceNode and by simport_check.
class Device
{
public:
  struct Send
  {
    SendSpec spec;
    SourcePtr source;  ///< null for reply-only messages
    std::unique_ptr<OutputChannel> channel;
  };

  struct Receive
  {
    ReceiveSpec spec;
    std::size_t index = 0;  ///< position in the device file
    std::unique_ptr<InputChannel> channel;
    rclcpp_lifecycle::LifecyclePublisher<simport_msgs::msg::Record>::SharedPtr publisher;
    rclcpp::Client<simport_msgs::srv::Exchange>::SharedPtr client;
    std::vector<OutputChannel *> replies;  ///< with a service: how responses are encoded
    std::uint64_t unanswered = 0;     ///< service calls without a ready service
  };

  /// `node` must outlive the device; sources and ROS entities are created on it.
  Device(rclcpp_lifecycle::LifecycleNode & node, LoggerPtr logger);
  ~Device();

  Device(const Device &) = delete;
  Device & operator=(const Device &) = delete;

  /// Load plugins and wire everything. With `open_transport`, also create the port. Throws
  /// ConfigError (with location) or TransportError.
  void build(const DeviceConfig & config, bool open_transport = true);

  /// Start data flow in both directions.
  void start();

  /// Stop data flow; the port stays (a silent device). Idempotent.
  void stop();

  /// Stop and destroy the port and all plugins. Idempotent.
  void close();

  const std::string & name() const {return name_;}
  const ClockPtr & clock() const {return clock_;}
  const TransportPtr & transport() const {return transport_;}
  const std::vector<Send> & sends() const {return sends_;}
  const std::vector<std::unique_ptr<Receive>> & receives() const {return receives_;}
  const InboundRouter * router() const {return router_.get();}
  const FramerPtr & framer() const {return framer_;}
  const BehaviorPtr & behavior() const {return behavior_;}

  /// Observe frames in both directions (for a debug topic). Call after build().
  void set_frame_tap(std::function<void(const Frame &, bool outbound, const std::string &)> tap);

private:
  struct Loaders;

  void on_received(Receive & receive, const RecordPtr & record, const Frame & frame);

  rclcpp_lifecycle::LifecycleNode & node_;
  LoggerPtr logger_;
  // Declared before every plugin holder so it is destroyed last: pluginlib requires loaders to
  // outlive the instances they create.
  std::unique_ptr<Loaders> loaders_;
  std::string name_;
  ClockPtr clock_;
  TransportPtr transport_;
  FramerPtr framer_;
  BehaviorPtr behavior_;
  std::vector<Answer> answers_;  ///< scratch for behavior answers (receive thread only)
  std::vector<Send> sends_;
  std::vector<std::unique_ptr<Receive>> receives_;
  std::unique_ptr<InboundRouter> router_;
  std::atomic<bool> running_{false};
};

}  // namespace simport

#endif  // SIMPORT_ROS__DEVICE_HPP_
