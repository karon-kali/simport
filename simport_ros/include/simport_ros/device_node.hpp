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

#ifndef SIMPORT_ROS__DEVICE_NODE_HPP_
#define SIMPORT_ROS__DEVICE_NODE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "simport_msgs/msg/frame.hpp"
#include "simport_ros/device.hpp"

namespace simport
{

/// One emulated device, assembled from a device YAML file.
///
/// Parameters:
/// - `config_file` (string, required): path to the device YAML.
/// - `autostart` (bool, default true): configure and activate on startup.
/// - `debug_frames` (bool, default false): publish every frame, both directions, on
///   `~/frames` (simport_msgs/Frame).
///
/// Publishes status on /diagnostics: one entry per output, input and the transport.
///
/// Lifecycle: configure creates the port, activate starts data flow, deactivate stops it
/// (the port stays, like a silent device), cleanup removes the port (device unplugged).
class DeviceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit DeviceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DeviceNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  void teardown();
  void log_stats();
  void add_diagnostics();
  void remove_diagnostics();

  std::unique_ptr<Device> device_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::vector<std::string> diagnostic_names_;
  /// Last (time, frames) per diagnostic entry, for rate computation.
  std::map<std::string, std::pair<std::chrono::steady_clock::time_point, std::uint64_t>>
  rate_state_;
  rclcpp::Publisher<simport_msgs::msg::Frame>::SharedPtr frames_publisher_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace simport

#endif  // SIMPORT_ROS__DEVICE_NODE_HPP_
