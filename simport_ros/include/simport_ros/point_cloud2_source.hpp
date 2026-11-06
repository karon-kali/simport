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

#ifndef SIMPORT_ROS__POINT_CLOUD2_SOURCE_HPP_
#define SIMPORT_ROS__POINT_CLOUD2_SOURCE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "simport_ros/source.hpp"

namespace simport
{

/// Subscribes to a sensor_msgs/PointCloud2 and provides its point fields as arrays, in storage
/// order (row by row for organized clouds), e.g. to feed a 3-D lidar's range image.
///
/// ```yaml
/// source:
///   plugin: simport/PointCloud2Source
///   topic: /sim/lidar/points
///   reliability: best_effort   # best_effort (default) | reliable
///   depth: 2
///   stamp: auto                # auto/header (header.stamp) | receive
/// map:
///   range_mm: { from: range, scale: 1000.0, nonfinite: 0 }
///   reflectivity: intensity
/// ```
///
/// Paths are point field names (`x`, `intensity`, `ring`, ...), or values computed from x, y
/// and z: `range` (distance from the origin, m), `azimuth` and `elevation` (rad); and `width` /
/// `height` (numbers). Field names are checked when the first cloud arrives.
class PointCloud2Source : public Source
{
public:
  ~PointCloud2Source() override;

  void configure(
    const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
    const MappingSpec & mapping) override;
  void start(RecordCallback callback) override;
  void stop() override;
  std::string describe() const override;

private:
  enum class Kind : std::uint8_t {kField, kRange, kAzimuth, kElevation, kWidth, kHeight};

  struct Binding
  {
    std::size_t target;
    Kind kind;
    std::string field;  ///< kField
    ValueTransform transform;
  };

  void on_cloud(const sensor_msgs::msg::PointCloud2 & cloud);

  std::optional<Context> context_;
  rclcpp_lifecycle::LifecycleNode * node_ = nullptr;
  std::string topic_;
  rclcpp::QoS qos_{2};
  bool header_stamp_ = true;
  std::vector<Binding> bindings_;
  Record prototype_;

  std::mutex mutex_;  // guards callback_ against stop()
  RecordCallback callback_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::atomic<bool> reported_missing_{false};
};

}  // namespace simport

#endif  // SIMPORT_ROS__POINT_CLOUD2_SOURCE_HPP_
