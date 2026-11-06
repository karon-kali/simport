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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simport_ros/dynamic_message.hpp"

namespace simport
{
namespace
{

template<typename MsgT>
void load(DynamicMessage & dyn, const MsgT & msg)
{
  rclcpp::Serialization<MsgT> ser;
  rclcpp::SerializedMessage out;
  ser.serialize_message(&msg, &out);
  dyn.deserialize(out);
}

FieldAccessor bind(const DynamicMessage & dyn, const std::string & path)
{
  return FieldAccessor::bind(dyn.members(), parse_field_path(path));
}

std::string bind_error(const DynamicMessage & dyn, const std::string & path)
{
  try {
    bind(dyn, path);
  } catch (const std::invalid_argument & e) {
    return e.what();
  }
  ADD_FAILURE() << "expected bind failure for " << path;
  return {};
}

TEST(DynamicMessage, ReadsImuFields)
{
  DynamicMessage dyn("sensor_msgs/Imu");  // short form accepted
  EXPECT_EQ(dyn.type(), "sensor_msgs/msg/Imu");
  EXPECT_EQ(type_name(dyn.members()), "sensor_msgs/msg/Imu");

  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 12;
  msg.header.stamp.nanosec = 345;
  msg.header.frame_id = "imu_link";
  msg.angular_velocity.x = 0.25;
  msg.linear_acceleration.z = -9.81;
  msg.orientation_covariance[4] = 7.0;
  load(dyn, msg);

  const auto gx = bind(dyn, "angular_velocity.x");
  EXPECT_EQ(gx.kind(), ValueKind::kFloat);
  EXPECT_DOUBLE_EQ(std::get<double>(*gx.read(dyn.data())), 0.25);
  EXPECT_DOUBLE_EQ(
    std::get<double>(*bind(dyn, "linear_acceleration.z").read(dyn.data())), -9.81);

  const auto sec = bind(dyn, "header.stamp.sec");
  EXPECT_EQ(sec.kind(), ValueKind::kInt);
  EXPECT_EQ(std::get<std::int64_t>(*sec.read(dyn.data())), 12);
  EXPECT_EQ(std::get<std::uint64_t>(*bind(dyn, "header.stamp.nanosec").read(dyn.data())), 345u);
  EXPECT_EQ(std::get<std::string>(*bind(dyn, "header.frame_id").read(dyn.data())), "imu_link");

  // Fixed-size array: element and whole array.
  EXPECT_DOUBLE_EQ(
    std::get<double>(*bind(dyn, "orientation_covariance[4]").read(dyn.data())), 7.0);
  const auto cov = bind(dyn, "orientation_covariance");
  EXPECT_EQ(cov.kind(), ValueKind::kFloatArray);
  const auto all = std::get<std::vector<double>>(*cov.read(dyn.data()));
  ASSERT_EQ(all.size(), 9u);
  EXPECT_DOUBLE_EQ(all[4], 7.0);
}

TEST(DynamicMessage, DynamicArrays)
{
  DynamicMessage dyn("sensor_msgs/msg/LaserScan");
  sensor_msgs::msg::LaserScan msg;
  msg.ranges = {1.5f, 2.5f, 3.5f};
  load(dyn, msg);
  EXPECT_DOUBLE_EQ(std::get<double>(*bind(dyn, "ranges[2]").read(dyn.data())), 3.5);
  // Out of range in this message: no value, not an error.
  EXPECT_FALSE(bind(dyn, "ranges[3]").read(dyn.data()).has_value());
  EXPECT_EQ(std::get<std::vector<double>>(*bind(dyn, "ranges").read(dyn.data())).size(), 3u);

  // Values change with each deserialized message.
  msg.ranges = {9.0f};
  load(dyn, msg);
  EXPECT_DOUBLE_EQ(std::get<double>(*bind(dyn, "ranges[0]").read(dyn.data())), 9.0);
  EXPECT_FALSE(bind(dyn, "ranges[2]").read(dyn.data()).has_value());
}

TEST(DynamicMessage, StringArrays)
{
  DynamicMessage dyn("sensor_msgs/msg/JointState");
  sensor_msgs::msg::JointState msg;
  msg.name = {"left", "right"};
  msg.position = {0.1, 0.2};
  load(dyn, msg);
  EXPECT_EQ(std::get<std::string>(*bind(dyn, "name[1]").read(dyn.data())), "right");
  EXPECT_DOUBLE_EQ(std::get<double>(*bind(dyn, "position[1]").read(dyn.data())), 0.2);
}

TEST(DynamicMessage, BindErrors)
{
  DynamicMessage dyn("sensor_msgs/msg/Imu");
  EXPECT_NE(bind_error(dyn, "angular_velocty.x").find(
      "no field 'angular_velocty' in sensor_msgs/msg/Imu; fields: header, orientation,"),
    std::string::npos);
  EXPECT_NE(bind_error(dyn, "angular_velocity.w").find(
      "no field 'w' in angular_velocity (geometry_msgs/msg/Vector3); fields: x, y, z"),
    std::string::npos);
  EXPECT_NE(bind_error(dyn, "angular_velocity").find(
      "'angular_velocity' is a message (geometry_msgs/msg/Vector3); "
      "map one of its fields: x, y, z"),
    std::string::npos);
  EXPECT_NE(bind_error(dyn, "angular_velocity[0].x").find("is not an array"), std::string::npos);
  EXPECT_NE(bind_error(dyn, "orientation_covariance[9]").find("out of range"),
    std::string::npos);
  EXPECT_NE(bind_error(dyn, "header.frame_id.x").find("is not a message"), std::string::npos);

  DynamicMessage js("sensor_msgs/msg/JointState");
  EXPECT_NE(bind_error(js, "name").find("non-numeric array"), std::string::npos);
}

TEST(DynamicMessage, UnknownType)
{
  EXPECT_THROW(DynamicMessage("no_such_pkg/msg/Thing"), std::invalid_argument);
  EXPECT_THROW(DynamicMessage("Imu"), std::invalid_argument);
}

}  // namespace
}  // namespace simport
