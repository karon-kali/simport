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

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "simport_ros/ros_clock.hpp"
#include "simport_ros/ros_topic_source.hpp"

using namespace std::chrono_literals;

namespace simport
{
namespace
{

class RosTopicSourceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("source_test");
    schema_ = std::make_shared<const Schema>(std::vector<FieldSpec>{
          {"sync", ValueKind::kUint, FieldRole::kComputed},
          {"gx", ValueKind::kInt, FieldRole::kInput},
          {"az", ValueKind::kFloat, FieldRole::kInput},
          {"id", ValueKind::kUint, FieldRole::kInput},
      });
    ctx_.emplace(
      "dev", std::make_shared<NullLogger>(), std::make_shared<RosClock>(node_->get_clock()));
  }

  MappingSpec mapping(const std::string & yaml)
  {
    // `map:` holds field descriptions, as a codec reports them.
    return MappingSpec::from_sources(Config::from_string(yaml, "map.yaml")["map"].entries(),
             schema_);
  }

  std::string configure_error(const std::string & source_yaml, const std::string & map_yaml)
  {
    RosTopicSource source;
    try {
      source.configure(Config::from_string(source_yaml, "src.yaml"), *ctx_, *node_,
        mapping(map_yaml));
    } catch (const ConfigError & e) {
      return e.what();
    }
    ADD_FAILURE() << "expected ConfigError";
    return {};
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  SchemaPtr schema_;
  std::optional<Context> ctx_;
};

TEST_F(RosTopicSourceTest, MapsPublishedMessagesToRecords)
{
  RosTopicSource source;
  const MappingSpec m = mapping(
    "map:\n"
    "  gx: { from: angular_velocity.x, scale: 1000.0 }\n"
    "  az: linear_acceleration.z\n");
  source.configure(
    Config::from_string("topic: /test_imu\ntype: sensor_msgs/msg/Imu\nreliability: reliable\n"),
    *ctx_, *node_, m);
  EXPECT_EQ(source.describe(), "/test_imu [sensor_msgs/msg/Imu]");

  std::mutex mutex;
  std::vector<RecordPtr> records;
  source.start([&](RecordPtr r) {
      std::lock_guard<std::mutex> lock(mutex);
      records.push_back(std::move(r));
    });

  auto pub_node = std::make_shared<rclcpp::Node>("pub");
  auto pub = pub_node->create_publisher<sensor_msgs::msg::Imu>("/test_imu", rclcpp::QoS(10));
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 3;
  msg.header.stamp.nanosec = 500;
  msg.angular_velocity.x = 0.0125;
  msg.linear_acceleration.z = 9.81;

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_->get_node_base_interface());
  exec.add_node(pub_node);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    exec.spin_some(50ms);
    std::lock_guard<std::mutex> lock(mutex);
    if (!records.empty()) {
      break;
    }
  }
  source.stop();

  ASSERT_FALSE(records.empty());
  const Record & r = *records.front();
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("gx")), 12.5);
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("az")), 9.81);
  EXPECT_EQ(std::get<std::uint64_t>(r.at("id")), 0u);  // unmapped: the default
  EXPECT_EQ(r.stamp, Timestamp(3'000'000'500));  // header.stamp
  EXPECT_GE(source.stats().messages, 1u);
}

TEST_F(RosTopicSourceTest, ConfigErrors)
{
  const std::string imu = "topic: /t\ntype: sensor_msgs/msg/Imu\n";
  EXPECT_NE(configure_error(imu, "map: {gx: angular_velocity.q}\n")
    .find("map.yaml:1:11: map.gx: no field 'q' in angular_velocity"), std::string::npos);
  EXPECT_NE(configure_error(imu, "map: {gx: {from: header.frame_id}}\n")
    .find("'header.frame_id' is string but codec field 'gx' is int"), std::string::npos);
  EXPECT_NE(configure_error("topic: /t\ntype: nope_msgs/msg/X\n", "map: {}\n")
    .find("src.yaml:2:7: type: cannot load message type 'nope_msgs/msg/X'"), std::string::npos);
  EXPECT_NE(configure_error(imu + "reliability: maybe\n", "map: {}\n")
    .find("expected 'best_effort' or 'reliable'"), std::string::npos);
  EXPECT_NE(configure_error(imu + "qos: x\n", "map: {}\n").find("unknown key 'qos'"),
    std::string::npos);
  EXPECT_NE(configure_error("topic: /t\ntype: geometry_msgs/msg/Vector3\nstamp: header\n",
    "map: {}\n").find("geometry_msgs/msg/Vector3 has no header.stamp"), std::string::npos);
}

}  // namespace
}  // namespace simport
