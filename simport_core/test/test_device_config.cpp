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

#include <string>
#include <vector>

#include "simport_core/device_config.hpp"

namespace simport
{
namespace
{

std::string error_of(const std::string & yaml)
{
  try {
    DeviceConfig::parse(Config::from_string(yaml, "dev.yaml"));
  } catch (const ConfigError & e) {
    return e.what();
  }
  ADD_FAILURE() << "expected ConfigError for:\n" << yaml;
  return {};
}

const char * const kMinimal =
  "device: d\n"
  "port: { udp: 127.0.0.1:9000 }\n";

TEST(DeviceConfig, LoadsExampleFile)
{
  const DeviceConfig cfg = DeviceConfig::load("data/device_imu.yaml");
  EXPECT_EQ(cfg.device, "imu0");
  EXPECT_FALSE(cfg.clock.has_value());
  EXPECT_EQ(cfg.port.plugin, "simport/SerialPty");
  EXPECT_EQ(cfg.port.config.get<int>("baud"), 115200);
  // The envelope comes from a file next to the device file.
  ASSERT_TRUE(cfg.packet.is_map());
  EXPECT_EQ(cfg.packet["start"].at(1).as<int>(), 0x55);
  EXPECT_FALSE(cfg.framer.has_value());

  ASSERT_EQ(cfg.sends.size(), 2u);
  const SendSpec & data = cfg.sends[0];
  EXPECT_EQ(data.name, "imu_data");
  ASSERT_TRUE(data.source.has_value());
  EXPECT_EQ(data.source->plugin, "simport/RosTopicSource");
  EXPECT_EQ(data.source->config.get<std::string>("topic"), "/imu");
  ASSERT_TRUE(data.rate.has_value());
  EXPECT_EQ(*data.rate, 200.0);
  EXPECT_EQ(data.codec.plugin, "simport/BinaryCodec");
  EXPECT_EQ(data.codec.config.path(), "send[0]");

  const SendSpec & ack = cfg.sends[1];
  EXPECT_FALSE(ack.source.has_value());
  EXPECT_FALSE(ack.rate.has_value());
  EXPECT_TRUE(data.stream);

  ASSERT_EQ(cfg.receives.size(), 1u);
  const ReceiveSpec & command = cfg.receives[0];
  EXPECT_EQ(command.name, "command");
  EXPECT_EQ(command.codec.plugin, "simport/BinaryCodec");
  EXPECT_EQ(command.match.get<int>("gx"), 1);
  EXPECT_EQ(command.target, ReceiveSpec::Target::kService);
  EXPECT_EQ(command.to, "~/command");
  EXPECT_EQ(command.replies, (std::vector<std::string>{"ack"}));
}

TEST(DeviceConfig, PortKinds)
{
  const auto port = [](const std::string & p) {
      return DeviceConfig::parse(Config::from_string(
          "device: d\nport: " + p + "\nreceive: [{ name: r, format: x }]\n", "d.yaml")).port;
    };
  EXPECT_EQ(port("{ serial: /tmp/x }").plugin, "simport/SerialPty");
  EXPECT_EQ(port("{ udp: 0.0.0.0:1 }").plugin, "simport/Udp");
  EXPECT_EQ(port("{ tcp: 0.0.0.0:1 }").plugin, "simport/TcpServer");
  EXPECT_EQ(port("{ can: vcan0 }").plugin, "simport/SocketCan");
  EXPECT_EQ(port("{ plugin: my/Port, x: 1 }").plugin, "my/Port");
  EXPECT_EQ(port("{ can: vcan0, fd: true }").config.get<bool>("fd"), true);
}

TEST(DeviceConfig, CodecsSourcesAndDefaults)
{
  const DeviceConfig cfg = DeviceConfig::parse(Config::from_string(
      std::string(kMinimal) +
      "clock: { plugin: simport/SystemClock }\n"
      "send:\n"
      "  - name: a\n"
      "    from: { topic: /points, type: sensor_msgs/msg/PointCloud2 }\n"
      "    fields: [x: u8]\n"
      "  - name: b\n"
      "    from: { topic: /t, type: std_msgs/msg/String }\n"
      "    format: \"{data}\\n\"\n"
      "  - name: c\n"
      "    codec: { plugin: my/Codec }\n"
      "receive:\n"
      "  - name: r\n"
      "    fields: [x: u8]\n"
      "    to: { topic: /out }\n",
      "dev.yaml"));
  ASSERT_TRUE(cfg.clock.has_value());
  EXPECT_EQ(cfg.clock->plugin, "simport/SystemClock");
  EXPECT_EQ(cfg.sends[0].source->plugin, "simport/PointCloud2Source");
  EXPECT_FALSE(cfg.sends[0].rate.has_value());
  EXPECT_EQ(cfg.sends[1].codec.plugin, "simport/TextCodec");
  EXPECT_EQ(cfg.sends[2].codec.plugin, "my/Codec");
  EXPECT_EQ(cfg.sends[2].codec.config.path(), "send[2].codec");
  EXPECT_FALSE(cfg.packet.defined());
  EXPECT_EQ(cfg.receives[0].target, ReceiveSpec::Target::kTopic);
  EXPECT_EQ(cfg.receives[0].to, "/out");
  EXPECT_TRUE(cfg.receives[0].replies.empty());
  const DeviceConfig defaults = DeviceConfig::parse(Config::from_string(
      std::string(kMinimal) + "receive: [{ name: r, fields: [x: u8] }]\n", "dev.yaml"));
  EXPECT_EQ(defaults.receives[0].to, "~/r");
}

TEST(DeviceConfig, Behavior)
{
  const DeviceConfig cfg = DeviceConfig::parse(Config::from_string(
      std::string(kMinimal) +
      "behavior: { plugin: my/Bus, ids: [1, 2] }\n"
      "send: [{ name: state, fields: [x: u8] }]\n"
      "receive: [{ name: cmd, fields: [x: u8], to: behavior }, { name: log, fields: [y: u16] }]\n",
      "dev.yaml"));
  ASSERT_TRUE(cfg.behavior.has_value());
  EXPECT_EQ(cfg.behavior->plugin, "my/Bus");
  EXPECT_EQ(cfg.receives[0].target, ReceiveSpec::Target::kBehavior);
  EXPECT_TRUE(cfg.receives[0].to.empty());
  EXPECT_EQ(cfg.receives[1].target, ReceiveSpec::Target::kTopic);
}

TEST(DeviceConfig, Errors)
{
  const auto has = [](const std::string & yaml, const std::string & what) {
      const std::string e = error_of(yaml);
      EXPECT_NE(e.find(what), std::string::npos) << "got: " << e;
    };
  const std::string dev = kMinimal;
  has("- a\n", "must be a mapping");
  has("port: { udp: x }\n", "device: required key is missing");
  has("device: 9lives\nport: { udp: x }\n", "invalid device name");
  has(dev, "has no 'send' or 'receive' messages");
  has(dev + "sned: []\n", "did you mean 'send'");
  has("device: d\nport: /dev/ttyUSB0\n", "expected e.g. { serial:");
  has("device: d\nport: { link: x }\n", "say what kind of port");
  has("device: d\nport: { serial: x, udp: y }\n", "a port is one of");
  has("device: d\nport: { plugin: '' }\n", "must not be empty");
  has(dev + "packet: [1]\nsend: [{ name: a, fields: [x: u8] }]\n", "expected the packet envelope");

  has(dev + "send: [{ name: a, fields: [x: u8] }]\nreceive: [{ name: a, fields: [x: u8] }]\n",
    "duplicate message name 'a'");
  has(dev + "send: [{ name: a-b, fields: [x: u8] }]\n", "invalid name 'a-b'");
  has(dev + "send: [{ name: a }]\n", "describe the packet with 'fields:'");
  has(dev + "send: [{ name: a, codec: { plugin: C }, format: x }]\n",
    "give either 'codec' or 'format'");
  has(dev + "send: { a: 1 }\n", "expected a list");
  has(dev + "send: [{ name: a, fields: [x: u8], form: { topic: /x } }]\n", "did you mean 'from'");
  has(dev + "send: [{ name: a, fields: [x: u8], from: /x }]\n", "expected e.g. { topic:");
  has(dev + "send: [{ name: a, fields: [x: u8], rate: 10 }]\n", "this message has no 'from'");
  has(dev + "send: [{ name: a, fields: [x: u8], from: { topic: /x, type: T }, rate: 0 }]\n",
    "rate must be positive");
  has(dev + "send: [{ name: a, fields: [x: u8], from: { topic: /x, type: T },"
    " stale_after: 1 }]\n", "applies with 'rate'");

  has(dev + "send: [{ name: a, fields: [x: u8], stream: false }]\n", "this message has no 'from'");
  has(dev + "send: [{ name: a, fields: [x: u8], from: { topic: /x, type: T }, stream: false,"
    " rate: 5 }]\n", "only sent as a reply");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: behavior }]\n", "needs a 'behavior:'");
  has(dev + "behavior: { plugin: B }\nreceive: [{ name: r, fields: [x: u8] }]\n",
    "no receive sends packets to the behavior");
  has(dev + "behavior: { plugin: B }\nreceive: [{ name: r, fields: [x: u8], to: behavior,"
    " reply: a }]\n", "remove 'reply'");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: behaviour }]\n", "or 'behavior'");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: { topic: /a, service: /b } }]\n",
    "either a 'topic' or a 'service'");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: { topic: '' } }]\n", "must not be empty");
  has(dev + "receive: [{ name: r, fields: [x: u8], reply: a }]\n", "replies come from a service");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: { service: /s }, reply: nope }]\n",
    "no send message named 'nope'; send messages: none");
  has(dev + "send: [{ name: a, fields: [x: u8] }]\n"
    "receive: [{ name: r, fields: [x: u8], to: { service: /s }, reply: [a, b] }]\n",
    "no send message named 'b'; send messages: a");
  has(dev + "receive: [{ name: r, fields: [x: u8], to: { service: /s }, reply: [] }]\n",
    "name the send message(s)");
}

TEST(DeviceConfig, NameValidation)
{
  EXPECT_TRUE(is_valid_name("imu0"));
  EXPECT_TRUE(is_valid_name("a_b_C"));
  EXPECT_FALSE(is_valid_name(""));
  EXPECT_FALSE(is_valid_name("_a"));
  EXPECT_FALSE(is_valid_name("0a"));
  EXPECT_FALSE(is_valid_name("a/b"));
}

}  // namespace
}  // namespace simport
