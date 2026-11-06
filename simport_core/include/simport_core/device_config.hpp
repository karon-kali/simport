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

#ifndef SIMPORT_CORE__DEVICE_CONFIG_HPP_
#define SIMPORT_CORE__DEVICE_CONFIG_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "simport_core/config.hpp"

namespace simport
{

/// A plugin and the YAML node that configures it.
struct PluginSpec
{
  /// pluginlib lookup name, e.g. `simport/SerialPty`.
  std::string plugin;
  /// The node the plugin reads its settings from.
  Config config;

  /// Parse `{ plugin: <name>, ... }`; the name must be a non-empty string.
  static PluginSpec parse(const Config & node);
};

/// One `send:` message: ROS -> driver.
struct SendSpec
{
  std::string name;
  /// BinaryCodec (`fields:`), TextCodec (`format:`) or a `codec:` plugin. `config` is the
  /// message itself for the first two, the `codec:` block for a plugin.
  PluginSpec codec;
  /// `from:`: where the data comes from. Absent for messages sent only as replies.
  std::optional<PluginSpec> source;
  /// `rate:` (Hz): send at the device's own rate; absent: one packet per ROS message.
  std::optional<double> rate;
  /// `stale_after:` (s): with `rate`, stop sending when no new data arrived for this long.
  double stale_after = 1.0;
  /// `stream: false`: keep the latest data from `from:`, but send it only as a reply (a
  /// polled device).
  bool stream = true;
  Config config;
};

/// One `receive:` message: driver -> ROS.
struct ReceiveSpec
{
  /// Where decoded packets go: a topic, a service (another process), or the device's
  /// in-process `behavior` plugin.
  enum class Target {kTopic, kService, kBehavior};

  std::string name;
  PluginSpec codec;
  /// `{ field: value }` a decoded packet must have (may be undefined).
  Config match;
  Target target = Target::kTopic;
  /// Topic or service name (`to:`); default topic `~/<name>`. Empty for a behavior.
  std::string to;
  /// With a service: the `send` messages a response may be encoded as. Each reply in the
  /// response names one (by `name`); an unnamed reply uses the first.
  std::vector<std::string> replies;
  Config config;
};

/// A device file: one emulated port and the messages that cross it.
///
/// ```yaml
/// device: my_sensor
/// port: { serial: /tmp/simport/my_sensor, baud: 115200 }
/// packet: { start: [0x5A], fields: [...] }   # optional envelope shared by all messages
/// behavior: { plugin: ... }                   # optional: answers in process
/// send: [...]                                 # ROS -> driver
/// receive: [...]                              # driver -> ROS
/// ```
struct DeviceConfig
{
  std::string device;
  /// Absent: the host chooses (simport_ros uses its ROS clock).
  std::optional<PluginSpec> clock;
  /// The transport, chosen by the port's key: serial, udp, tcp, can, or `plugin:`.
  PluginSpec port;
  /// The envelope (`packet:`), undefined if the messages are complete packets.
  Config packet;
  /// An explicit framer; otherwise derived from `packet` where the device needs one.
  std::optional<PluginSpec> framer;
  /// `behavior:`: an in-process plugin that answers the receives with `to: behavior`.
  std::optional<PluginSpec> behavior;
  std::vector<SendSpec> sends;
  std::vector<ReceiveSpec> receives;
  Config root;

  static DeviceConfig parse(const Config & root);
  static DeviceConfig load(const std::filesystem::path & file);
};

/// Device and message names: `[A-Za-z][A-Za-z0-9_]*`, so they are valid in ROS names.
bool is_valid_name(std::string_view name);

}  // namespace simport

#endif  // SIMPORT_CORE__DEVICE_CONFIG_HPP_
