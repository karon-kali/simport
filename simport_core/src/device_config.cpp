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

#include "simport_core/device_config.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <string>
#include <utility>

namespace simport
{

namespace
{

std::string parse_name(const Config & node, std::set<std::string> * taken)
{
  const Config n = node.required("name");
  std::string name = n.as<std::string>();
  if (!is_valid_name(name)) {
    n.error("invalid name '" + name +
      "': must start with a letter and contain only letters, digits and '_'");
  }
  if (taken != nullptr && !taken->insert(name).second) {
    n.error("duplicate message name '" + name + "'");
  }
  return name;
}

/// The transport behind a `port:` block.
PluginSpec parse_port(const Config & port)
{
  if (!port.is_map()) {
    port.error("expected e.g. { serial: /tmp/simport/my_sensor, baud: 115200 }");
  }
  if (port.has("plugin")) {
    return PluginSpec::parse(port);
  }
  const std::pair<const char *, const char *> kinds[] = {
    {"serial", "simport/SerialPty"}, {"udp", "simport/Udp"}, {"tcp", "simport/TcpServer"},
    {"can", "simport/SocketCan"}};
  std::optional<PluginSpec> found;
  for (const auto & [key, plugin] : kinds) {
    if (port.has(key)) {
      if (found) {
        port[key].error("a port is one of serial, udp, tcp or can");
      }
      found = PluginSpec{plugin, port};
    }
  }
  if (!found) {
    port.error("say what kind of port: serial, udp, tcp, can (or plugin: for your own)");
  }
  return *found;
}

/// The codec of a message: a `codec:` plugin, text (`format:`) or binary (`fields:`).
PluginSpec parse_codec(const Config & message)
{
  if (message.has("codec")) {
    if (message.has("format")) {
      message["format"].error("give either 'codec' or 'format'");
    }
    return PluginSpec::parse(message["codec"]);
  }
  if (message.has("format")) {
    return PluginSpec{"simport/TextCodec", message};
  }
  if (!message.has("fields")) {
    message.error("describe the packet with 'fields:' (binary), 'format:' (text) or 'codec:'");
  }
  return PluginSpec{"simport/BinaryCodec", message};
}

/// The source behind `from:`.
PluginSpec parse_source(const Config & from)
{
  if (!from.is_map()) {
    from.error("expected e.g. { topic: /imu, type: sensor_msgs/msg/Imu }");
  }
  if (from.has("plugin")) {
    return PluginSpec::parse(from);
  }
  const std::string type = from.get<std::string>("type", "");
  if (type == "sensor_msgs/msg/PointCloud2" || type == "sensor_msgs/PointCloud2") {
    return PluginSpec{"simport/PointCloud2Source", from};
  }
  return PluginSpec{"simport/RosTopicSource", from};
}

}  // namespace

bool is_valid_name(std::string_view name)
{
  if (name.empty() || !std::isalpha(static_cast<unsigned char>(name[0]))) {
    return false;
  }
  for (const char c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      return false;
    }
  }
  return true;
}

PluginSpec PluginSpec::parse(const Config & node)
{
  if (!node.is_map()) {
    node.error("expected a plugin mapping such as { plugin: simport/Name, ... }");
  }
  const Config plugin = node.required("plugin");
  std::string name = plugin.as<std::string>();
  if (name.empty()) {
    plugin.error("plugin name must not be empty");
  }
  return PluginSpec{std::move(name), node};
}

DeviceConfig DeviceConfig::parse(const Config & root)
{
  if (!root.is_map()) {
    root.error("a device file must be a mapping with at least 'device' and 'port'");
  }
  root.expect_keys({"device", "clock", "port", "packet", "framer", "behavior", "send",
      "receive"});

  DeviceConfig cfg;
  cfg.root = root;
  const Config device = root.required("device");
  cfg.device = device.as<std::string>();
  if (!is_valid_name(cfg.device)) {
    device.error("invalid device name '" + cfg.device +
      "': must start with a letter and contain only letters, digits and '_'");
  }
  if (root.has("clock")) {
    cfg.clock = PluginSpec::parse(root["clock"]);
  }
  cfg.port = parse_port(root.required("port"));
  if (root.has("packet") && !root["packet"].is_null()) {
    cfg.packet = root["packet"].inline_or_file();  // inline, or a shared envelope file
    if (!cfg.packet.is_map()) {
      cfg.packet.error("expected the packet envelope: { start: [...], fields: [...] }");
    }
  }
  if (root.has("framer") && !root["framer"].is_null()) {
    cfg.framer = PluginSpec::parse(root["framer"]);
  }
  if (root.has("behavior") && !root["behavior"].is_null()) {
    cfg.behavior = PluginSpec::parse(root["behavior"]);
  }

  std::set<std::string> names;
  for (const Config & m : root["send"].items()) {
    m.expect_keys({"name", "from", "rate", "stale_after", "stream", "fields", "frames", "endian",
        "format", "checksum", "codec"});
    SendSpec send;
    send.config = m;
    send.name = parse_name(m, &names);
    send.codec = parse_codec(m);
    if (m.has("from")) {
      send.source = parse_source(m["from"]);
    }
    if (m.has("stream")) {
      if (!send.source) {
        m["stream"].error("'stream' applies to data from 'from:'; this message has no 'from'");
      }
      send.stream = m.get<bool>("stream");
    }
    if (m.has("rate")) {
      if (!send.stream) {
        m["rate"].error("a message with 'stream: false' is only sent as a reply; remove 'rate'");
      }
      if (!send.source) {
        m["rate"].error("'rate' paces data from 'from:'; this message has no 'from'");
      }
      send.rate = m.get<double>("rate");
      if (!(*send.rate > 0.0)) {
        m["rate"].error("rate must be positive (Hz)");
      }
    }
    if (m.has("stale_after")) {
      if (!send.rate) {
        m["stale_after"].error("'stale_after' applies with 'rate'");
      }
      send.stale_after = m.get<double>("stale_after");
      if (send.stale_after < 0.0) {
        m["stale_after"].error("stale_after must be >= 0 seconds (0 = never)");
      }
    }
    cfg.sends.push_back(std::move(send));
  }

  for (const Config & m : root["receive"].items()) {
    m.expect_keys({"name", "fields", "endian", "format", "checksum", "codec", "match", "to",
        "reply"});
    ReceiveSpec receive;
    receive.config = m;
    receive.name = parse_name(m, &names);
    receive.codec = parse_codec(m);
    receive.match = m["match"];
    receive.to = "~/" + receive.name;
    if (m.has("to") && m["to"].is_scalar()) {
      if (m["to"].scalar() != "behavior") {
        m["to"].error("expected { topic: ... }, { service: ... } or 'behavior'");
      }
      if (!root.has("behavior")) {
        m["to"].error("'to: behavior' needs a 'behavior:' plugin in the device file");
      }
      receive.target = ReceiveSpec::Target::kBehavior;
      receive.to.clear();
    } else if (m.has("to")) {
      const Config to = m["to"];
      to.expect_keys({"topic", "service"});
      if (to.has("topic") == to.has("service")) {
        to.error("send it to either a 'topic' or a 'service'");
      }
      if (to.has("service")) {
        receive.target = ReceiveSpec::Target::kService;
        receive.to = to.get<std::string>("service");
      } else {
        receive.to = to.get<std::string>("topic");
      }
      if (receive.to.empty()) {
        to.error("the name must not be empty");
      }
    }
    if (m.has("reply")) {
      if (receive.target == ReceiveSpec::Target::kBehavior) {
        m["reply"].error("a behavior answers with any send message; remove 'reply'");
      }
      if (receive.target != ReceiveSpec::Target::kService) {
        m["reply"].error("replies come from a service: use 'to: { service: ... }'");
      }
      const Config reply = m["reply"];
      receive.replies = reply.is_scalar() ? std::vector<std::string>{reply.as<std::string>()} :
      reply.as<std::vector<std::string>>();
      if (receive.replies.empty()) {
        reply.error("name the send message(s) the service's replies are sent as");
      }
    }
    cfg.receives.push_back(std::move(receive));
  }

  // Replies name `send` messages. One with a `from:` topic replies with its latest data, with
  // the fields the response sets replaced.
  for (const ReceiveSpec & r : cfg.receives) {
    for (const std::string & name : r.replies) {
      const auto it = std::find_if(cfg.sends.begin(), cfg.sends.end(),
          [&](const SendSpec & s) {return s.name == name;});
      if (it == cfg.sends.end()) {
        std::string known;
        for (const SendSpec & s : cfg.sends) {
          known += (known.empty() ? "" : ", ") + s.name;
        }
        r.config["reply"].error("no send message named '" + name + "'; send messages: " +
          (known.empty() ? "none" : known));
      }
    }
  }
  if (cfg.behavior && std::none_of(cfg.receives.begin(), cfg.receives.end(),
    [](const ReceiveSpec & r) {return r.target == ReceiveSpec::Target::kBehavior;}))
  {
    cfg.behavior->config.error("no receive sends packets to the behavior: add 'to: behavior'");
  }
  if (cfg.sends.empty() && cfg.receives.empty()) {
    root.error("device '" + cfg.device + "' has no 'send' or 'receive' messages");
  }
  return cfg;
}

DeviceConfig DeviceConfig::load(const std::filesystem::path & file)
{
  return parse(Config::load_file(file));
}

}  // namespace simport
