// Copyright 2026 Your Name
//
// Template: a simport behavior plugin.
//
// A behavior decides what a device answers, inside the device's process: no service round
// trip, so it keeps up with fast control loops (thousands of packets per second). For slower
// devices, a ROS node serving the receive's service (templates/device_node.py) is simpler.
//
// This one answers the commands of example.yaml: PING (cmd 1) with an ACK of status 0, any
// other command with status 1. In the device file:
//
//   behavior: { plugin: my_simport_behavior/MyBehavior, ping_cmd: 1 }
//   receive:
//     - name: command
//       fields: [...]
//       to: behavior
//
// Rename the class, the messages and the fields for your device; keep the structure.

#include <cstdint>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "simport_ros/behavior.hpp"

namespace my_simport_behavior
{

class MyBehavior : public simport::Behavior
{
public:
  void configure(
    const simport::Config & config, simport::Context & /*context*/,
    rclcpp_lifecycle::LifecycleNode & /*node*/, const simport::DeviceMessages & messages) override
  {
    // Your own keys, checked so that typos are errors pointing at the YAML line.
    config.expect_keys({"plugin", "ping_cmd"});
    ping_cmd_ = config.get<std::uint64_t>("ping_cmd", 1);

    // Bind message and field names to indices once, so handle() never looks up names.
    command_ = messages.receive("command", config);
    cmd_ = field(messages.receives[command_], "cmd", config);
    ack_ = messages.send("ack", config);
    ack_cmd_ = field(messages.sends[ack_], "cmd", config);
    ack_status_ = field(messages.sends[ack_], "status", config);

    // `node` is the device's ROS node: create your own subscriptions and publishers on it,
    // e.g. to read simulated joints. Their callbacks run on another thread than handle().
  }

  void handle(
    std::size_t receive, const simport::Record & packet,
    std::vector<simport::Answer> & answers) override
  {
    if (receive != command_) {
      return;
    }
    const simport::Value & cmd = packet[cmd_];
    const std::uint64_t status = simport::saturate_cast<std::uint64_t>(cmd) == ping_cmd_ ? 0 : 1;
    // An answer: a send message and the field values to set. Constants, counters, lengths and
    // checksums are filled in by simport; the packet goes back to whoever sent the command.
    answers.push_back(simport::Answer{ack_, {
          {ack_cmd_, cmd},
          {ack_status_, simport::Value{status}}}});
  }

  std::string describe() const override {return "my behavior";}

private:
  static std::size_t field(
    const simport::MessageInfo & message, const char * name, const simport::Config & where)
  {
    const auto index = message.schema->find(name);
    if (!index) {
      where.error("message '" + message.name + "' has no field '" + name + "'");
    }
    return *index;
  }

  std::uint64_t ping_cmd_ = 1;
  std::size_t command_ = 0, cmd_ = 0, ack_ = 0, ack_cmd_ = 0, ack_status_ = 0;
};

}  // namespace my_simport_behavior

PLUGINLIB_EXPORT_CLASS(my_simport_behavior::MyBehavior, simport::Behavior)
