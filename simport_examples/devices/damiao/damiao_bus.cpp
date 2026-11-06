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

// Damiao DM-J motors on CAN, in process: the fast alternative to a service node for motor
// control loops (a 7-motor arm at 750 Hz is over 10,000 frames per second).
//
//   behavior:
//     plugin: simport_examples/DamiaoBus
//     motors:
//       - { id: 0x01, master_id: 0x11, type: DM8009, joint: openarm_joint1 }
//     joint_states: /sim/joint_states            # sensor_msgs/JointState (default)
//     commands: /sim/{joint}/cmd_{mode}          # std_msgs/Float64, mode pos|vel|eff (default)
//
// The device file's receives `param`, `command`, `mit`, `pos_vel` and `vel` (any that exist)
// go `to: behavior`; it answers with the sends `state` and `param_reply`.

#include <bit>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "damiao.hpp"
#include "simport_ros/behavior.hpp"
#include "std_msgs/msg/float64.hpp"

namespace simport_examples
{

namespace damiao
{

std::optional<Limits> limits_of(std::string_view type)
{
  static const std::map<std::string_view, Limits> kTypes{
    {"DM3507", {12.5, 50, 5}}, {"DM4310", {12.5, 30, 10}}, {"DM4310_48V", {12.5, 50, 10}},
    {"DM4340", {12.5, 10, 28}}, {"DM4340_48V", {12.5, 10, 28}}, {"DM6006", {12.5, 45, 20}},
    {"DM8006", {12.5, 45, 40}}, {"DM8009", {12.5, 45, 54}}, {"DM10010L", {12.5, 25, 200}},
    {"DM10010", {12.5, 20, 200}}, {"DMH3510", {12.5, 280, 1}}, {"DMH6215", {12.5, 45, 10}},
    {"DMG6220", {12.5, 45, 10}}};
  const auto it = kTypes.find(type);
  return it == kTypes.end() ? std::nullopt : std::optional<Limits>(it->second);
}

}  // namespace damiao

namespace
{

using simport::Value;

constexpr std::uint64_t kEnable = 0xFC, kDisable = 0xFD, kSetZero = 0xFE;
constexpr std::uint64_t kRefresh = 0xCC, kRead = 0x33, kWrite = 0x55, kSave = 0xAA;
constexpr std::uint8_t kRidMasterId = 7, kRidEscId = 8, kRidCtrlMode = 10, kRidPmax = 21,
  kRidVmax = 22, kRidTmax = 23, kRidCanBaud = 35;

bool integer_param(std::uint64_t rid)
{
  return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) || rid == 35 || rid == 36;
}

std::uint32_t float_bits(double x) {return std::bit_cast<std::uint32_t>(static_cast<float>(x));}

std::uint64_t as_uint(const Value & v) {return simport::saturate_cast<std::uint64_t>(v);}

}  // namespace

class DamiaoBus : public simport::Behavior
{
public:
  ~DamiaoBus() override
  {
    // Release the command publishers in parallel. When the simulator stops at the same time
    // (Ctrl-C on a launch), its subscribers never acknowledge the last commands, and DDS may
    // hold back deleting each reliable publisher for its linger time (Cyclone: 1 s). One after
    // another, seven joints would take seven seconds.
    std::vector<std::thread> releases;
    for (auto & [id, m] : motors_) {
      for (auto & [mode, publisher] : m.commands) {
        releases.emplace_back([p = std::move(publisher)]() mutable {p.reset();});
      }
    }
    for (std::thread & t : releases) {
      t.join();
    }
  }

  void configure(
    const simport::Config & config, simport::Context & /*context*/,
    rclcpp_lifecycle::LifecycleNode & node, const simport::DeviceMessages & messages) override
  {
    config.expect_keys({"plugin", "motors", "joint_states", "commands"});
    const std::string commands = config.get<std::string>("commands", "/sim/{joint}/cmd_{mode}");
    motors_.clear();
    for (const simport::Config & item : config.required("motors").items()) {
      item.expect_keys({"id", "master_id", "type", "joint"});
      Motor m;
      m.id = item.get<std::uint32_t>("id");
      m.master_id = item.get<std::uint32_t>("master_id");
      m.joint = item.get<std::string>("joint");
      const auto limits = damiao::limits_of(item.get<std::string>("type", "DM4310"));
      if (!limits) {
        item["type"].error("unknown Damiao motor type");
      }
      m.limits = *limits;
      reset_params(m);
      for (const char * mode : {"pos", "vel", "eff"}) {
        std::string topic = commands;
        replace(topic, "{joint}", m.joint);
        replace(topic, "{mode}", mode);
        m.commands[mode] = rclcpp::create_publisher<std_msgs::msg::Float64>(node, topic, 10);
      }
      if (!motors_.emplace(m.id, std::move(m)).second) {
        item["id"].error("duplicate motor id");
      }
    }
    if (motors_.empty()) {
      config["motors"].error("give at least one motor");
    }

    // Bind messages and fields to indices once; handle() then never looks up names.
    const auto field = [](const simport::MessageInfo & msg, const char * name,
      const simport::Config & where) {
        const auto index = msg.schema->find(name);
        if (!index) {
          where.error("message '" + msg.name + "' has no field '" + name + "'");
        }
        return *index;
      };
    kinds_.assign(messages.receives.size(), Kind::kNone);
    for (std::size_t i = 0; i < messages.receives.size(); ++i) {
      const simport::MessageInfo & r = messages.receives[i];
      Kind kind = Kind::kNone;
      if (r.name == "param") {
        kind = Kind::kParam;
        in_.motor_id = field(r, "motor_id", config);
        in_.op = field(r, "op", config);
        in_.rid = field(r, "rid", config);
        in_.value = field(r, "value", config);
      } else if (r.name == "command") {
        kind = Kind::kCommand;
        in_.cmd = field(r, "cmd", config);
      } else if (r.name == "mit") {
        kind = Kind::kMit;
        in_.q = field(r, "q", config);
        in_.dq = field(r, "dq", config);
        in_.kp = field(r, "kp", config);
        in_.kd = field(r, "kd", config);
        in_.tau = field(r, "tau", config);
      } else if (r.name == "pos_vel") {
        kind = Kind::kPosVel;
        in_.p = field(r, "p", config);
        in_.pv_v = field(r, "v", config);
      } else if (r.name == "vel") {
        kind = Kind::kVel;
        in_.v = field(r, "v", config);
      }
      if (kind != Kind::kNone && kind != Kind::kParam) {
        can_id_[i] = field(r, "can_id", config);
      }
      kinds_[i] = kind;
    }
    const simport::MessageInfo & state = messages.sends[messages.send("state", config)];
    state_ = messages.send("state", config);
    out_.can_id = field(state, "can_id", config);
    out_.motor_id = field(state, "motor_id", config);
    out_.status = field(state, "status", config);
    out_.q = field(state, "q", config);
    out_.dq = field(state, "dq", config);
    out_.tau = field(state, "tau", config);
    out_.t_mos = field(state, "t_mos", config);
    out_.t_rotor = field(state, "t_rotor", config);
    const simport::MessageInfo & reply = messages.sends[messages.send("param_reply", config)];
    param_reply_ = messages.send("param_reply", config);
    out_.p_can_id = field(reply, "can_id", config);
    out_.p_motor_id = field(reply, "motor_id", config);
    out_.p_op = field(reply, "op", config);
    out_.p_rid = field(reply, "rid", config);
    out_.p_value = field(reply, "value", config);

    joint_states_ = node.create_subscription<sensor_msgs::msg::JointState>(
      config.get<std::string>("joint_states", "/sim/joint_states"), 10,
      [this](const sensor_msgs::msg::JointState & msg) {on_joint_states(msg);});
  }

  void start() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [id, m] : motors_) {  // power-up state
      m.enabled = false;
      m.zero = 0.0;
      reset_params(m);
    }
  }

  void handle(
    std::size_t receive, const simport::Record & packet,
    std::vector<simport::Answer> & answers) override
  {
    const Kind kind = kinds_[receive];
    if (kind == Kind::kNone) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t id = kind == Kind::kParam ? as_uint(packet[in_.motor_id]) :
      as_uint(packet[can_id_.at(receive)]) & 0xFF;
    const auto it = motors_.find(static_cast<std::uint32_t>(id));
    if (it == motors_.end()) {
      return;  // another device on the bus
    }
    Motor & m = it->second;
    const damiao::Limits & l = m.limits;
    switch (kind) {
      case Kind::kParam: {
          const std::uint64_t op = as_uint(packet[in_.op]);
          const std::uint64_t rid = as_uint(packet[in_.rid]);
          if (op == kRefresh) {
            answers.push_back(state(m));
          } else if (op == kRead || op == kWrite) {
            const auto & value = std::get<simport::Bytes>(packet[in_.value]);
            if (op == kWrite && value.size() >= 4) {
              m.params[rid] = static_cast<std::uint32_t>(
                value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24));
            }
            const auto p = m.params.find(rid);
            const std::uint32_t v = p != m.params.end() ? p->second :
              (integer_param(rid) ? 0u : float_bits(0.0));
            answers.push_back(param_reply(m, op, rid, v));
          } else if (op == kSave) {
            answers.push_back(param_reply(m, op, 1, 0));
          }
          return;
        }
      case Kind::kCommand: {
          const std::uint64_t cmd = as_uint(packet[in_.cmd]);
          if (cmd == kEnable) {
            m.enabled = true;
          } else if (cmd == kDisable) {
            m.enabled = false;
          } else if (cmd == kSetZero) {
            m.zero = joint(m).position;
          }
          break;
        }
      case Kind::kMit:
        if (m.enabled) {
          if (as_uint(packet[in_.kp]) > 0) {
            command(m, "pos", damiao::from_uint(as_uint(packet[in_.q]), -l.p_max, l.p_max, 16) +
              m.zero);
          } else if (as_uint(packet[in_.kd]) > 0) {
            command(m, "vel", damiao::from_uint(as_uint(packet[in_.dq]), -l.v_max, l.v_max, 12));
          } else {
            command(m, "eff",
              damiao::from_uint(as_uint(packet[in_.tau]), -l.t_max, l.t_max, 12));
          }
        }
        break;
      case Kind::kPosVel:
        if (m.enabled) {
          command(m, "pos", simport::to_double(packet[in_.p]) + m.zero);
        }
        break;
      case Kind::kVel:
        if (m.enabled) {
          command(m, "vel", simport::to_double(packet[in_.v]));
        }
        break;
      case Kind::kNone:
        return;
    }
    answers.push_back(state(m));
  }

  std::string describe() const override
  {
    return "Damiao bus: " + std::to_string(motors_.size()) + " motors";
  }

private:
  enum class Kind {kNone, kParam, kCommand, kMit, kPosVel, kVel};

  struct JointValue
  {
    double position = 0.0, velocity = 0.0, effort = 0.0;
  };

  struct Motor
  {
    std::uint32_t id = 0;
    std::uint32_t master_id = 0;
    std::string joint;
    damiao::Limits limits{};
    bool enabled = false;
    double zero = 0.0;
    std::map<std::uint64_t, std::uint32_t> params;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> commands;
  };

  static void replace(std::string & s, const std::string & what, const std::string & with)
  {
    for (std::size_t at = s.find(what); at != std::string::npos; at = s.find(what, at)) {
      s.replace(at, what.size(), with);
      at += with.size();
    }
  }

  static void reset_params(Motor & m)
  {
    m.params = {
      {kRidMasterId, m.master_id}, {kRidEscId, m.id}, {kRidCtrlMode, 1},  // MIT
      {kRidPmax, float_bits(m.limits.p_max)}, {kRidVmax, float_bits(m.limits.v_max)},
      {kRidTmax, float_bits(m.limits.t_max)}, {kRidCanBaud, 9}};  // 5 Mbit/s data rate
  }

  void on_joint_states(const sensor_msgs::msg::JointState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      JointValue & j = joints_[msg.name[i]];
      j.position = i < msg.position.size() ? msg.position[i] : 0.0;
      j.velocity = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
      j.effort = i < msg.effort.size() ? msg.effort[i] : 0.0;
    }
  }

  JointValue joint(const Motor & m) const
  {
    const auto it = joints_.find(m.joint);
    return it == joints_.end() ? JointValue{} : it->second;
  }

  static void command(Motor & m, const char * mode, double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    m.commands.at(mode)->publish(msg);
  }

  simport::Answer state(const Motor & m) const
  {
    const JointValue j = joint(m);
    const damiao::Limits & l = m.limits;
    return simport::Answer{state_, {
        {out_.can_id, Value{std::uint64_t{m.master_id}}},
        {out_.motor_id, Value{std::uint64_t{m.id & 0x0F}}},
        {out_.status, Value{std::uint64_t{m.enabled ? 1u : 0u}}},
        {out_.q, Value{std::uint64_t{damiao::to_uint(j.position - m.zero, -l.p_max, l.p_max, 16)}}},
        {out_.dq, Value{std::uint64_t{damiao::to_uint(j.velocity, -l.v_max, l.v_max, 12)}}},
        {out_.tau, Value{std::uint64_t{damiao::to_uint(j.effort, -l.t_max, l.t_max, 12)}}},
        {out_.t_mos, Value{std::uint64_t{30}}}, {out_.t_rotor, Value{std::uint64_t{30}}}}};
  }

  simport::Answer param_reply(
    const Motor & m, std::uint64_t op, std::uint64_t rid, std::uint32_t value) const
  {
    return simport::Answer{param_reply_, {
        {out_.p_can_id, Value{std::uint64_t{m.master_id}}},
        {out_.p_motor_id, Value{std::uint64_t{m.id}}},
        {out_.p_op, Value{op}}, {out_.p_rid, Value{rid}},
        {out_.p_value, Value{std::uint64_t{value}}}}};
  }

  std::mutex mutex_;  ///< motors_ and joints_: the receive thread and ROS callbacks
  std::map<std::uint32_t, Motor> motors_;
  std::map<std::string, JointValue> joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;

  std::vector<Kind> kinds_;                      ///< by receive index
  std::map<std::size_t, std::size_t> can_id_;    ///< receive index -> its can_id field
  struct
  {
    std::size_t motor_id = 0, op = 0, rid = 0, value = 0, cmd = 0;
    std::size_t q = 0, dq = 0, kp = 0, kd = 0, tau = 0, p = 0, pv_v = 0, v = 0;
  } in_;
  std::size_t state_ = 0, param_reply_ = 0;
  struct
  {
    std::size_t can_id = 0, motor_id = 0, status = 0, q = 0, dq = 0, tau = 0, t_mos = 0,
      t_rotor = 0;
    std::size_t p_can_id = 0, p_motor_id = 0, p_op = 0, p_rid = 0, p_value = 0;
  } out_;
};

}  // namespace simport_examples

PLUGINLIB_EXPORT_CLASS(simport_examples::DamiaoBus, simport::Behavior)
