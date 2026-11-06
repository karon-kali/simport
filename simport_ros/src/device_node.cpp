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

#include "simport_ros/device_node.hpp"

#include <chrono>
#include <cinttypes>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "simport_core/device_config.hpp"
#include "simport_ros/ros_logger.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace simport
{

DeviceNode::DeviceNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("simport_device", options)
{
  declare_parameter("config_file", std::string());
  declare_parameter("autostart", true);
  declare_parameter("debug_frames", false);

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID(get_name());

  if (get_parameter("autostart").as_bool()) {
    // Deferred so that the node is fully constructed (and owned by a shared_ptr) first.
    autostart_timer_ = create_wall_timer(
      0ms, [this]() {
        autostart_timer_->cancel();
        if (configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          activate();
        }
      });
  }
}

DeviceNode::~DeviceNode()
{
  teardown();
}

DeviceNode::CallbackReturn DeviceNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    const std::string file = get_parameter("config_file").as_string();
    if (file.empty()) {
      throw ConfigError("parameter 'config_file' is not set");
    }
    const DeviceConfig cfg = DeviceConfig::load(file);
    device_ = std::make_unique<Device>(*this, std::make_shared<RosLogger>(get_logger()));
    device_->build(cfg);

    if (get_parameter("debug_frames").as_bool()) {
      frames_publisher_ = create_publisher<simport_msgs::msg::Frame>("~/frames", 100);
      auto publisher = frames_publisher_;
      device_->set_frame_tap(
        [publisher](const Frame & f, bool outbound, const std::string & channel) {
          simport_msgs::msg::Frame msg;
          msg.header.stamp.sec = static_cast<std::int32_t>(f.stamp.count() / 1'000'000'000);
          msg.header.stamp.nanosec = static_cast<std::uint32_t>(f.stamp.count() % 1'000'000'000);
          msg.name = channel;
          msg.direction = outbound ? simport_msgs::msg::Frame::DIRECTION_OUT :
          simport_msgs::msg::Frame::DIRECTION_IN;
          msg.data = f.data;
          if (f.meta.endpoint) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "endpoint";
            kv.value = f.meta.endpoint->host + ":" + std::to_string(f.meta.endpoint->port);
            msg.meta.push_back(kv);
          }
          publisher->publish(std::move(msg));
        });
    }

    for (const auto & s : device_->sends()) {
      const std::string rate = s.spec.rate ? " at " + std::to_string(*s.spec.rate) + " Hz" :
        "";
      RCLCPP_INFO(
        get_logger(), "send '%s': %s%s", s.spec.name.c_str(),
        s.source ? s.source->describe().c_str() : "replies only",
        s.spec.stream ? rate.c_str() : ", as replies only");
    }
    for (const auto & r : device_->receives()) {
      std::string replies;
      for (const std::string & name : r->spec.replies) {
        replies += (replies.empty() ? ", replies as " : " or ") + name;
      }
      if (r->spec.target == ReceiveSpec::Target::kBehavior) {
        RCLCPP_INFO(get_logger(), "receive '%s': to the behavior", r->spec.name.c_str());
        continue;
      }
      RCLCPP_INFO(
        get_logger(), "receive '%s': to %s %s%s", r->spec.name.c_str(),
        r->spec.target == ReceiveSpec::Target::kService ? "service" : "topic",
        r->spec.to.c_str(), replies.c_str());
    }
    if (device_->behavior()) {
      RCLCPP_INFO(get_logger(), "behavior: %s", device_->behavior()->describe().c_str());
    }
    RCLCPP_INFO(
      get_logger(), "device '%s' configured: %s (clock: %s)", device_->name().c_str(),
      device_->transport()->describe().c_str(), device_->clock()->describe().c_str());
    add_diagnostics();
    return CallbackReturn::SUCCESS;
  } catch (const ConfigError & e) {
    RCLCPP_ERROR(get_logger(), "configuration error: %s", e.what());
  } catch (const TransportError & e) {
    RCLCPP_ERROR(get_logger(), "cannot open port: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
  }
  teardown();
  return CallbackReturn::FAILURE;
}

DeviceNode::CallbackReturn DeviceNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  device_->start();
  RCLCPP_INFO(get_logger(), "active: data is flowing");
  return CallbackReturn::SUCCESS;
}

DeviceNode::CallbackReturn DeviceNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  device_->stop();
  log_stats();
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

DeviceNode::CallbackReturn DeviceNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  teardown();
  return CallbackReturn::SUCCESS;
}

DeviceNode::CallbackReturn DeviceNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  teardown();
  return CallbackReturn::SUCCESS;
}

DeviceNode::CallbackReturn DeviceNode::on_error(const rclcpp_lifecycle::State &)
{
  teardown();
  return CallbackReturn::SUCCESS;
}

void DeviceNode::teardown()
{
  remove_diagnostics();
  device_.reset();
  frames_publisher_.reset();
}

void DeviceNode::log_stats()
{
  for (const auto & o : device_->sends()) {
    const OutputChannelStats s = o.channel->stats();
    RCLCPP_INFO(
      get_logger(),
      "send '%s': %" PRIu64 " records in, %" PRIu64 " packets (%" PRIu64 " bytes) sent, %"
      PRIu64 " dropped, %" PRIu64 " replies, %" PRIu64 " encode errors",
      o.spec.name.c_str(), s.records_in, s.frames, s.bytes, s.dropped, s.requests,
      s.encode_errors);
  }
  for (const auto & r : device_->receives()) {
    RCLCPP_INFO(
      get_logger(), "receive '%s': %" PRIu64 " packets", r->spec.name.c_str(),
      r->channel->stats().frames);
  }
}

void DeviceNode::add_diagnostics()
{
  // diagnostic_updater already prefixes entries with the node name.
  const std::string prefix;
  // Frames per second since the previous update of entry `key`.
  auto rate = [this](const std::string & key, std::uint64_t frames) {
      const auto now = std::chrono::steady_clock::now();
      auto [it, fresh] = rate_state_.try_emplace(key, now, frames);
      const double dt = std::chrono::duration<double>(now - it->second.first).count();
      const double r = fresh || dt <= 0 ? 0.0 :
        static_cast<double>(frames - it->second.second) / dt;
      it->second = {now, frames};
      return r;
    };

  const std::string transport_name = prefix + "transport";
  diagnostics_->add(
    transport_name, [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      const TransportStats s = device_->transport()->stats();
      if (s.peer_connected && !*s.peer_connected) {
        stat.summary(DiagnosticStatus::OK, "waiting for a driver to connect");
      } else {
        stat.summary(DiagnosticStatus::OK, "open");
      }
      stat.add("port", device_->transport()->describe());
      if (s.peer_connected) {
        stat.add("driver connected", *s.peer_connected);
      }
      stat.add("frames sent", s.frames_tx);
      stat.add("bytes sent", s.bytes_tx);
      stat.add("frames received", s.frames_rx);
      stat.add("bytes received", s.bytes_rx);
      stat.add("dropped", s.tx_dropped);
      stat.add("receive errors", s.rx_errors);
      if (const InboundRouter * r = device_->router()) {
        stat.add("inbound frames", r->stats().frames);
        stat.add("unmatched inbound frames", r->stats().unmatched);
      }
    });
  diagnostic_names_.push_back(transport_name);

  for (const auto & o : device_->sends()) {
    const std::string name = prefix + "send " + o.spec.name;
    OutputChannel * channel = o.channel.get();
    diagnostics_->add(
      name, [channel, name, rate](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        const OutputChannelStats s = channel->stats();
        const double hz = rate(name, s.frames);
        if (s.encode_errors > 0) {
          stat.summary(DiagnosticStatus::ERROR, "encode errors; check the mapping and codec");
        } else {
          stat.summaryf(DiagnosticStatus::OK, "%.1f frames/s", hz);
        }
        stat.add("rate (frames/s)", hz);
        stat.add("records from source", s.records_in);
        stat.add("frames sent", s.frames);
        stat.add("bytes sent", s.bytes);
        stat.add("dropped by transport", s.dropped);
        stat.add("replies", s.requests);
        stat.add("encode errors", s.encode_errors);
      });
    diagnostic_names_.push_back(name);
  }

  for (const auto & i : device_->receives()) {
    const std::string name = prefix + "receive " + i->spec.name;
    InputChannel * channel = i->channel.get();
    diagnostics_->add(
      name, [channel](diagnostic_updater::DiagnosticStatusWrapper & stat) {
        stat.summary(DiagnosticStatus::OK, "ok");
        stat.add("frames", channel->stats().frames);
      });
    diagnostic_names_.push_back(name);
  }
}

void DeviceNode::remove_diagnostics()
{
  for (const auto & name : diagnostic_names_) {
    diagnostics_->removeByName(name);
  }
  diagnostic_names_.clear();
  rate_state_.clear();
}

}  // namespace simport

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simport::DeviceNode)
