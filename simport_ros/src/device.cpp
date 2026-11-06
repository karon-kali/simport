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

#include "simport_ros/device.hpp"

#include <algorithm>
#include <format>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/codec.hpp"
#include "simport_core/frame_filter.hpp"
#include "simport_core/trigger.hpp"
#include "simport_ros/plugin_loader.hpp"
#include "simport_ros/records.hpp"
#include "simport_ros/ros_clock.hpp"

namespace simport
{

struct Device::Loaders
{
  PluginLoader<Clock> clock{"simport_core", "simport::Clock"};
  PluginLoader<Transport> transport{"simport_core", "simport::Transport"};
  PluginLoader<Codec> codec{"simport_core", "simport::Codec"};
  PluginLoader<Trigger> trigger{"simport_core", "simport::Trigger"};
  PluginLoader<FrameFilter> filter{"simport_core", "simport::FrameFilter"};
  PluginLoader<Framer> framer{"simport_core", "simport::Framer"};
  PluginLoader<Source> source{"simport_ros", "simport::Source"};
  PluginLoader<Behavior> behavior{"simport_ros", "simport::Behavior"};
};

namespace
{

std::size_t index_of(
  const std::vector<MessageInfo> & messages, const std::string & name, const Config & where,
  const char * kind)
{
  std::string names;
  for (std::size_t i = 0; i < messages.size(); ++i) {
    if (messages[i].name == name) {
      return i;
    }
    names += (names.empty() ? "" : ", ") + messages[i].name;
  }
  where.error(std::string("the device has no ") + kind + " message '" + name + "'; " + kind +
    " messages: " + (names.empty() ? "none" : names));
}

}  // namespace

std::size_t DeviceMessages::send(const std::string & name, const Config & where) const
{
  return index_of(sends, name, where, "send");
}

std::size_t DeviceMessages::receive(const std::string & name, const Config & where) const
{
  return index_of(receives, name, where, "receive");
}

namespace
{

/// Requests to a receive's service that may be waiting for an answer.
constexpr std::size_t kExchangeQueueDepth = 1000;

/// A plugin configured by the device itself (not written by the user).
PluginSpec internal(const std::string & plugin, const std::string & yaml, const std::string & what)
{
  return PluginSpec{plugin, Config::from_string(yaml.empty() ? "{}" : yaml, what)};
}

}  // namespace

Device::Device(rclcpp_lifecycle::LifecycleNode & node, LoggerPtr logger)
: node_(node), logger_(std::move(logger)), loaders_(std::make_unique<Loaders>()) {}

Device::~Device()
{
  close();
}

void Device::build(const DeviceConfig & cfg, bool open_transport)
{
  close();
  name_ = cfg.device;

  // -- clock ---------------------------------------------------------------------------------
  if (!cfg.clock || cfg.clock->plugin == "simport/RosClock") {
    clock_ = std::make_shared<RosClock>(node_.get_clock());
  } else {
    clock_ = loaders_->clock.create(*cfg.clock);
    clock_->configure(cfg.clock->config, *logger_);
  }
  Context device(cfg.device, logger_, clock_);

  // -- port ----------------------------------------------------------------------------------
  transport_ = loaders_->transport.create(cfg.port);
  Context transport_ctx = device.child("port");
  transport_->configure(cfg.port.config, transport_ctx);
  const TransportCaps caps = transport_->caps();

  // Line-rate model: the serial baud rate, or the port's `bps`. Shared by all messages.
  FrameFilterPtr line;
  const Config port = cfg.port.config;
  if (caps.line_rate_bps || port.has("bps")) {
    std::string yaml;
    if (port.has("bps")) {
      yaml += std::format("bps: {}\nbits_per_byte: 8\n", port.get<double>("bps"));
    }
    if (port.has("max_delay")) {
      yaml += std::format("max_delay: {}\n", port.get<double>("max_delay"));
    }
    const PluginSpec spec = internal("simport/Bandwidth", yaml, cfg.device + " line model");
    line = loaders_->filter.create(spec);
    Context line_ctx = device.child("port").child("line");
    line->configure(spec.config, line_ctx);
    line->bind(caps);
  }

  // -- send: ROS -> driver -------------------------------------------------------------------
  for (const SendSpec & spec : cfg.sends) {
    Context ctx = device.child(spec.name);
    auto codec = loaders_->codec.create(spec.codec);
    Context codec_ctx = ctx.child("codec");
    codec->configure(spec.codec.config, cfg.packet, codec_ctx);
    if (!codec->can_encode()) {
      spec.config.error("this codec cannot encode, so it cannot be used to send");
    }
    MappingSpec mapping = MappingSpec::from_sources(codec->sources(), codec->schema());
    if (!spec.source && !mapping.paths().empty()) {
      const FieldMapping & m = mapping.paths().front();
      m.config.error("maps message field '" + m.path + "', but '" + spec.name +
        "' has no 'from:' topic");
    }
    if (spec.source) {
      for (const std::size_t i : mapping.unmapped()) {
        ctx.logger().warn(
          "field '{}' has no 'from:' and is always sent as 0", codec->schema()->field(i).name);
      }
    }

    PluginSpec trigger_spec = !spec.source || !spec.stream ?
      internal("simport/OnRequest", "", spec.name) :
      spec.rate ? internal(
      "simport/FixedRate",
      std::format("rate_hz: {}\nstale_timeout: {}\n", *spec.rate, spec.stale_after), spec.name) :
      internal("simport/OnMessage", "", spec.name);
    auto trigger = loaders_->trigger.create(trigger_spec);
    Context trigger_ctx = ctx.child("trigger");
    trigger->configure(trigger_spec.config, trigger_ctx);

    SourcePtr source;
    if (spec.source) {
      source = loaders_->source.create(*spec.source);
      Context source_ctx = ctx.child("from");
      source->configure(spec.source->config, source_ctx, node_, mapping);
    }
    std::vector<FrameFilterPtr> filters;
    if (line) {
      filters.push_back(line);
    }
    auto channel = std::make_unique<OutputChannel>(
      spec.name, ctx, std::move(mapping), std::move(trigger), std::move(codec),
      std::move(filters), transport_);
    sends_.push_back(Send{spec, std::move(source), std::move(channel)});
  }

  // -- receive: driver -> ROS ----------------------------------------------------------------
  std::optional<PluginSpec> framer_spec;
  if (!cfg.receives.empty()) {
    if (caps.stream && !cfg.framer && !cfg.packet.defined()) {
      cfg.receives.front().config.error(
        "this port is a byte stream: give a 'packet:' envelope with 'start' bytes and a length "
        "field, or a 'framer:', so packets can be told apart");
    }
    if (!caps.stream && cfg.framer) {
      cfg.framer->config.error(
        "this port delivers whole packets, so no framer is needed; remove 'framer'");
    }
    if (caps.stream) {
      framer_spec = cfg.framer ? *cfg.framer : PluginSpec{"simport/SyncLength", cfg.packet};
      framer_ = loaders_->framer.create(*framer_spec);
      Context framer_ctx = device.child("framer");
      framer_->configure(framer_spec->config, framer_ctx);
    }
  }
  for (const ReceiveSpec & spec : cfg.receives) {
    const std::size_t index = receives_.size();
    Context ctx = device.child(spec.name);
    auto codec = loaders_->codec.create(spec.codec);
    Context codec_ctx = ctx.child("codec");
    codec->configure(spec.codec.config, cfg.packet, codec_ctx);
    if (!codec->can_decode()) {
      spec.config.error("this codec cannot decode, so it cannot be used to receive");
    }
    auto receive = std::make_unique<Receive>();
    receive->spec = spec;
    receive->index = index;
    if (spec.target == ReceiveSpec::Target::kBehavior) {
      // Handled by behavior_, created below.
    } else if (spec.target == ReceiveSpec::Target::kTopic) {
      receive->publisher = node_.create_publisher<simport_msgs::msg::Record>(
        spec.to, rclcpp::QoS(10));
    } else {
      // A driver may send many packets at once (a motor bus: one per motor per cycle); the
      // default service queue of 10 would drop some of them.
      receive->client = node_.create_client<simport_msgs::srv::Exchange>(
        spec.to, rclcpp::ServicesQoS().keep_last(kExchangeQueueDepth));
      for (const std::string & reply : spec.replies) {
        for (Send & s : sends_) {
          if (s.spec.name == reply) {
            receive->replies.push_back(s.channel.get());
          }
        }
      }
    }
    Receive * r = receive.get();
    receive->channel = std::make_unique<InputChannel>(
      spec.name, ctx, codec, parse_conditions(spec.match, *codec->schema()),
      [this, r](const RecordPtr & record, const Frame & frame) {on_received(*r, record, frame);});
    receives_.push_back(std::move(receive));
  }
  // -- behavior: answers in process ----------------------------------------------------------
  if (cfg.behavior) {
    DeviceMessages messages;
    for (const Send & s : sends_) {
      messages.sends.push_back(MessageInfo{s.spec.name, s.channel->schema()});
    }
    for (const auto & r : receives_) {
      messages.receives.push_back(MessageInfo{r->spec.name, r->channel->codec()->schema()});
    }
    behavior_ = loaders_->behavior.create(*cfg.behavior);
    Context behavior_ctx = device.child("behavior");
    behavior_->configure(cfg.behavior->config, behavior_ctx, node_, messages);
  }

  if (!receives_.empty()) {
    std::vector<InputChannel *> channels;
    for (const auto & r : receives_) {
      channels.push_back(r->channel.get());
    }
    FramerFactory make_framer;
    if (framer_) {
      // One framer per peer (TCP clients); the first is the one already configured.
      make_framer = [this, spec = *framer_spec, ctx = device.child("framer"),
          first = framer_]() mutable -> FramerPtr {
          if (first) {
            return std::exchange(first, nullptr);
          }
          FramerPtr framer = loaders_->framer.create(spec);
          framer->configure(spec.config, ctx);
          return framer;
        };
    }
    router_ = std::make_unique<InboundRouter>(
      device.child("receive"), std::move(make_framer), std::move(channels));
    InboundRouter * router = router_.get();
    transport_->set_receive_callback(
      [router](ByteView data, const FrameMeta & meta, Timestamp stamp) {
        router->on_receive(data, meta, stamp);
      });
  }

  if (open_transport) {
    transport_->open();
  }
}

void Device::on_received(Receive & receive, const RecordPtr & record, const Frame & frame)
{
  if (!running_) {
    return;
  }
  if (receive.spec.target == ReceiveSpec::Target::kBehavior) {
    answers_.clear();
    behavior_->handle(receive.index, *record, answers_);
    for (const Answer & a : answers_) {
      if (a.send < sends_.size()) {
        sends_[a.send].channel->request(a.values, frame.meta);
      }
    }
    return;
  }
  if (receive.publisher) {
    receive.publisher->publish(to_msg(*record, receive.spec.name));
    return;
  }
  if (!receive.client->service_is_ready()) {
    if (receive.unanswered++ == 0) {
      logger_->warn(
        "[{}/{}] no service {} to answer the driver (further cases are counted)", name_,
        receive.spec.name, receive.spec.to);
    }
    return;
  }
  auto request = std::make_shared<simport_msgs::srv::Exchange::Request>();
  request->request = to_msg(*record, receive.spec.name);
  receive.client->async_send_request(
    request,
    [this, replies = receive.replies, meta = frame.meta, name = receive.spec.name](
      rclcpp::Client<simport_msgs::srv::Exchange>::SharedFuture future) {
      if (!running_) {
        return;
      }
      for (const auto & msg : future.get()->replies) {
        if (replies.empty()) {
          logger_->warn(
            "[{}/{}] the service replied, but the receive has no 'reply:' to send it as", name_,
            name);
          return;
        }
        OutputChannel * reply = replies.front();
        if (!msg.name.empty()) {
          const auto it = std::find_if(replies.begin(), replies.end(),
          [&](OutputChannel * c) {return c->name() == msg.name;});
          if (it == replies.end()) {
            logger_->warn(
              "[{}/{}] the reply is named '{}', which is not in this receive's 'reply:' list",
              name_, name, msg.name);
            continue;
          }
          reply = *it;
        }
        std::vector<std::string> unknown;
        const FieldOverrides values = from_msg(msg, *reply->schema(), &unknown);
        if (!unknown.empty()) {
          logger_->warn(
            "[{}/{}] the reply sets '{}', which '{}' does not have", name_, name, unknown.front(),
            reply->name());
        }
        reply->request(values, meta);
      }
    });
}

void Device::start()
{
  if (running_) {
    return;
  }
  for (Send & s : sends_) {
    s.channel->start();
    if (s.source) {
      OutputChannel * channel = s.channel.get();
      s.source->start([channel](RecordPtr record) {channel->push(std::move(record));});
    }
  }
  if (behavior_) {
    behavior_->start();
  }
  running_ = true;
  if (router_) {
    router_->set_active(true);
  }
}

void Device::stop()
{
  if (!running_) {
    return;
  }
  if (router_) {
    router_->set_active(false);
  }
  running_ = false;
  if (behavior_) {
    behavior_->stop();
  }
  for (Send & s : sends_) {
    if (s.source) {
      s.source->stop();  // first, so nothing is pushed into a stopping channel
    }
    s.channel->stop();
  }
}

void Device::close()
{
  stop();
  if (transport_) {
    transport_->close();  // stops the receive thread before the router goes away
  }
  router_.reset();
  behavior_.reset();
  receives_.clear();
  sends_.clear();
  framer_.reset();
  transport_.reset();
  clock_.reset();
}

void Device::set_frame_tap(
  std::function<void(const Frame &, bool outbound, const std::string &)> tap)
{
  for (Send & s : sends_) {
    const std::string name = s.spec.name;
    s.channel->set_frame_tap([tap, name](const Frame & f) {tap(f, true, name);});
  }
  if (router_) {
    router_->set_frame_tap([tap](const Frame & f) {tap(f, false, "received");});
  }
}

}  // namespace simport
