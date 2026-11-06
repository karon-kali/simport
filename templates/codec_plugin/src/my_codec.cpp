// Copyright 2026 Your Name
//
// Template: a simport codec plugin.
//
// Use a plugin when the YAML `fields:` cannot describe a packet: here, a proprietary check
// byte. The packet (4 bytes):
//
//   AA | value (u16, little endian) | check = 0xFF - (sum of the value bytes)
//
// In a device file:
//
//   send:
//     - name: range
//       from: { topic: /range, type: sensor_msgs/msg/Range }
//       codec:
//         plugin: my_simport_codec/MyCodec
//         fields: { value: { from: range, scale: 1000.0 } }   # where `value` comes from
//
// The same codec decodes the packet in a `receive:`. Rename the class, the fields and the
// bytes for your protocol; keep the structure.

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "simport_core/codec.hpp"

namespace my_simport_codec
{

class MyCodec : public simport::Codec
{
public:
  void configure(
    const simport::Config & message, const simport::Config & envelope,
    simport::Context & /*context*/) override
  {
    // Check every key, so that typos are errors pointing at the YAML line.
    message.expect_keys({"plugin", "fields"});
    if (envelope.defined()) {
      envelope.error("MyCodec makes complete packets; remove the device's 'packet:' envelope");
    }

    // The fields of a packet: what a `send` fills in from ROS, or a `receive` decodes.
    schema_ = std::make_shared<const simport::Schema>(std::vector<simport::FieldSpec>{
        {"value", simport::ValueKind::kUint, simport::FieldRole::kInput}});

    // Where the fields get their values: `fields: { name: { from: ..., scale: ... } }`.
    // simport reads `from`, `scale`, `offset`, ... from these descriptions.
    sources_.clear();
    for (const auto & [name, description] : message["fields"].entries()) {
      if (!schema_->find(name)) {
        description.error("MyCodec has no field '" + name + "'; its field is: value");
      }
      sources_.emplace_back(name, description);
    }
  }

  std::vector<std::pair<std::string, simport::Config>> sources() const override
  {
    return sources_;
  }

  simport::SchemaPtr schema() const override {return schema_;}

  void encode(const simport::Record & record, std::vector<simport::Frame> & out) override
  {
    // Values may arrive as any numeric kind; saturate_cast converts and clamps.
    const auto value = simport::saturate_cast<std::uint16_t>(record[0]);
    simport::Bytes packet{0xAA, static_cast<std::uint8_t>(value),
      static_cast<std::uint8_t>(value >> 8), 0};
    packet[3] = check(packet[1], packet[2]);
    out.push_back(simport::Frame{std::move(packet), {}, {}});
  }

  bool can_decode() const override {return true;}

  simport::DecodeResult decode(const simport::Frame & frame, simport::Record & out) override
  {
    using simport::DecodeResult;
    const simport::Bytes & p = frame.data;
    if (p.size() != 4) {
      return DecodeResult::failure(DecodeResult::Code::kBadLength, "expected 4 bytes");
    }
    if (p[0] != 0xAA) {
      return DecodeResult::failure(DecodeResult::Code::kNoMatch);  // another packet type
    }
    if (p[3] != check(p[1], p[2])) {
      return DecodeResult::failure(DecodeResult::Code::kBadChecksum);
    }
    out[0] = std::uint64_t{static_cast<std::uint64_t>(p[1] | (p[2] << 8))};
    return DecodeResult::success();
  }

private:
  static std::uint8_t check(std::uint8_t a, std::uint8_t b)
  {
    return static_cast<std::uint8_t>(0xFF - ((a + b) & 0xFF));
  }

  simport::SchemaPtr schema_;
  std::vector<std::pair<std::string, simport::Config>> sources_;
};

}  // namespace my_simport_codec

PLUGINLIB_EXPORT_CLASS(my_simport_codec::MyCodec, simport::Codec)
