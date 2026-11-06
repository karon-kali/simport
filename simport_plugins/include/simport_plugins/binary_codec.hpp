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

#ifndef SIMPORT_PLUGINS__BINARY_CODEC_HPP_
#define SIMPORT_PLUGINS__BINARY_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "simport_core/checksum.hpp"
#include "simport_core/codec.hpp"

namespace simport::plugins
{

/// Primitive wire types of BinaryCodec.
enum class WireType : std::uint8_t
{
  kU8, kU16, kU32, kU64,
  kI8, kI16, kI32, kI64,
  kF32, kF64,
  kBool,   ///< one byte, 0 or 1
  kPad,    ///< zero bytes, not in the schema
  kBytes,  ///< raw bytes: a constant byte list, or the variable-length rest of the body
  kMeta,   ///< not on the wire: frame metadata (`can_id`)
};

struct WireTypeInfo
{
  WireType type;
  std::string_view name;
  std::size_t size;  ///< 0 when the size is given per field
  ValueKind kind;
};

/// Lookup by YAML spelling (`u8` ... `f64`, `bool`, `bytes`, `meta`).
std::optional<WireTypeInfo> find_wire_type(std::string_view name);

/// Binary packets described field by field.
///
/// A message's `fields` go into the `body` of the port's envelope (`packet:` in the device
/// file), if there is one. One field per line, in wire order:
///
/// ```yaml
/// fields:
///   - seq: u8                                       # a value (from a mapping or a record)
///   - version: { u8: 0x02 }                         # a constant
///   - gyro_z: { i16, from: angular_velocity.z, scale: 1000 }   # `from` & co: the mapping
///   - counter: { u8, counter: true }                # counts up per packet
///   - crc: { u16, checksum: crc16_modbus, over: [version, gyro_z] }
///   - flags: { u8, bits: [ok: 1, mode: 3, spare: 4] }   # bit fields, low bit first
///   - data: bytes                                   # the rest of the body, any length
///   - ids: { u8, repeat: rest }                     # repeated until the body ends
///   - can_id: meta                                  # the CAN identifier, not on the wire
///   - pad: 2                                        # two zero bytes
///   - group: { count: 6, fields: [...] }            # repeated fields (arrays)
/// ```
///
/// The envelope adds `start` (sync bytes), its own fields around `body`, and optional
/// `stuffing` of the body. A field with `length_of: [names]` holds the byte size of those
/// fields (`body` counts the whole, stuffed, body). Checksums cover `over: [first, last]`
/// (default: from the first byte to the field before the checksum).
///
/// Decoding checks the length, every constant, every checksum and length field.
class BinaryCodec : public Codec
{
public:
  void configure(const Config & message, const Config & envelope, Context & context) override;
  SchemaPtr schema() const override {return schema_;}
  bool can_decode() const override {return !frames_auto_ && frames_ == 1;}
  void encode(const Record & record, std::vector<Frame> & out) override;
  DecodeResult decode(const Frame & frame, Record & out) override;
  void reset() override;
  std::vector<std::pair<std::string, Config>> sources() const override {return sources_;}

  /// Bytes of one packet without the variable-length field (all bytes if there is none).
  std::size_t frame_size() const {return before_size_ + body_size_ + after_size_;}

  /// Stuffing: after the `after` bytes, insert `insert` (Dynamixel: FF FF FD -> FF FF FD FD).
  static Bytes stuff(const Bytes & data, const Bytes & after, std::uint8_t insert);
  static std::optional<Bytes> unstuff(ByteView data, const Bytes & after, std::uint8_t insert);

private:
  enum class Segment : std::uint8_t {kBefore, kBody, kAfter};

  /// A value the codec derives from the packet's position in the record.
  struct Computed
  {
    enum class Kind : std::uint8_t {kNone, kFrameIndex, kElementIndex, kFirstElement};
    Kind kind = Kind::kNone;
    double scale = 1.0;
    double offset = 0.0;
    double first = 1.0;
    double other = 0.0;

    bool set() const {return kind != Kind::kNone;}
    Value evaluate(std::size_t frame, std::size_t element) const;
  };

  struct Bits
  {
    std::string name;
    unsigned shift = 0;
    unsigned width = 1;
    bool wrap = false;
    std::optional<std::size_t> index;
    std::optional<std::uint64_t> constant;
    Computed computed;
    bool length = false;  ///< filled in by a LengthField
  };

  struct Group
  {
    std::size_t offset = 0;
    std::size_t size = 0;
    std::size_t count = 0;
  };

  struct Field
  {
    std::string name;
    WireType type = WireType::kU8;
    Segment segment = Segment::kBody;
    std::size_t offset = 0;      ///< in its segment (or group repetition), variable size 0
    std::size_t size = 0;        ///< bytes; for the variable field, bytes per element
    bool big_endian = false;
    bool wrap = false;
    bool after_variable = false;  ///< body field placed after the variable-length field
    std::size_t order = 0;        ///< position among top-level items, for checksum ranges
    std::vector<std::size_t> groups;
    std::optional<std::size_t> index;
    Bytes const_bytes;
    std::optional<std::size_t> counter;
    std::optional<std::size_t> checksum;
    std::optional<std::size_t> length;
    Computed computed;
    std::vector<Bits> bits;

    bool in_group() const {return !groups.empty();}
  };

  struct Counter
  {
    std::uint64_t start = 0;
    std::uint64_t step = 1;
    std::uint64_t value = 0;
    bool per_record = false;
  };

  /// A range of the packet: a field, or the whole body (`field` unset).
  struct Part
  {
    std::optional<std::size_t> field;
  };

  struct ChecksumField
  {
    Checksum algorithm;
    std::size_t field;
    Part from;
    Part to;
  };

  struct LengthField
  {
    std::size_t field;
    std::vector<Part> parts;
    std::optional<std::size_t> bits;  ///< the bit field of `field` holding the length
  };

  struct Pending
  {
    std::size_t field;
    Config node;
    std::optional<Part> previous;  ///< the item before the field (default end of a range)
    std::size_t length = 0;        ///< for a length: its index in lengths_
  };

  struct Layout;  // byte positions of one packet

  void parse_items(
    const Config & items, Segment segment, const std::vector<std::size_t> & groups,
    Schema & schema, bool envelope, bool * body_seen);
  void parse_field(
    const std::string & name, const Config & spec, Segment segment,
    const std::vector<std::size_t> & groups, Schema & schema);
  static Computed parse_computed(const Config & spec, bool in_group);
  Part part_named(const Config & node, const std::string & name) const;
  std::size_t order_of(const Part & part) const;
  void resolve_ranges();

  static void write_value(const Field & field, const Value & value, std::uint8_t * dst);
  static Value read_value(const Field & field, const std::uint8_t * src);
  template<typename Fn>
  void for_each_repetition(const Field & f, std::size_t frame, Fn && fn) const;
  void encode_field(
    const Field & f, const Record & record, std::size_t frame, std::size_t element,
    std::uint8_t * dst);
  std::pair<std::size_t, std::size_t> span(const Part & part, const Layout & layout) const;

  std::vector<Field> fields_;
  std::vector<Group> groups_;
  std::vector<Counter> counters_;
  std::vector<ChecksumField> checksums_;
  std::vector<LengthField> lengths_;
  std::vector<Pending> pending_checksums_;
  std::vector<Pending> pending_lengths_;
  std::vector<std::pair<std::string, Config>> sources_;
  std::optional<std::size_t> variable_;   ///< the variable-length body field
  std::optional<std::size_t> meta_can_id_;  ///< schema index of a `can_id: meta` field
  std::optional<Part> first_item_;
  std::optional<Part> last_item_;
  std::size_t order_ = 0;
  std::size_t body_order_ = 0;
  bool has_envelope_ = false;
  bool default_big_ = false;
  bool body_big_ = false;
  std::size_t before_size_ = 0;
  std::size_t body_size_ = 0;
  std::size_t after_size_ = 0;
  Bytes stuffing_after_;
  std::uint8_t stuffing_insert_ = 0;
  bool stuffing_ = false;
  std::size_t frames_ = 1;
  bool frames_auto_ = false;
  std::vector<std::pair<std::size_t, std::size_t>> group_inputs_;  ///< schema index, per frame
  SchemaPtr schema_;
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__BINARY_CODEC_HPP_
