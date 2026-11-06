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

#include "simport_plugins/binary_codec.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

namespace
{

constexpr std::array<WireTypeInfo, 13> kWireTypes{{
  {WireType::kU8, "u8", 1, ValueKind::kUint},
  {WireType::kU16, "u16", 2, ValueKind::kUint},
  {WireType::kU32, "u32", 4, ValueKind::kUint},
  {WireType::kU64, "u64", 8, ValueKind::kUint},
  {WireType::kI8, "i8", 1, ValueKind::kInt},
  {WireType::kI16, "i16", 2, ValueKind::kInt},
  {WireType::kI32, "i32", 4, ValueKind::kInt},
  {WireType::kI64, "i64", 8, ValueKind::kInt},
  {WireType::kF32, "f32", 4, ValueKind::kFloat},
  {WireType::kF64, "f64", 8, ValueKind::kFloat},
  {WireType::kBool, "bool", 1, ValueKind::kBool},
  {WireType::kBytes, "bytes", 0, ValueKind::kBytes},
  {WireType::kMeta, "meta", 0, ValueKind::kUint},
}};

/// Keys of a field description that belong to the mapping, not to the codec.
constexpr std::array<const char *, 9> kMappingKeys{
  "from", "scale", "offset", "nonfinite", "reverse", "step", "resize", "transpose", "ramp"};

std::string type_names()
{
  std::string out;
  for (const auto & t : kWireTypes) {
    out += (out.empty() ? "" : ", ") + std::string(t.name);
  }
  return out;
}

bool parse_endian(const Config & node)
{
  const std::string e = node.as<std::string>();
  if (e == "little") {
    return false;
  }
  if (e == "big") {
    return true;
  }
  node.error("expected 'little' or 'big', got '" + e + "'");
}

void store(std::uint8_t * dst, std::uint64_t v, std::size_t n, bool big)
{
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t shift = 8 * (big ? n - 1 - i : i);
    dst[i] = static_cast<std::uint8_t>(v >> shift);
  }
}

std::uint64_t load(const std::uint8_t * src, std::size_t n, bool big)
{
  std::uint64_t v = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t shift = 8 * (big ? n - 1 - i : i);
    v |= static_cast<std::uint64_t>(src[i]) << shift;
  }
  return v;
}

/// A constant for the given wire type, range-checked against it.
Value parse_const(const Config & node, WireType type)
{
  switch (type) {
    case WireType::kU8: return std::uint64_t{node.as<std::uint8_t>()};
    case WireType::kU16: return std::uint64_t{node.as<std::uint16_t>()};
    case WireType::kU32: return std::uint64_t{node.as<std::uint32_t>()};
    case WireType::kU64: return node.as<std::uint64_t>();
    case WireType::kI8: return std::int64_t{node.as<std::int8_t>()};
    case WireType::kI16: return std::int64_t{node.as<std::int16_t>()};
    case WireType::kI32: return std::int64_t{node.as<std::int32_t>()};
    case WireType::kI64: return node.as<std::int64_t>();
    case WireType::kF32:
    case WireType::kF64: return node.as<double>();
    case WireType::kBool: return node.as<bool>();
    default: break;
  }
  node.error("this type cannot have a constant value");
}

/// Low bits of the two's-complement value, like a hardware counter rolling over.
std::uint64_t wrap_bits(const Value & v)
{
  switch (kind_of(v)) {
    case ValueKind::kInt: return static_cast<std::uint64_t>(std::get<std::int64_t>(v));
    case ValueKind::kUint: return std::get<std::uint64_t>(v);
    default: {
        const double d = to_double(v);
        return d >= 0.0 ? saturate_cast<std::uint64_t>(v) :
               static_cast<std::uint64_t>(saturate_cast<std::int64_t>(v));
      }
  }
}

/// Element `i` of an array value (0 past its end); a scalar is the same for every element.
Value element_of(const Value & v, std::size_t i)
{
  if (kind_of(v) != ValueKind::kFloatArray) {
    return v;
  }
  const auto & array = std::get<std::vector<double>>(v);
  return i < array.size() ? array[i] : 0.0;
}

std::uint64_t mask_of(unsigned width)
{
  return width >= 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << width) - 1;
}

/// `name: spec` (one-key map) of a field list item.
std::pair<std::string, Config> single_entry(const Config & item)
{
  if (!item.is_map() || item.size() != 1) {
    item.error("expected one field per line, `name: type` or `name: { type, options }`");
  }
  return item.entries().front();
}

}  // namespace

std::optional<WireTypeInfo> find_wire_type(std::string_view name)
{
  for (const auto & t : kWireTypes) {
    if (t.name == name) {
      return t;
    }
  }
  return std::nullopt;
}

// -- stuffing ----------------------------------------------------------------------------------

Bytes BinaryCodec::stuff(const Bytes & data, const Bytes & after, std::uint8_t insert)
{
  Bytes out;
  out.reserve(data.size() + data.size() / 8);
  for (const std::uint8_t b : data) {
    out.push_back(b);
    if (out.size() >= after.size() &&
      std::equal(after.begin(), after.end(), out.end() - static_cast<std::ptrdiff_t>(after.size())))
    {
      out.push_back(insert);
    }
  }
  return out;
}

std::optional<Bytes> BinaryCodec::unstuff(ByteView data, const Bytes & after, std::uint8_t insert)
{
  Bytes out;
  out.reserve(data.size());
  for (std::size_t i = 0; i < data.size(); ++i) {
    out.push_back(data[i]);
    if (out.size() >= after.size() &&
      std::equal(after.begin(), after.end(), out.end() - static_cast<std::ptrdiff_t>(after.size())))
    {
      if (i + 1 >= data.size() || data[i + 1] != insert) {
        return std::nullopt;  // the pattern must be followed by the stuffing byte
      }
      ++i;
    }
  }
  return out;
}

// -- configuration -----------------------------------------------------------------------------

BinaryCodec::Computed BinaryCodec::parse_computed(const Config & spec, bool in_group)
{
  Computed c;
  if (!spec.is_map()) {
    return c;
  }
  int count = 0;
  for (const char * key : {"frame_index", "element_index", "first_element"}) {
    count += spec.has(key) ? 1 : 0;
  }
  if (count == 0) {
    return c;
  }
  if (count > 1) {
    spec.error("use only one of 'frame_index', 'element_index' or 'first_element'");
  }
  const auto parse_index = [&](const Config & node) {
      if (node.is_map()) {
        node.expect_keys({"scale", "offset"});
        c.scale = node.get<double>("scale", 1.0);
        c.offset = node.get<double>("offset", 0.0);
      } else if (!node.as<bool>()) {
        node.error("use 'true' or '{ scale: S, offset: O }'");
      }
    };
  if (spec.has("frame_index")) {
    c.kind = Computed::Kind::kFrameIndex;
    parse_index(spec["frame_index"]);
  } else if (spec.has("element_index")) {
    if (!in_group) {
      spec["element_index"].error("'element_index' applies to fields inside a 'group'");
    }
    c.kind = Computed::Kind::kElementIndex;
    parse_index(spec["element_index"]);
  } else {
    const Config node = spec["first_element"];
    const auto values = node.as<std::vector<double>>();
    if (values.size() != 2) {
      node.error("expected [value on the first element, value on the others]");
    }
    c.kind = Computed::Kind::kFirstElement;
    c.first = values[0];
    c.other = values[1];
  }
  return c;
}

Value BinaryCodec::Computed::evaluate(std::size_t frame, std::size_t element) const
{
  switch (kind) {
    case Kind::kFrameIndex: return static_cast<double>(frame) * scale + offset;
    case Kind::kElementIndex: return static_cast<double>(element) * scale + offset;
    case Kind::kFirstElement: return element == 0 ? first : other;
    case Kind::kNone: break;
  }
  return 0.0;
}

struct BinaryCodec::Layout
{
  std::size_t var_bytes = 0;   ///< bytes of the variable-length field
  std::size_t body_bytes = 0;  ///< the body's bytes in the packet (after stuffing)
};

void BinaryCodec::parse_items(
  const Config & items, Segment segment, const std::vector<std::size_t> & groups,
  Schema & schema, bool envelope, bool * body_seen)
{
  for (const Config & item : items.items()) {
    if (item.is_scalar() && item.scalar() == "body") {
      if (!envelope || body_seen == nullptr) {
        item.error("'body' marks where messages go; it belongs in the packet envelope only");
      }
      if (*body_seen) {
        item.error("the envelope has 'body' twice");
      }
      *body_seen = true;
      return;  // the caller parses the message's fields and then the rest
    }
    const auto [name, spec] = single_entry(item);
    if (name == "group") {
      if (segment != Segment::kBody) {
        spec.error("groups belong in a message's fields, not in the envelope");
      }
      if (variable_) {
        spec.error("a group cannot be combined with a variable-length field");
      }
      spec.expect_keys({"count", "fields"});
      Group group;
      group.count = spec.get<std::uint32_t>("count");
      if (group.count == 0) {
        spec["count"].error("count must be at least 1");
      }
      group.offset = body_size_;
      const std::size_t index = groups_.size();
      groups_.push_back(group);
      std::vector<std::size_t> inner = groups;
      inner.push_back(index);
      const std::size_t saved = body_size_;
      body_size_ = 0;  // fields inside get offsets within one repetition
      const std::size_t first = fields_.size();
      if (spec.required("fields").items().empty()) {
        spec["fields"].error("a group needs at least one field");
      }
      parse_items(spec["fields"], Segment::kBody, inner, schema, false, nullptr);
      groups_[index].size = body_size_;
      body_size_ = saved + groups_[index].size * groups_[index].count;
      if (groups.empty() && first < fields_.size()) {
        last_item_ = Part{first};
        if (!first_item_) {
          first_item_ = last_item_;
        }
      }
      continue;
    }
    const std::optional<Part> previous = last_item_;
    if (name == "pad") {
      Field pad;
      pad.name = "";
      pad.type = WireType::kPad;
      pad.segment = segment;
      pad.groups = groups;
      pad.size = spec.as<std::uint32_t>();
      if (pad.size == 0) {
        spec.error("padding size must be at least 1");
      }
      std::size_t & seg_size = segment == Segment::kBefore ? before_size_ :
        segment == Segment::kAfter ? after_size_ : body_size_;
      pad.offset = seg_size;
      pad.after_variable = segment == Segment::kBody && variable_.has_value();
      seg_size += pad.size;
      fields_.push_back(std::move(pad));
    } else {
      parse_field(name, spec, segment, groups, schema);
      if (fields_.back().checksum) {
        pending_checksums_.back().previous = previous;
      }
    }
    if (groups.empty()) {
      last_item_ = Part{fields_.size() - 1};
      if (!first_item_) {
        first_item_ = last_item_;
      }
    }
  }
}

void BinaryCodec::parse_field(
  const std::string & name, const Config & spec, Segment segment,
  const std::vector<std::size_t> & groups, Schema & schema)
{
  const bool in_group = !groups.empty();
  std::size_t per_frame = 1;
  for (const std::size_t g : groups) {
    per_frame *= groups_[g].count;
  }

  // The type: `name: u8`, `name: { u8, ... }`, `name: { u8: const, ... }` or `{ type: u8 }`.
  std::string type_name;
  Config type_value;
  if (spec.is_scalar()) {
    type_name = spec.scalar();
  } else if (spec.is_map()) {
    for (const auto & [key, value] : spec.entries()) {
      if (find_wire_type(key)) {
        if (!type_name.empty()) {
          value.error("a field has one type; got '" + type_name + "' and '" + key + "'");
        }
        type_name = key;
        type_value = value;
      }
    }
    if (spec.has("type")) {
      if (!type_name.empty()) {
        spec["type"].error("the type is already given as '" + type_name + "'");
      }
      type_name = spec.get<std::string>("type");
    }
    if (type_name.empty()) {
      spec.error("give the field's type, e.g. { u8, ... }; types: " + type_names());
    }
    std::vector<std::string> allowed{type_name, "type", "value", "endian", "overflow", "counter",
      "checksum", "over", "length_of", "frame_index", "element_index", "first_element", "bits",
      "repeat"};
    allowed.insert(allowed.end(), kMappingKeys.begin(), kMappingKeys.end());
    spec.expect_keys(allowed);
  } else {
    spec.error("expected a type such as u8, or { type, options }");
  }
  const auto info = find_wire_type(type_name);
  if (!info) {
    (spec.is_scalar() ? spec : spec).error(
      "unknown type '" + type_name + "'; types: " + type_names());
  }
  const auto option = [&](std::string_view key) {return spec.is_map() ? spec[key] : Config();};
  const Config value_node = type_value.defined() && !type_value.is_null() ? type_value :
    option("value");

  Field field;
  field.name = name;
  field.type = info->type;
  field.segment = segment;
  field.groups = groups;
  field.big_endian = option("endian").defined() ? parse_endian(option("endian")) :
    (segment == Segment::kBody ? body_big_ : default_big_);
  std::size_t & seg_size = segment == Segment::kBefore ? before_size_ :
    segment == Segment::kAfter ? after_size_ : body_size_;
  field.offset = seg_size;
  field.after_variable = segment == Segment::kBody && variable_.has_value();

  const auto add_to_schema = [&](ValueKind kind, bool computed) {
      try {
        return schema.add(FieldSpec{name, kind,
                   computed ? FieldRole::kComputed : FieldRole::kInput});
      } catch (const std::invalid_argument &) {
        spec.error("duplicate field name '" + name + "'");
      }
    };
  const auto add_source = [&](std::size_t index) {
      if (spec.is_map()) {
        sources_.emplace_back(name, spec);
      }
      if (in_group) {
        group_inputs_.emplace_back(index, per_frame);
      }
    };

  // Not on the wire: frame metadata.
  if (info->type == WireType::kMeta) {
    if (name != "can_id") {
      spec.error("'meta' fields carry frame metadata; the only one is 'can_id' (CAN identifier)");
    }
    if (meta_can_id_) {
      spec.error("duplicate 'can_id' field");
    }
    field.index = add_to_schema(ValueKind::kUint, false);
    meta_can_id_ = field.index;
    add_source(*field.index);
    fields_.push_back(std::move(field));
    return;
  }

  // Raw bytes: a constant byte list, or the variable-length rest of the body.
  if (info->type == WireType::kBytes) {
    if (value_node.defined()) {
      field.const_bytes = value_node.as<Bytes>();
      if (field.const_bytes.empty()) {
        value_node.error("a constant byte list needs at least one byte");
      }
      field.size = field.const_bytes.size();
      field.index = add_to_schema(ValueKind::kBytes, true);
    } else {
      if (segment != Segment::kBody || in_group) {
        spec.error("variable-length 'bytes' belong in a message's fields, outside groups");
      }
      if (variable_) {
        spec.error("a message can have only one variable-length field");
      }
      if (!groups_.empty()) {
        spec.error("a variable-length field cannot be combined with groups");
      }
      field.size = 1;
      field.index = add_to_schema(ValueKind::kBytes, false);
      variable_ = fields_.size();
      add_source(*field.index);
      fields_.push_back(std::move(field));
      return;
    }
    seg_size += field.size;
    fields_.push_back(std::move(field));
    return;
  }

  field.size = info->size;

  // Repeated until the body ends: a variable-length array.
  if (option("repeat").defined()) {
    if (option("repeat").as<std::string>() != "rest") {
      option("repeat").error("expected 'rest' (repeat until the end of the body)");
    }
    if (segment != Segment::kBody || in_group || variable_ || !groups_.empty()) {
      spec.error("'repeat: rest' needs a message field outside groups, and only one per message");
    }
    field.index = add_to_schema(ValueKind::kFloatArray, false);
    variable_ = fields_.size();
    add_source(*field.index);
    fields_.push_back(std::move(field));
    return;
  }

  // Bit fields: the sub-fields are the schema fields.
  if (option("bits").defined()) {
    if (info->kind != ValueKind::kUint) {
      spec.error("bit fields need an unsigned integer type (u8, u16, u32, u64)");
    }
    unsigned shift = 0;
    const auto items = option("bits").items();
    if (items.empty()) {
      option("bits").error("give at least one bit field, e.g. [flag: 1, mode: 3]");
    }
    for (const Config & b : items) {
      const auto [bit_name, bit_spec] = single_entry(b);
      Bits bits;
      bits.name = bit_name;
      std::optional<Config> source;
      if (bit_spec.is_scalar()) {
        bits.width = bit_spec.as<std::uint8_t>();
      } else {
        std::vector<std::string> allowed{"width", "value", "overflow", "frame_index",
          "element_index", "first_element", "length_of"};
        allowed.insert(allowed.end(), kMappingKeys.begin(), kMappingKeys.end());
        bit_spec.expect_keys(allowed);
        bits.width = bit_spec.get<std::uint8_t>("width");
        const std::string overflow = bit_spec.get<std::string>("overflow", "saturate");
        if (overflow != "saturate" && overflow != "wrap") {
          bit_spec["overflow"].error("expected 'saturate' or 'wrap', got '" + overflow + "'");
        }
        bits.wrap = overflow == "wrap";
        bits.computed = parse_computed(bit_spec, in_group);
        if (bit_spec.has("value")) {
          if (bits.computed.set()) {
            bit_spec.error("a bit field is either a constant ('value') or computed, not both");
          }
          bits.constant = bit_spec.get<std::uint64_t>("value");
        }
        if (bit_spec.has("length_of")) {
          if (bits.computed.set() || bits.constant) {
            bit_spec.error("a bit field is one of: a constant, computed, or 'length_of'");
          }
          if (in_group) {
            bit_spec.error("lengths cannot be inside a group");
          }
          bits.length = true;
          pending_lengths_.push_back(
            Pending{fields_.size(), bit_spec["length_of"], std::nullopt, lengths_.size()});
          lengths_.push_back(LengthField{fields_.size(), {}, field.bits.size()});
        }
        source = bit_spec;
      }
      if (bits.width == 0) {
        b.error("width must be at least 1");
      }
      if (bits.constant && bits.width < 64 && *bits.constant >> bits.width != 0) {
        b.error("value does not fit in " + std::to_string(bits.width) + " bits");
      }
      bits.shift = shift;
      shift += bits.width;
      if (shift > 8 * field.size) {
        b.error("bit fields add up to " + std::to_string(shift) + " bits but '" + type_name +
          "' holds " + std::to_string(8 * field.size));
      }
      const bool computed = bits.constant.has_value() || bits.computed.set() || bits.length;
      try {
        bits.index = schema.add(FieldSpec{bits.name, in_group ? ValueKind::kFloatArray :
              ValueKind::kUint, computed ? FieldRole::kComputed : FieldRole::kInput});
      } catch (const std::invalid_argument &) {
        b.error("duplicate field name '" + bits.name + "'");
      }
      if (!computed) {
        if (source) {
          sources_.emplace_back(bits.name, *source);
        }
        if (in_group) {
          group_inputs_.emplace_back(*bits.index, per_frame);
        }
      }
      field.bits.push_back(std::move(bits));
    }
    seg_size += field.size;
    fields_.push_back(std::move(field));
    return;
  }

  const std::string overflow = option("overflow").defined() ?
    option("overflow").as<std::string>() : "saturate";
  if (overflow == "wrap") {
    if (info->kind != ValueKind::kUint && info->kind != ValueKind::kInt) {
      option("overflow").error("'wrap' applies to integer types only");
    }
    field.wrap = true;
  } else if (overflow != "saturate") {
    option("overflow").error("expected 'saturate' or 'wrap', got '" + overflow + "'");
  }
  field.computed = parse_computed(spec, in_group);
  const bool is_const = value_node.defined();
  const bool is_counter = option("counter").defined();
  const bool is_checksum = option("checksum").defined();
  const bool is_length = option("length_of").defined();
  if (int{is_const} + int{is_counter} + int{is_checksum} + int{is_length} +
    int{field.computed.set()} > 1)
  {
    spec.error("a field is only one of: a constant, 'counter', 'checksum', 'length_of' or a "
      "computed index");
  }
  if ((is_checksum || is_length) && in_group) {
    spec.error("checksums and lengths cannot be inside a group");
  }
  if (option("over").defined() && !is_checksum) {
    option("over").error("'over' gives a checksum's range; this field has no 'checksum'");
  }
  const bool computed = is_const || is_counter || is_checksum || is_length ||
    field.computed.set();
  field.index = add_to_schema(in_group ? ValueKind::kFloatArray : info->kind, computed);
  if (!computed) {
    add_source(*field.index);
  }
  if (is_const) {
    field.const_bytes.resize(field.size);
    write_value(field, parse_const(value_node, field.type), field.const_bytes.data());
  }
  if (is_counter) {
    const Config c = option("counter");
    if (info->kind != ValueKind::kUint && info->kind != ValueKind::kInt) {
      c.error("counters must be integer fields");
    }
    Counter counter;
    if (c.is_map()) {
      c.expect_keys({"start", "step", "per"});
      counter.start = c.get<std::uint64_t>("start", 0);
      counter.step = c.get<std::uint64_t>("step", 1);
      const std::string per = c.get<std::string>("per", "packet");
      if (per != "packet" && per != "record") {
        c["per"].error("expected 'packet' or 'record', got '" + per + "'");
      }
      counter.per_record = per == "record";
    } else if (!c.as<bool>()) {
      c.error("use 'counter: true' or 'counter: { start: N, step: N, per: packet|record }'");
    }
    counter.value = counter.start;
    field.counter = counters_.size();
    field.wrap = true;  // counters roll over
    counters_.push_back(counter);
  }
  if (is_checksum) {
    if (info->kind != ValueKind::kUint) {
      option("checksum").error("checksums must be unsigned integer fields (u8, u16, u32, u64)");
    }
    Checksum algorithm = Checksum::parse(option("checksum"), {});
    if (algorithm.width_bytes() > field.size) {
      option("checksum").error(
        "checksum '" + algorithm.name() + "' is " + std::to_string(algorithm.width_bytes()) +
        " bytes but field '" + name + "' holds only " + std::to_string(field.size));
    }
    field.checksum = checksums_.size();
    checksums_.push_back(ChecksumField{std::move(algorithm), fields_.size(), {}, {}});
    pending_checksums_.push_back(Pending{fields_.size(), spec, std::nullopt});
  }
  if (is_length) {
    if (info->kind != ValueKind::kUint) {
      option("length_of").error("lengths must be unsigned integer fields");
    }
    field.length = lengths_.size();
    lengths_.push_back(LengthField{fields_.size(), {}, std::nullopt});
    pending_lengths_.push_back(
      Pending{fields_.size(), option("length_of"), std::nullopt, lengths_.size() - 1});
  }
  field.order = order_++;
  seg_size += field.size;
  fields_.push_back(std::move(field));
}

BinaryCodec::Part BinaryCodec::part_named(const Config & node, const std::string & name) const
{
  if (name == "body") {
    if (!has_envelope_) {
      node.error("'body' is only defined when the port has a packet envelope");
    }
    return Part{};
  }
  for (std::size_t i = 0; i < fields_.size(); ++i) {
    if (fields_[i].name == name) {
      if (fields_[i].in_group()) {
        node.error("'" + name + "' is inside a group; name the fields around the group");
      }
      if (stuffing_ && fields_[i].segment == Segment::kBody) {
        node.error("with stuffing, ranges can name envelope fields and 'body', not '" + name +
          "' inside the body");
      }
      return Part{i};
    }
  }
  node.error("no field named '" + name + "'");
}

std::size_t BinaryCodec::order_of(const Part & part) const
{
  return part.field ? fields_[*part.field].order : body_order_;
}

void BinaryCodec::resolve_ranges()
{
  for (Pending & p : pending_checksums_) {
    ChecksumField & c = checksums_[*fields_[p.field].checksum];
    const Config over = p.node["over"];
    std::optional<Part> from = first_item_;
    std::optional<Part> to = p.previous;
    if (over.defined() && !(over.is_scalar() && over.scalar() == "all")) {
      if (over.is_scalar()) {
        from = part_named(over, over.scalar());
      } else {
        const auto names = over.as<std::vector<std::string>>();
        if (names.size() != 2) {
          over.error("expected [first field, last field], a first field, or 'all'");
        }
        from = part_named(over, names[0]);
        to = part_named(over, names[1]);
      }
    }
    if (!from || !to) {
      p.node.error("a checksum at the start of the packet needs 'over: [first, last]'");
    }
    const std::size_t own = fields_[p.field].order;
    if (order_of(*from) > order_of(*to)) {
      p.node.error("the checksum range is empty: its first field comes after its last");
    }
    if (order_of(*from) <= own && own <= order_of(*to)) {
      p.node.error("the checksum range must not include the checksum itself");
    }
    for (const ChecksumField & other : checksums_) {
      const std::size_t o = fields_[other.field].order;
      if (o > own && order_of(*from) <= o && o <= order_of(*to)) {
        p.node.error("the checksum range includes checksum '" + fields_[other.field].name +
          "', which comes later and is not computed yet");
      }
    }
    c.from = *from;
    c.to = *to;
  }
  for (Pending & p : pending_lengths_) {
    LengthField & l = lengths_[p.length];
    const std::vector<std::string> names = p.node.is_scalar() ?
      std::vector<std::string>{p.node.scalar()} : p.node.as<std::vector<std::string>>();
    if (names.empty()) {
      p.node.error("list the fields whose size this is, e.g. [body, crc]");
    }
    for (const std::string & n : names) {
      l.parts.push_back(part_named(p.node, n));
    }
  }
}

void BinaryCodec::configure(const Config & message, const Config & envelope, Context &)
{
  fields_.clear();
  groups_.clear();
  counters_.clear();
  checksums_.clear();
  lengths_.clear();
  pending_checksums_.clear();
  pending_lengths_.clear();
  sources_.clear();
  group_inputs_.clear();
  variable_.reset();
  meta_can_id_.reset();
  first_item_.reset();
  last_item_.reset();
  before_size_ = body_size_ = after_size_ = 0;
  order_ = 0;
  body_order_ = 0;
  stuffing_ = false;
  frames_ = 1;
  frames_auto_ = false;
  has_envelope_ = envelope.defined() && !envelope.is_null();

  auto schema = std::make_shared<Schema>();
  default_big_ = has_envelope_ && envelope.has("endian") && parse_endian(envelope["endian"]);
  body_big_ = message.has("endian") ? parse_endian(message["endian"]) : default_big_;
  if (!message.required("fields").is_sequence()) {
    message["fields"].error("expected a list of fields, one per line: `- name: type`");
  }

  if (has_envelope_) {
    envelope.expect_keys({"endian", "start", "fields", "stuffing"});
    if (envelope.has("stuffing")) {
      const Config s = envelope["stuffing"];
      s.expect_keys({"after", "insert"});
      stuffing_after_ = s.required("after").as<Bytes>();
      stuffing_insert_ = s.get<std::uint8_t>("insert");
      if (stuffing_after_.empty()) {
        s["after"].error("give the byte pattern after which a byte is inserted");
      }
      stuffing_ = true;
    }
    if (envelope.has("start")) {
      Field start;
      start.name = "start";
      start.type = WireType::kBytes;
      start.segment = Segment::kBefore;
      start.const_bytes = envelope["start"].as<Bytes>();
      if (start.const_bytes.empty()) {
        envelope["start"].error("give at least one start byte");
      }
      start.size = start.const_bytes.size();
      start.order = order_++;
      before_size_ = start.size;
      fields_.push_back(std::move(start));
      first_item_ = last_item_ = Part{0};
    }
    // Envelope fields up to `body`, the message, then the rest of the envelope.
    const Config env_fields = envelope["fields"];
    std::vector<Config> before, after;
    bool body_seen = false;
    for (const Config & item : env_fields.items()) {
      if (item.is_scalar() && item.scalar() == "body") {
        if (body_seen) {
          item.error("the envelope has 'body' twice");
        }
        body_seen = true;
        continue;
      }
      (body_seen ? after : before).push_back(item);
    }
    if (env_fields.defined() && !body_seen) {
      env_fields.error("the envelope needs a 'body' entry where the messages go");
    }
    const auto parse_list = [&](const std::vector<Config> & list, Segment segment) {
        for (const Config & item : list) {
          const std::optional<Part> previous = last_item_;
          const auto [name, spec] = single_entry(item);
          if (name == "group") {
            spec.error("groups belong in a message's fields, not in the envelope");
          }
          if (name == "pad") {
            Field pad;
            pad.type = WireType::kPad;
            pad.segment = segment;
            pad.size = spec.as<std::uint32_t>();
            std::size_t & seg_size = segment == Segment::kBefore ? before_size_ : after_size_;
            pad.offset = seg_size;
            pad.order = order_++;
            seg_size += pad.size;
            fields_.push_back(std::move(pad));
          } else {
            parse_field(name, spec, segment, {}, *schema);
            if (fields_.back().checksum) {
              pending_checksums_.back().previous = previous;
            }
          }
          last_item_ = Part{fields_.size() - 1};
          if (!first_item_) {
            first_item_ = last_item_;
          }
        }
      };
    parse_list(before, Segment::kBefore);
    body_order_ = order_++;
    if (!first_item_) {
      first_item_ = Part{};
    }
    parse_items(message["fields"], Segment::kBody, {}, *schema, false, nullptr);
    last_item_ = Part{};  // the body, as one item
    parse_list(after, Segment::kAfter);
  } else {
    parse_items(message["fields"], Segment::kBody, {}, *schema, false, nullptr);
  }
  if (fields_.empty()) {
    message["fields"].error("a message needs at least one field");
  }

  if (message.has("frames")) {
    const Config frames = message["frames"];
    if (frames.is_scalar() && frames.scalar() == "auto") {
      frames_auto_ = true;
    } else {
      frames_ = frames.as<std::uint32_t>();
      if (frames_ == 0) {
        frames.error("frames must be at least 1, or 'auto'");
      }
    }
    if (groups_.empty() && (frames_auto_ || frames_ != 1)) {
      frames.error("several packets per record need a 'group' of array fields");
    }
  }
  resolve_ranges();
  schema_ = std::move(schema);
}

void BinaryCodec::reset()
{
  for (Counter & c : counters_) {
    c.value = c.start;
  }
}

// -- values ------------------------------------------------------------------------------------

void BinaryCodec::write_value(const Field & field, const Value & v, std::uint8_t * dst)
{
  const bool big = field.big_endian;
  if (field.wrap) {
    store(dst, wrap_bits(v), field.size, big);
    return;
  }
  switch (field.type) {
    case WireType::kU8: store(dst, saturate_cast<std::uint8_t>(v), 1, big); break;
    case WireType::kU16: store(dst, saturate_cast<std::uint16_t>(v), 2, big); break;
    case WireType::kU32: store(dst, saturate_cast<std::uint32_t>(v), 4, big); break;
    case WireType::kU64: store(dst, saturate_cast<std::uint64_t>(v), 8, big); break;
    case WireType::kI8:
      store(dst, static_cast<std::uint8_t>(saturate_cast<std::int8_t>(v)), 1, big);
      break;
    case WireType::kI16:
      store(dst, static_cast<std::uint16_t>(saturate_cast<std::int16_t>(v)), 2, big);
      break;
    case WireType::kI32:
      store(dst, static_cast<std::uint32_t>(saturate_cast<std::int32_t>(v)), 4, big);
      break;
    case WireType::kI64:
      store(dst, static_cast<std::uint64_t>(saturate_cast<std::int64_t>(v)), 8, big);
      break;
    case WireType::kF32:
      store(dst, std::bit_cast<std::uint32_t>(saturate_cast<float>(v)), 4, big);
      break;
    case WireType::kF64:
      store(dst, std::bit_cast<std::uint64_t>(saturate_cast<double>(v)), 8, big);
      break;
    case WireType::kBool: dst[0] = saturate_cast<bool>(v) ? 1 : 0; break;
    default: break;
  }
}

Value BinaryCodec::read_value(const Field & field, const std::uint8_t * src)
{
  const std::uint64_t raw = load(src, field.size, field.big_endian);
  switch (field.type) {
    case WireType::kU8:
    case WireType::kU16:
    case WireType::kU32:
    case WireType::kU64:
      return raw;
    case WireType::kI8: return std::int64_t{static_cast<std::int8_t>(raw)};
    case WireType::kI16: return std::int64_t{static_cast<std::int16_t>(raw)};
    case WireType::kI32: return std::int64_t{static_cast<std::int32_t>(raw)};
    case WireType::kI64: return static_cast<std::int64_t>(raw);
    case WireType::kF32:
      return static_cast<double>(std::bit_cast<float>(static_cast<std::uint32_t>(raw)));
    case WireType::kF64: return std::bit_cast<double>(raw);
    case WireType::kBool: return raw != 0;
    default: break;
  }
  return std::uint64_t{0};
}

template<typename Fn>
void BinaryCodec::for_each_repetition(const Field & f, std::size_t frame, Fn && fn) const
{
  if (!f.in_group()) {
    fn(f.offset, frame);
    return;
  }
  // Repetitions of the enclosing groups, outermost slowest, so elements come in order.
  const auto walk = [&](auto && self, std::size_t level, std::size_t offset,
    std::size_t element) -> void {
      if (level == f.groups.size()) {
        fn(offset + f.offset, element);
        return;
      }
      const Group & g = groups_[f.groups[level]];
      for (std::size_t r = 0; r < g.count; ++r) {
        self(self, level + 1, offset + g.offset + r * g.size, element * g.count + r);
      }
    };
  walk(walk, 0, 0, frame);
}

void BinaryCodec::encode_field(
  const Field & f, const Record & record, std::size_t frame, std::size_t element,
  std::uint8_t * dst)
{
  const auto input = [&](std::size_t index) -> Value {
      return f.in_group() ? element_of(record[index], element) : record[index];
    };
  if (!f.bits.empty()) {
    std::uint64_t word = 0;
    for (const Bits & b : f.bits) {
      const std::uint64_t mask = mask_of(b.width);
      std::uint64_t v = 0;
      if (b.constant) {
        v = *b.constant;
      } else if (b.length) {
        v = 0;  // set once the packet is assembled
      } else {
        const Value value = b.computed.set() ? b.computed.evaluate(frame, element) :
          input(*b.index);
        v = b.wrap ? wrap_bits(value) & mask : std::min(saturate_cast<std::uint64_t>(value), mask);
      }
      word |= (v & mask) << b.shift;
    }
    store(dst, word, f.size, f.big_endian);
  } else if (!f.const_bytes.empty()) {
    std::memcpy(dst, f.const_bytes.data(), f.size);
  } else if (f.counter) {
    write_value(f, Value{counters_[*f.counter].value}, dst);
  } else if (f.computed.set()) {
    write_value(f, f.computed.evaluate(frame, element), dst);
  } else if (f.index && !f.checksum && !f.length) {
    write_value(f, input(*f.index), dst);
  }
}

std::pair<std::size_t, std::size_t> BinaryCodec::span(
  const Part & part, const Layout & layout) const
{
  if (!part.field) {
    return {before_size_, before_size_ + layout.body_bytes};
  }
  const Field & f = fields_[*part.field];
  if (!f.groups.empty()) {
    // A field inside a group stands for the whole (outermost) group, every repetition.
    const Group & g = groups_[f.groups.front()];
    return {before_size_ + g.offset, before_size_ + g.offset + g.size * g.count};
  }
  const std::size_t size = part.field == variable_ ? layout.var_bytes : f.size;
  switch (f.segment) {
    case Segment::kBefore: return {f.offset, f.offset + size};
    case Segment::kAfter: {
        const std::size_t at = before_size_ + layout.body_bytes + f.offset;
        return {at, at + size};
      }
    case Segment::kBody: break;
  }
  const std::size_t at = before_size_ + f.offset + (f.after_variable ? layout.var_bytes : 0);
  return {at, at + size};
}

// -- encoding ----------------------------------------------------------------------------------

void BinaryCodec::encode(const Record & record, std::vector<Frame> & out)
{
  std::size_t frames = frames_;
  if (frames_auto_) {
    frames = 1;
    bool any_array = false;
    for (const auto & [index, per_frame] : group_inputs_) {
      if (kind_of(record[index]) == ValueKind::kFloatArray) {
        const std::size_t n = std::get<std::vector<double>>(record[index]).size();
        const std::size_t needed = (n + per_frame - 1) / per_frame;
        frames = any_array ? std::max(frames, needed) : needed;
        any_array = true;
      }
    }
  }

  // The variable-length field's bytes.
  Bytes variable;
  if (variable_) {
    const Field & v = fields_[*variable_];
    const Value & value = record[*v.index];
    if (v.type == WireType::kBytes) {
      if (kind_of(value) == ValueKind::kBytes) {
        variable = std::get<Bytes>(value);
      }
    } else {
      const std::size_t n = kind_of(value) == ValueKind::kFloatArray ?
        std::get<std::vector<double>>(value).size() : 1;
      variable.resize(n * v.size);
      for (std::size_t i = 0; i < n; ++i) {
        write_value(v, element_of(value, i), variable.data() + i * v.size);
      }
    }
  }

  for (std::size_t frame_number = 0; frame_number < frames; ++frame_number) {
    Bytes body(body_size_ + variable.size(), 0);
    Bytes before(before_size_, 0);
    Bytes after(after_size_, 0);
    for (const Field & f : fields_) {
      if (f.type == WireType::kPad || f.type == WireType::kMeta) {
        continue;
      }
      if (variable_ && &f == &fields_[*variable_]) {
        std::copy(variable.begin(), variable.end(), body.begin() +
          static_cast<std::ptrdiff_t>(f.offset));
        continue;
      }
      std::uint8_t * base = f.segment == Segment::kBefore ? before.data() :
        f.segment == Segment::kAfter ? after.data() : body.data();
      const std::size_t shift = f.after_variable ? variable.size() : 0;
      for_each_repetition(
        f, frame_number, [&](std::size_t offset, std::size_t element) {
          encode_field(f, record, frame_number, element, base + offset + shift);
        });
    }
    if (stuffing_) {
      body = stuff(body, stuffing_after_, stuffing_insert_);
    }
    const Layout layout{variable.size(), body.size()};
    Frame frame;
    frame.data.reserve(before.size() + body.size() + after.size());
    frame.data.insert(frame.data.end(), before.begin(), before.end());
    frame.data.insert(frame.data.end(), body.begin(), body.end());
    frame.data.insert(frame.data.end(), after.begin(), after.end());
    std::uint8_t * packet = frame.data.data();

    for (const LengthField & l : lengths_) {
      std::size_t total = 0;
      for (const Part & p : l.parts) {
        const auto [b, e] = span(p, layout);
        total += e - b;
      }
      const Field & f = fields_[l.field];
      std::uint8_t * at = packet + span(Part{l.field}, layout).first;
      if (l.bits) {
        const Bits & b = f.bits[*l.bits];
        const std::uint64_t mask = mask_of(b.width);
        std::uint64_t word = load(at, f.size, f.big_endian) & ~(mask << b.shift);
        word |= std::min<std::uint64_t>(total, mask) << b.shift;
        store(at, word, f.size, f.big_endian);
      } else {
        store(at, total, f.size, f.big_endian);
      }
    }
    for (const ChecksumField & c : checksums_) {
      const std::size_t begin = span(c.from, layout).first;
      const std::size_t end = span(c.to, layout).second;
      const Field & f = fields_[c.field];
      store(packet + span(Part{c.field}, layout).first,
        c.algorithm.compute(ByteView(packet + begin, end - begin)), f.size, f.big_endian);
    }
    if (meta_can_id_) {
      frame.meta.can_id = saturate_cast<std::uint32_t>(record[*meta_can_id_]);
    }
    for (Counter & c : counters_) {
      if (!c.per_record) {
        c.value += c.step;
      }
    }
    out.push_back(std::move(frame));
  }
  for (Counter & c : counters_) {
    if (c.per_record) {
      c.value += c.step;
    }
  }
}

// -- decoding ----------------------------------------------------------------------------------

DecodeResult BinaryCodec::decode(const Frame & frame, Record & out)
{
  if (!can_decode()) {
    return DecodeResult::failure(
      DecodeResult::Code::kUnsupported, "messages spanning several packets cannot be decoded");
  }
  const std::size_t n = frame.data.size();
  const std::size_t envelope = before_size_ + after_size_;
  if (n < envelope + (stuffing_ ? 0 : body_size_) ||
    (!variable_ && !stuffing_ && n != envelope + body_size_))
  {
    return DecodeResult::failure(
      DecodeResult::Code::kBadLength,
      std::format("expected {}{} bytes, got {}", frame_size(), variable_ ? " or more" : "", n));
  }
  const std::uint8_t * base = frame.data.data();
  Layout layout{0, n - envelope};
  if (!stuffing_) {
    layout.var_bytes = layout.body_bytes - body_size_;
  }

  for (const ChecksumField & c : checksums_) {
    const Field & f = fields_[c.field];
    const std::size_t begin = span(c.from, layout).first;
    const std::size_t end = span(c.to, layout).second;
    const std::uint64_t expected = c.algorithm.compute(ByteView(base + begin, end - begin));
    const std::uint64_t got = load(base + span(Part{c.field}, layout).first, f.size,
        f.big_endian);
    if (expected != got) {
      return DecodeResult::failure(
        DecodeResult::Code::kBadChecksum,
        std::format("checksum '{}' mismatch: expected {:#x}, got {:#x}", f.name, expected, got));
    }
  }

  // The body, unstuffed.
  Bytes unstuffed;
  const std::uint8_t * body = base + before_size_;
  std::size_t body_bytes = layout.body_bytes;
  if (stuffing_) {
    auto u = unstuff(ByteView(body, body_bytes), stuffing_after_, stuffing_insert_);
    if (!u) {
      return DecodeResult::failure(DecodeResult::Code::kBadValue, "invalid byte stuffing");
    }
    unstuffed = std::move(*u);
    body = unstuffed.data();
    body_bytes = unstuffed.size();
    if (body_bytes < body_size_ || (!variable_ && body_bytes != body_size_)) {
      return DecodeResult::failure(DecodeResult::Code::kBadLength, "wrong body length");
    }
  }
  const std::size_t var_bytes = body_bytes - body_size_;
  if (variable_ && var_bytes % fields_[*variable_].size != 0) {
    return DecodeResult::failure(DecodeResult::Code::kBadLength,
             "the variable-length field is not a whole number of elements");
  }
  if (!stuffing_) {
    layout.var_bytes = var_bytes;
  }

  for (const LengthField & l : lengths_) {
    std::size_t total = 0;
    for (const Part & p : l.parts) {
      const auto [b, e] = span(p, layout);
      total += e - b;
    }
    const Field & f = fields_[l.field];
    std::uint64_t stored = load(base + span(Part{l.field}, layout).first, f.size, f.big_endian);
    if (l.bits) {
      const Bits & b = f.bits[*l.bits];
      stored = (stored >> b.shift) & mask_of(b.width);
    }
    if (stored != total) {
      return DecodeResult::failure(DecodeResult::Code::kBadLength,
               "length field '" + f.name + "' does not match the packet");
    }
  }

  for (const Field & f : fields_) {
    if (f.in_group()) {
      if (f.index) {
        out[*f.index] = std::vector<double>{};
      }
      for (const Bits & b : f.bits) {
        out[*b.index] = std::vector<double>{};
      }
    }
  }
  std::optional<DecodeResult> failure;
  for (std::size_t i = 0; i < fields_.size(); ++i) {
    const Field & f = fields_[i];
    if (f.type == WireType::kPad || f.type == WireType::kMeta) {
      continue;
    }
    const std::uint8_t * seg = f.segment == Segment::kBefore ? base :
      f.segment == Segment::kAfter ? base + before_size_ + layout.body_bytes : body;
    if (variable_ && i == *variable_) {
      const std::uint8_t * src = seg + f.offset;
      if (f.type == WireType::kBytes) {
        out[*f.index] = Bytes(src, src + var_bytes);
      } else {
        std::vector<double> values;
        for (std::size_t k = 0; k < var_bytes / f.size; ++k) {
          values.push_back(to_double(read_value(f, src + k * f.size)));
        }
        out[*f.index] = std::move(values);
      }
      continue;
    }
    const std::size_t shift = f.after_variable ? var_bytes : 0;
    const auto store_value = [&](std::size_t index, const Value & v) {
        if (f.in_group()) {
          std::get<std::vector<double>>(out[index]).push_back(to_double(v));
        } else {
          out[index] = v;
        }
      };
    for_each_repetition(
      f, 0, [&](std::size_t offset, std::size_t) {
        if (failure) {
          return;
        }
        const std::uint8_t * src = seg + offset + shift;
        if (!f.const_bytes.empty() && std::memcmp(src, f.const_bytes.data(), f.size) != 0) {
          failure = DecodeResult::failure(
            DecodeResult::Code::kNoMatch, "constant field '" + f.name + "' does not match");
          return;
        }
        if (!f.bits.empty()) {
          const std::uint64_t word = load(src, f.size, f.big_endian);
          for (const Bits & b : f.bits) {
            const std::uint64_t v = (word >> b.shift) & mask_of(b.width);
            if (b.constant && v != *b.constant) {
              failure = DecodeResult::failure(
                DecodeResult::Code::kNoMatch, "constant bit field '" + b.name + "' does not match");
              return;
            }
            store_value(*b.index, Value{v});
          }
        } else if (f.index && f.type != WireType::kBytes) {
          store_value(*f.index, read_value(f, src));
        }
      });
    if (failure) {
      return *failure;
    }
  }
  if (meta_can_id_) {
    out[*meta_can_id_] = std::uint64_t{frame.meta.can_id.value_or(0)};
  }
  out.stamp = frame.stamp;
  return DecodeResult::success();
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::BinaryCodec, simport::Codec)
