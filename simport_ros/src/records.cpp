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

#include "simport_ros/records.hpp"

#include <string>
#include <utility>
#include <vector>

namespace simport
{

simport_msgs::msg::Record to_msg(const Record & record, const std::string & name)
{
  using Field = simport_msgs::msg::RecordField;
  simport_msgs::msg::Record msg;
  msg.header.stamp.sec = static_cast<std::int32_t>(record.stamp.count() / 1'000'000'000);
  msg.header.stamp.nanosec = static_cast<std::uint32_t>(record.stamp.count() % 1'000'000'000);
  msg.name = name;
  msg.fields.reserve(record.values.size());
  for (std::size_t i = 0; i < record.values.size(); ++i) {
    const Value & v = record.values[i];
    Field f;
    f.name = record.schema->field(i).name;
    f.text = to_display_string(v);
    switch (kind_of(v)) {
      case ValueKind::kBool: f.kind = Field::KIND_BOOL; break;
      case ValueKind::kInt: f.kind = Field::KIND_INT; break;
      case ValueKind::kUint: f.kind = Field::KIND_UINT; break;
      case ValueKind::kFloat: f.kind = Field::KIND_FLOAT; break;
      case ValueKind::kString: f.kind = Field::KIND_STRING; break;
      case ValueKind::kBytes:
        f.kind = Field::KIND_BYTES;
        f.bytes = std::get<Bytes>(v);
        break;
      case ValueKind::kFloatArray:
        f.kind = Field::KIND_FLOAT_ARRAY;
        f.array = std::get<std::vector<double>>(v);
        break;
    }
    if (is_numeric(kind_of(v))) {
      f.number = to_double(v);
    }
    msg.fields.push_back(std::move(f));
  }
  return msg;
}

FieldOverrides from_msg(
  const simport_msgs::msg::Record & msg, const Schema & schema, std::vector<std::string> * unknown)
{
  using Field = simport_msgs::msg::RecordField;
  FieldOverrides out;
  for (const Field & f : msg.fields) {
    const auto index = schema.find(f.name);
    if (!index) {
      if (unknown != nullptr) {
        unknown->push_back(f.name);
      }
      continue;
    }
    const ValueKind kind = schema.field(*index).kind;
    Value v;
    switch (kind) {
      case ValueKind::kBytes:
        if (f.kind == Field::KIND_FLOAT_ARRAY) {  // a list of numbers, one per byte
          Bytes bytes;
          for (const double b : f.array) {
            bytes.push_back(saturate_cast<std::uint8_t>(Value{b}));
          }
          v = std::move(bytes);
        } else {
          v = Bytes(f.bytes.begin(), f.bytes.end());
        }
        break;
      case ValueKind::kFloatArray:
        v = f.kind == Field::KIND_FLOAT_ARRAY ? Value{f.array} : Value{f.number};
        break;
      case ValueKind::kString: v = f.text; break;
      case ValueKind::kBool: v = f.number != 0.0; break;
      case ValueKind::kInt: v = saturate_cast<std::int64_t>(Value{f.number}); break;
      case ValueKind::kUint: v = saturate_cast<std::uint64_t>(Value{f.number}); break;
      case ValueKind::kFloat: v = f.number; break;
    }
    out.emplace_back(*index, std::move(v));
  }
  return out;
}

}  // namespace simport
