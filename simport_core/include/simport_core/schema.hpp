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

#ifndef SIMPORT_CORE__SCHEMA_HPP_
#define SIMPORT_CORE__SCHEMA_HPP_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "simport_core/types.hpp"
#include "simport_core/value.hpp"

namespace simport
{

/// Who provides a field's value when encoding.
enum class FieldRole : std::uint8_t
{
  /// Provided by the source through the device's `map` block.
  kInput,
  /// Produced by the codec itself (constants, counters, checksums, padding). Cannot be mapped.
  kComputed,
};

/// Description of one field of a Schema.
struct FieldSpec
{
  std::string name;
  ValueKind kind = ValueKind::kFloat;
  FieldRole role = FieldRole::kInput;
};

/// Ordered, named set of fields. Codecs publish their Schema; sources and sinks bind to it
/// by name once, at configure time, and then address fields by index.
class Schema
{
public:
  Schema() = default;

  /// Throws std::invalid_argument on empty or duplicate names.
  explicit Schema(std::vector<FieldSpec> fields);

  /// Append a field and return its index. Throws std::invalid_argument on empty or duplicate
  /// names.
  std::size_t add(FieldSpec field);

  std::size_t size() const {return fields_.size();}
  bool empty() const {return fields_.empty();}
  const FieldSpec & field(std::size_t index) const {return fields_.at(index);}
  const std::vector<FieldSpec> & fields() const {return fields_;}

  /// Index of the named field, if present.
  std::optional<std::size_t> find(std::string_view name) const;

  /// Index of the named field. Throws std::out_of_range if absent.
  std::size_t index_of(std::string_view name) const;

  /// Names of all fields with the given role, in order.
  std::vector<std::string> names(std::optional<FieldRole> role = std::nullopt) const;

private:
  std::vector<FieldSpec> fields_;
  std::map<std::string, std::size_t, std::less<>> index_;
};

using SchemaPtr = std::shared_ptr<const Schema>;

/// One set of field values following a Schema. This is what flows between sources, triggers,
/// codecs and sinks.
///
/// A numeric field (bool/int/uint/float kind) may hold any numeric Value alternative, e.g. a
/// scaled mapping yields a double for an integer field. Consumers convert with
/// saturate_cast<T>(). Non-numeric fields hold exactly their kind.
struct Record
{
  SchemaPtr schema;
  std::vector<Value> values;
  /// Time the data was produced (e.g. the source message stamp), on the device clock.
  Timestamp stamp{};

  /// Record with every field set to the default value of its kind.
  static Record make(SchemaPtr schema);

  Value & operator[](std::size_t index) {return values[index];}
  const Value & operator[](std::size_t index) const {return values[index];}

  /// Lookup by name; throws std::out_of_range if absent. Not for the data path.
  Value & at(std::string_view name);
  const Value & at(std::string_view name) const;
};

using RecordPtr = std::shared_ptr<const Record>;

}  // namespace simport

#endif  // SIMPORT_CORE__SCHEMA_HPP_
