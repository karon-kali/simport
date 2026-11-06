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

#ifndef SIMPORT_CORE__MAPPING_HPP_
#define SIMPORT_CORE__MAPPING_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "simport_core/config.hpp"
#include "simport_core/schema.hpp"
#include "simport_core/types.hpp"
#include "simport_core/value.hpp"

namespace simport
{

/// Values a mapping can take from the device rather than from the source message. They are
/// evaluated when a frame is emitted, so a fixed-rate output repeating the same source sample
/// still gets fresh timestamps.
enum class Builtin : std::uint8_t
{
  kTimeNs, kTimeUs, kTimeMs, kTimeS,              ///< device clock at emission ($time_*)
  kStampNs, kStampUs, kStampMs, kStampS,          ///< Record::stamp, i.e. data time ($stamp_*)
  kWallTimeNs, kWallTimeUs, kWallTimeMs, kWallTimeS,  ///< system clock at emission
};

/// Parse `$time_us` etc. Returns nullopt for unknown names.
std::optional<Builtin> parse_builtin(std::string_view name);

/// Spelling including the leading `$`.
std::string_view to_string(Builtin builtin);

/// All builtin spellings, for help and error messages.
std::vector<std::string_view> builtin_names();

/// Inputs for evaluating builtins.
struct BuiltinInputs
{
  Timestamp now{};    ///< device clock
  Timestamp wall{};   ///< system clock
  Timestamp stamp{};  ///< Record::stamp
};

/// `*_ns/_us/_ms` builtins are integers (kInt); `*_s` builtins are floats (kFloat).
Value evaluate(Builtin builtin, const BuiltinInputs & inputs);

/// One element of a field path: `name` or `name[index]`.
struct PathElement
{
  std::string name;
  std::optional<std::size_t> index;

  bool operator==(const PathElement &) const = default;
};

/// Parse a message field path such as `pose.position.x` or `ranges[3]` or
/// `poses[0].pose.position`. Throws std::invalid_argument with a description on bad syntax.
std::vector<PathElement> parse_field_path(std::string_view path);

/// Conversion from a source value to a codec value.
///
/// Arrays first have their elements rearranged (`transpose`, `reverse`, `step`, `resize`, in
/// that order); then every number becomes `value * scale + offset`, and a non-finite result
/// (inf, NaN) is replaced by `nonfinite` when set. A single number can become an array with
/// `ramp`: `count` elements `value + i * step`.
struct ValueTransform
{
  struct Ramp
  {
    double step = 0.0;
    std::size_t count = 1;
  };

  std::optional<double> scale;
  std::optional<double> offset;
  std::optional<double> nonfinite;  ///< replacement for inf/NaN results
  std::optional<std::size_t> transpose_rows;  ///< arrays: row-major rows x cols -> column-major
  bool reverse = false;             ///< arrays: last element first
  std::size_t step = 1;             ///< arrays: keep every step-th element, starting with the first
  std::optional<std::size_t> resize;  ///< arrays: pick this many evenly spaced elements
  std::optional<Ramp> ramp;         ///< numbers: expand into an array

  bool identity() const {return !scale && !offset && !nonfinite && !array_ops() && !ramp;}

  /// True if the transform rearranges array elements, so it needs an array source.
  bool array_ops() const
  {
    return transpose_rows.has_value() || reverse || step != 1 || resize.has_value();
  }

  /// Identity passes the value through unchanged (preserving integer precision); otherwise
  /// numbers become doubles and arrays stay arrays. Throws ValueError for non-numeric input, for
  /// scalar input when array_ops(), for array input with `ramp`, and for a transpose whose rows
  /// do not divide the array.
  Value apply(const Value & in) const;
};

/// How one codec input field gets its value.
struct FieldMapping
{
  enum class From : std::uint8_t {kPath, kBuiltin};

  std::size_t target = 0;   ///< index into the codec schema
  std::string target_name;
  From from = From::kPath;
  std::string path;          ///< kPath: source-specific field path, syntax-checked
  std::vector<PathElement> path_elements;
  Builtin builtin = Builtin::kTimeNs;  ///< kBuiltin
  ValueTransform transform;
  Config config;             ///< the field description, for error reporting during binding
};

/// Where the input fields of a send message get their values, from the codec's field
/// descriptions (Codec::sources()):
/// ```yaml
/// - gx: { i16, from: angular_velocity.x, scale: 1000.0 }       # a message path
/// - t:  { u32, from: $time_ms }                                 # a builtin
/// - dist: { u16, from: ranges, reverse: true, nonfinite: 0 }   # array transforms
/// - ts: { u64, from: $stamp_ns, ramp: { step: 97656.25, count: 1024 } }
/// ```
/// Array fields (kind float[]) accept an array, or a single number that every element takes.
class MappingSpec
{
public:
  MappingSpec() = default;

  /// Build the mapping from field descriptions, as reported by Codec::sources(): a field maps
  /// from `from:` (a message path or a $builtin) with optional `scale`, `offset`, `nonfinite`,
  /// `transpose`, `reverse`, `step`, `resize`, `ramp`; or, written `name: path`, from a path.
  /// Descriptions without `from` are not mapped; other keys are the codec's.
  static MappingSpec from_sources(
    const std::vector<std::pair<std::string, Config>> & sources, SchemaPtr schema);

  const SchemaPtr & schema() const {return schema_;}

  /// Mappings the Source must bind (kPath), in schema order.
  const std::vector<FieldMapping> & paths() const {return paths_;}
  /// Mappings applied at emission (kBuiltin).
  const std::vector<FieldMapping> & builtins() const {return builtins_;}
  /// Indices of kInput fields that nothing maps; they keep their default value.
  const std::vector<std::size_t> & unmapped() const {return unmapped_;}

  /// A Record with every field at its default. Sources start from this.
  Record make_record() const;

  /// Fill builtin-mapped fields of `record`.
  void apply_builtins(Record & record, const BuiltinInputs & inputs) const;

private:
  SchemaPtr schema_;
  std::vector<FieldMapping> paths_;
  std::vector<FieldMapping> builtins_;
  std::vector<std::size_t> unmapped_;
};

}  // namespace simport

#endif  // SIMPORT_CORE__MAPPING_HPP_
