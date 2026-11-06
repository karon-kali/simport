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

#ifndef SIMPORT_CORE__VALUE_HPP_
#define SIMPORT_CORE__VALUE_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "simport_core/types.hpp"

namespace simport
{

/// Logical kind of a Value. The enumerator order matches the alternatives of simport::Value.
enum class ValueKind : std::uint8_t
{
  kBool,
  kInt,
  kUint,
  kFloat,
  kString,
  kBytes,
  kFloatArray,
};

/// A dynamically typed field value: the currency of simport::Record.
using Value = std::variant<
  bool, std::int64_t, std::uint64_t, double, std::string, Bytes, std::vector<double>>;

/// Thrown when a Value cannot be converted to the requested type.
class ValueError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Kind of the alternative currently held by `v`.
inline ValueKind kind_of(const Value & v) {return static_cast<ValueKind>(v.index());}

/// True for kBool, kInt, kUint and kFloat.
inline bool is_numeric(ValueKind k) {return k <= ValueKind::kFloat;}

/// Lower-case name of a kind ("bool", "int", "uint", "float", "string", "bytes", "float[]").
std::string_view to_string(ValueKind kind);

/// Inverse of to_string(ValueKind).
std::optional<ValueKind> parse_value_kind(std::string_view name);

/// Zero / empty value of the given kind.
Value default_value(ValueKind kind);

/// Numeric value as double. Throws ValueError for non-numeric kinds.
double to_double(const Value & v);

/// Human-readable rendering, e.g. for logs and debug messages.
std::string to_display_string(const Value & v);

/// Convert a numeric Value to arithmetic type T with saturation.
///
/// - Integral targets: floats are rounded to nearest (half away from zero) and clamped to T's
///   range; NaN becomes 0. Integers are clamped to T's range.
/// - Floating-point targets: plain conversion.
/// - bool target: non-zero is true.
///
/// Throws ValueError for non-numeric kinds.
template<typename T>
T saturate_cast(const Value & v)
{
  static_assert(std::is_arithmetic_v<T>, "saturate_cast requires an arithmetic target type");
  using Lim = std::numeric_limits<T>;

  if constexpr (std::is_same_v<T, bool>) {
    return to_double(v) != 0.0;
  } else if constexpr (std::is_floating_point_v<T>) {
    return static_cast<T>(to_double(v));
  } else {
    auto clamp_int = [](auto x) -> T {
        if (std::cmp_less(x, Lim::min())) {return Lim::min();}
        if (std::cmp_greater(x, Lim::max())) {return Lim::max();}
        return static_cast<T>(x);
      };
    switch (kind_of(v)) {
      case ValueKind::kBool:
        return static_cast<T>(std::get<bool>(v) ? 1 : 0);
      case ValueKind::kInt:
        return clamp_int(std::get<std::int64_t>(v));
      case ValueKind::kUint:
        return clamp_int(std::get<std::uint64_t>(v));
      case ValueKind::kFloat: {
          const double d = std::get<double>(v);
          if (std::isnan(d)) {return T{0};}
          const double r = std::round(d);
          // Compare in long double so that the bounds of 64-bit types are represented exactly.
          if (static_cast<long double>(r) <= static_cast<long double>(Lim::min())) {
            return Lim::min();
          }
          if (static_cast<long double>(r) >= static_cast<long double>(Lim::max())) {
            return Lim::max();
          }
          return static_cast<T>(r);
        }
      default:
        throw ValueError(
                "cannot convert " + std::string(to_string(kind_of(v))) + " value to a number");
    }
  }
}

}  // namespace simport

#endif  // SIMPORT_CORE__VALUE_HPP_
