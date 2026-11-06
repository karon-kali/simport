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

#include "simport_core/value.hpp"


#include <array>
#include <format>
#include <string>
#include <utility>

namespace simport
{

namespace
{
constexpr std::array<std::pair<ValueKind, std::string_view>, 7> kKindNames{{
  {ValueKind::kBool, "bool"},
  {ValueKind::kInt, "int"},
  {ValueKind::kUint, "uint"},
  {ValueKind::kFloat, "float"},
  {ValueKind::kString, "string"},
  {ValueKind::kBytes, "bytes"},
  {ValueKind::kFloatArray, "float[]"},
}};
}  // namespace

std::string_view to_string(ValueKind kind)
{
  for (const auto & [k, name] : kKindNames) {
    if (k == kind) {
      return name;
    }
  }
  return "unknown";
}

std::optional<ValueKind> parse_value_kind(std::string_view name)
{
  for (const auto & [k, n] : kKindNames) {
    if (n == name) {
      return k;
    }
  }
  return std::nullopt;
}

Value default_value(ValueKind kind)
{
  switch (kind) {
    case ValueKind::kBool:
      return false;
    case ValueKind::kInt:
      return std::int64_t{0};
    case ValueKind::kUint:
      return std::uint64_t{0};
    case ValueKind::kFloat:
      return 0.0;
    case ValueKind::kString:
      return std::string{};
    case ValueKind::kBytes:
      return Bytes{};
    case ValueKind::kFloatArray:
      return std::vector<double>{};
  }
  return 0.0;
}

double to_double(const Value & v)
{
  switch (kind_of(v)) {
    case ValueKind::kBool:
      return std::get<bool>(v) ? 1.0 : 0.0;
    case ValueKind::kInt:
      return static_cast<double>(std::get<std::int64_t>(v));
    case ValueKind::kUint:
      return static_cast<double>(std::get<std::uint64_t>(v));
    case ValueKind::kFloat:
      return std::get<double>(v);
    default:
      throw ValueError("cannot convert " + std::string(to_string(kind_of(v))) +
        " value to a number");
  }
}

std::string to_display_string(const Value & v)
{
  switch (kind_of(v)) {
    case ValueKind::kBool:
      return std::get<bool>(v) ? "true" : "false";
    case ValueKind::kInt:
      return std::to_string(std::get<std::int64_t>(v));
    case ValueKind::kUint:
      return std::to_string(std::get<std::uint64_t>(v));
    case ValueKind::kFloat:
      return std::format("{}", std::get<double>(v));
    case ValueKind::kString:
      return std::get<std::string>(v);
    case ValueKind::kBytes: {
        std::string out;
        for (const auto b : std::get<Bytes>(v)) {
          out += std::format("{}{:02x}", out.empty() ? "" : " ", b);
        }
        return out;
      }
    case ValueKind::kFloatArray: {
        std::string out = "[";
        for (const double d : std::get<std::vector<double>>(v)) {
          out += std::format("{}{}", out.size() > 1 ? ", " : "", d);
        }
        return out + "]";
      }
  }
  return {};
}

}  // namespace simport
