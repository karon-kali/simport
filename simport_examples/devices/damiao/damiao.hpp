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

#ifndef DEVICES__DAMIAO__DAMIAO_HPP_
#define DEVICES__DAMIAO__DAMIAO_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string_view>

namespace simport_examples::damiao
{

/// Factory limits of a DM-J motor: position (rad), velocity (rad/s), torque (Nm).
struct Limits
{
  double p_max;
  double v_max;
  double t_max;
};

/// Limits of a motor type such as "DM4310", or nothing for an unknown type.
std::optional<Limits> limits_of(std::string_view type);

/// Map [lo, hi] onto an unsigned integer of `bits` bits, as the motor firmware does.
inline std::uint32_t to_uint(double x, double lo, double hi, int bits)
{
  x = std::clamp(x, lo, hi);
  return static_cast<std::uint32_t>((x - lo) / (hi - lo) * static_cast<double>((1u << bits) - 1));
}

inline double from_uint(std::uint64_t u, double lo, double hi, int bits)
{
  return static_cast<double>(u) / static_cast<double>((1u << bits) - 1) * (hi - lo) + lo;
}

}  // namespace simport_examples::damiao

#endif  // DEVICES__DAMIAO__DAMIAO_HPP_
