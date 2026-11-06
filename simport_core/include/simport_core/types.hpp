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

#ifndef SIMPORT_CORE__TYPES_HPP_
#define SIMPORT_CORE__TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <span>
#include <vector>

namespace simport
{

/// A span of time.
using Duration = std::chrono::nanoseconds;

/// A point in time on a device clock (see simport::Clock), expressed as the time since that
/// clock's epoch. With a simulated clock this is simulation time.
using Timestamp = std::chrono::nanoseconds;

/// Owned bytes.
using Bytes = std::vector<std::uint8_t>;

/// Non-owning view of bytes.
using ByteView = std::span<const std::uint8_t>;

}  // namespace simport

#endif  // SIMPORT_CORE__TYPES_HPP_
