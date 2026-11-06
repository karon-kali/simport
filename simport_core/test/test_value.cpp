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

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "simport_core/value.hpp"

namespace simport
{
namespace
{

TEST(Value, KindMatchesAlternative)
{
  EXPECT_EQ(kind_of(Value{true}), ValueKind::kBool);
  EXPECT_EQ(kind_of(Value{std::int64_t{-1}}), ValueKind::kInt);
  EXPECT_EQ(kind_of(Value{std::uint64_t{1}}), ValueKind::kUint);
  EXPECT_EQ(kind_of(Value{1.5}), ValueKind::kFloat);
  EXPECT_EQ(kind_of(Value{std::string("x")}), ValueKind::kString);
  EXPECT_EQ(kind_of(Value{Bytes{1, 2}}), ValueKind::kBytes);
  EXPECT_EQ(kind_of(Value{std::vector<double>{1.0}}), ValueKind::kFloatArray);
}

TEST(Value, KindNamesRoundTrip)
{
  for (auto k : {ValueKind::kBool, ValueKind::kInt, ValueKind::kUint, ValueKind::kFloat,
      ValueKind::kString, ValueKind::kBytes, ValueKind::kFloatArray})
  {
    EXPECT_EQ(parse_value_kind(to_string(k)), k);
    EXPECT_EQ(kind_of(default_value(k)), k);
  }
  EXPECT_FALSE(parse_value_kind("complex").has_value());
}

TEST(Value, SaturateCastRoundsAndClamps)
{
  EXPECT_EQ(saturate_cast<std::int16_t>(Value{1.5}), 2);
  EXPECT_EQ(saturate_cast<std::int16_t>(Value{-1.5}), -2);
  EXPECT_EQ(saturate_cast<std::int16_t>(Value{1e9}), std::numeric_limits<std::int16_t>::max());
  EXPECT_EQ(saturate_cast<std::int16_t>(Value{-1e9}), std::numeric_limits<std::int16_t>::min());
  EXPECT_EQ(saturate_cast<std::uint8_t>(Value{-3.0}), 0);
  EXPECT_EQ(saturate_cast<std::uint8_t>(Value{std::int64_t{300}}), 255);
  EXPECT_EQ(saturate_cast<std::int8_t>(Value{std::uint64_t{200}}), 127);
  EXPECT_EQ(saturate_cast<std::int32_t>(Value{std::numeric_limits<double>::quiet_NaN()}), 0);
  EXPECT_EQ(saturate_cast<std::uint8_t>(Value{true}), 1);
}

TEST(Value, SaturateCast64BitBounds)
{
  EXPECT_EQ(saturate_cast<std::int64_t>(Value{1e30}), std::numeric_limits<std::int64_t>::max());
  EXPECT_EQ(saturate_cast<std::int64_t>(Value{-1e30}), std::numeric_limits<std::int64_t>::min());
  EXPECT_EQ(saturate_cast<std::uint64_t>(Value{1e30}),
    std::numeric_limits<std::uint64_t>::max());
  // Integer alternatives pass through exactly, beyond double precision.
  const std::uint64_t big = (std::uint64_t{1} << 63) + 1;
  EXPECT_EQ(saturate_cast<std::uint64_t>(Value{big}), big);
}

TEST(Value, SaturateCastToFloatingAndBool)
{
  EXPECT_FLOAT_EQ(saturate_cast<float>(Value{std::int64_t{-7}}), -7.0f);
  EXPECT_DOUBLE_EQ(saturate_cast<double>(Value{0.25}), 0.25);
  EXPECT_TRUE(saturate_cast<bool>(Value{0.1}));
  EXPECT_FALSE(saturate_cast<bool>(Value{std::uint64_t{0}}));
}

TEST(Value, NonNumericConversionThrows)
{
  EXPECT_THROW(to_double(Value{std::string("1")}), ValueError);
  EXPECT_THROW(saturate_cast<int>(Value{Bytes{}}), ValueError);
}

TEST(Value, DisplayString)
{
  EXPECT_EQ(to_display_string(Value{false}), "false");
  EXPECT_EQ(to_display_string(Value{std::int64_t{-12}}), "-12");
  EXPECT_EQ(to_display_string(Value{0.5}), "0.5");
  EXPECT_EQ(to_display_string(Value{Bytes{0x0a, 0xff}}), "0a ff");
  EXPECT_EQ(to_display_string(Value{std::vector<double>{1.0, 2.5}}), "[1, 2.5]");
}

}  // namespace
}  // namespace simport
