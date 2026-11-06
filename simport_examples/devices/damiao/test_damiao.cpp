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

#include "damiao.hpp"

namespace simport_examples::damiao
{
namespace
{

TEST(Damiao, ValuePacking)
{
  const Limits l = *limits_of("DM4310");
  EXPECT_EQ(l.v_max, 30.0);
  EXPECT_EQ(to_uint(0.0, -l.p_max, l.p_max, 16), 32767u);
  EXPECT_EQ(to_uint(100.0, -l.p_max, l.p_max, 16), 65535u);  // clamped
  EXPECT_NEAR(from_uint(to_uint(1.25, -l.p_max, l.p_max, 16), -l.p_max, l.p_max, 16), 1.25, 1e-3);
  EXPECT_DOUBLE_EQ(from_uint(4095, -l.v_max, l.v_max, 12), l.v_max);
  EXPECT_FALSE(limits_of("DM9999"));
}

}  // namespace
}  // namespace simport_examples::damiao
