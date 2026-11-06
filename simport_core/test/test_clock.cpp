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

#include <chrono>
#include <memory>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "simport_core/clock.hpp"
#include "simport_core/context.hpp"
#include "simport_core/logger.hpp"

using namespace std::chrono_literals;

namespace simport
{
namespace
{

TEST(SystemClock, SleepsUntilDeadline)
{
  SystemClock clock;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(clock.sleep_for(20ms, std::stop_token{}));
  EXPECT_GE(std::chrono::steady_clock::now() - start, 20ms);
}

TEST(SystemClock, PastDeadlineReturnsImmediately)
{
  SystemClock clock;
  EXPECT_TRUE(clock.sleep_until(clock.now() - 1s, std::stop_token{}));
}

TEST(SystemClock, StopInterruptsSleep)
{
  SystemClock clock;
  std::stop_source stop;
  const auto start = std::chrono::steady_clock::now();
  std::jthread stopper([&] {
      std::this_thread::sleep_for(20ms);
      stop.request_stop();
    });
  EXPECT_FALSE(clock.sleep_for(10s, stop.get_token()));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 5s);
}

class CaptureLogger : public Logger
{
public:
  void log(LogLevel, std::string_view message) override {lines.emplace_back(message);}
  bool enabled(LogLevel) const override {return true;}
  std::vector<std::string> lines;
};

TEST(Context, ChildScopesPrefixLogMessages)
{
  auto logger = std::make_shared<CaptureLogger>();
  Context root("imu0", logger, std::make_shared<SystemClock>());
  Context codec = root.child("imu_data").child("codec");
  EXPECT_EQ(codec.scope(), "imu0/imu_data/codec");
  EXPECT_EQ(codec.device_name(), "imu0");
  codec.logger().warn("value {} clipped", 42);
  ASSERT_EQ(logger->lines.size(), 1u);
  // Nested scopes are not double-prefixed.
  EXPECT_EQ(logger->lines[0], "[imu0/imu_data/codec] value 42 clipped");
}

}  // namespace
}  // namespace simport
