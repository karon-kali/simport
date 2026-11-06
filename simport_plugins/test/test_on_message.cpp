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

#include <memory>

#include "simport_plugins/on_message.hpp"

namespace simport::plugins
{
namespace
{

TEST(OnMessage, EmitsOnlyWhileStarted)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  OnMessage t;
  t.configure(Config(), ctx);
  int emitted = 0;
  auto r = std::make_shared<const Record>();
  t.on_record(r);
  t.start([&](const RecordPtr & rec) {
      EXPECT_EQ(rec, r);
      ++emitted;
    });
  t.on_record(r);
  t.on_record(r);
  t.stop();
  t.on_record(r);
  EXPECT_EQ(emitted, 2);
}

TEST(OnRequest, NeverEmitsOnItsOwn)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  OnRequest t;
  t.configure(Config::from_string("plugin: simport/OnRequest\n"), ctx);
  int emitted = 0;
  t.start([&](const RecordPtr &) {++emitted;});
  t.on_record(std::make_shared<const Record>());
  t.stop();
  EXPECT_EQ(emitted, 0);
}

TEST(OnMessage, RejectsUnknownKeys)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  OnMessage t;
  EXPECT_THROW(t.configure(Config::from_string("plugin: x\nrate_hz: 5\n"), ctx), ConfigError);
}

}  // namespace
}  // namespace simport::plugins
