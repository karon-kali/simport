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

#include "simport_plugins/on_message.hpp"

#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "simport_core/clock.hpp"

namespace simport::plugins
{

void OnMessage::configure(const Config & config, Context & /*context*/)
{
  config.expect_keys({"plugin"});
}

void OnMessage::start(EmitCallback emit)
{
  std::lock_guard<std::mutex> lock(mutex_);
  emit_ = std::move(emit);
}

void OnMessage::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  emit_ = nullptr;
}

void OnMessage::on_record(RecordPtr record)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (emit_) {
    emit_(record);
  }
}

void OnRequest::configure(const Config & config, Context & /*context*/)
{
  config.expect_keys({"plugin"});
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::OnMessage, simport::Trigger)
PLUGINLIB_EXPORT_CLASS(simport::plugins::OnRequest, simport::Trigger)
// SystemClock is implemented in simport_core; exported here so it can be selected by name.
PLUGINLIB_EXPORT_CLASS(simport::SystemClock, simport::Clock)
