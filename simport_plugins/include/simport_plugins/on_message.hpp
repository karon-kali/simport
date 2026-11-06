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

#ifndef SIMPORT_PLUGINS__ON_MESSAGE_HPP_
#define SIMPORT_PLUGINS__ON_MESSAGE_HPP_

#include <mutex>

#include "simport_core/trigger.hpp"

namespace simport::plugins
{

/// Emits every record as soon as the source delivers it, on the source's thread.
///
/// ```yaml
/// trigger: { plugin: simport/OnMessage }   # the default when `trigger` is omitted
/// ```
class OnMessage : public Trigger
{
public:
  void configure(const Config & config, Context & context) override;
  void start(EmitCallback emit) override;
  void stop() override;
  void on_record(RecordPtr record) override;

private:
  std::mutex mutex_;  // held while emitting: serializes emits and makes stop() wait for them
  EmitCallback emit_;
};

/// Never emits by itself: the output is only sent when an input action requests it (a reply to
/// a poll or a command acknowledgement).
///
/// ```yaml
/// trigger: { plugin: simport/OnRequest }
/// ```
class OnRequest : public Trigger
{
public:
  void configure(const Config & config, Context & context) override;
  void start(EmitCallback) override {}
  void stop() override {}
  void on_record(RecordPtr) override {}
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__ON_MESSAGE_HPP_
