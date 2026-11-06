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

#ifndef SIMPORT_CORE__TRIGGER_HPP_
#define SIMPORT_CORE__TRIGGER_HPP_

#include <functional>
#include <memory>

#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/schema.hpp"

namespace simport
{

/// Hands a Record to the rest of the outbound pipeline (codec → filters → transport).
using EmitCallback = std::function<void (const RecordPtr & record)>;

/// Decides when an outbound channel emits, decoupling the device's output timing from the rate
/// at which the simulator produces data.
///
/// Lifecycle: configure() → start() → on_record()* → stop(); start/stop may repeat.
class Trigger
{
public:
  virtual ~Trigger() = default;

  virtual void configure(const Config & config, Context & context) = 0;

  /// Begin emitting through `emit`. Timed triggers start their thread here. `emit` may be
  /// called from that thread or from the thread calling on_record(), but never concurrently.
  virtual void start(EmitCallback emit) = 0;

  /// Stop emitting. Blocks until `emit` is no longer running and will not be called again.
  virtual void stop() = 0;

  /// New data from the source. Thread-safe; must not block for long.
  virtual void on_record(RecordPtr record) = 0;
};

using TriggerPtr = std::shared_ptr<Trigger>;

}  // namespace simport

#endif  // SIMPORT_CORE__TRIGGER_HPP_
