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

#ifndef SIMPORT_CORE__FRAME_FILTER_HPP_
#define SIMPORT_CORE__FRAME_FILTER_HPP_

#include <functional>
#include <memory>

#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/frame.hpp"
#include "simport_core/transport.hpp"

namespace simport
{

using FrameSink = std::function<void (Frame && frame)>;

/// Models what happens to outbound frames on the wire: line rate, latency, loss, corruption.
/// Filters form a chain between the codec and the transport.
class FrameFilter
{
public:
  virtual ~FrameFilter() = default;

  virtual void configure(const Config & config, Context & context) = 0;

  /// Called once after configure(), with the capabilities of the transport at the end of the
  /// chain.
  virtual void bind(const TransportCaps & caps) {(void)caps;}

  virtual void start() {}

  /// After return, process() is not running and `next` will not be called.
  virtual void stop() {}

  /// Pass zero or more frames on to `next`. May block (e.g. to model line rate) or defer
  /// delivery to its own thread.
  virtual void process(Frame && frame, const FrameSink & next) = 0;
};

using FrameFilterPtr = std::shared_ptr<FrameFilter>;

}  // namespace simport

#endif  // SIMPORT_CORE__FRAME_FILTER_HPP_
