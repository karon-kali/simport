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

#include "simport_core/logger.hpp"

#include <iostream>
#include <string>

namespace simport
{

std::string_view to_string(LogLevel level)
{
  switch (level) {
    case LogLevel::kDebug:
      return "debug";
    case LogLevel::kInfo:
      return "info";
    case LogLevel::kWarn:
      return "warn";
    case LogLevel::kError:
      return "error";
  }
  return "unknown";
}

void StderrLogger::log(LogLevel level, std::string_view message)
{
  if (!enabled(level)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::cerr << '[' << to_string(level) << "] " << message << '\n';
}

void ScopedLogger::log(LogLevel level, std::string_view message)
{
  std::string line = prefix_;
  line += message;
  inner_->log(level, line);
}

}  // namespace simport
