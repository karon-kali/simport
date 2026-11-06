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

#ifndef SIMPORT_CORE__LOGGER_HPP_
#define SIMPORT_CORE__LOGGER_HPP_

#include <cstdint>
#include <format>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

namespace simport
{

enum class LogLevel : std::uint8_t {kDebug, kInfo, kWarn, kError};

std::string_view to_string(LogLevel level);

/// Logging sink used by plugins. simport_ros forwards to rclcpp logging; the core library
/// provides StderrLogger for tools and tests.
class Logger
{
public:
  virtual ~Logger() = default;

  virtual void log(LogLevel level, std::string_view message) = 0;

  /// Cheap pre-check so callers can skip formatting.
  virtual bool enabled(LogLevel level) const {return level >= LogLevel::kInfo;}

  template<typename ... Args>
  void debug(std::format_string<Args...> fmt, Args && ... args)
  {
    emit(LogLevel::kDebug, fmt, std::forward<Args>(args)...);
  }
  template<typename ... Args>
  void info(std::format_string<Args...> fmt, Args && ... args)
  {
    emit(LogLevel::kInfo, fmt, std::forward<Args>(args)...);
  }
  template<typename ... Args>
  void warn(std::format_string<Args...> fmt, Args && ... args)
  {
    emit(LogLevel::kWarn, fmt, std::forward<Args>(args)...);
  }
  template<typename ... Args>
  void error(std::format_string<Args...> fmt, Args && ... args)
  {
    emit(LogLevel::kError, fmt, std::forward<Args>(args)...);
  }

private:
  template<typename ... Args>
  void emit(LogLevel level, std::format_string<Args...> fmt, Args && ... args)
  {
    if (enabled(level)) {
      log(level, std::format(fmt, std::forward<Args>(args)...));
    }
  }
};

using LoggerPtr = std::shared_ptr<Logger>;

/// Writes `[level] message` lines to stderr.
class StderrLogger : public Logger
{
public:
  explicit StderrLogger(LogLevel min_level = LogLevel::kInfo)
  : min_level_(min_level) {}

  void log(LogLevel level, std::string_view message) override;
  bool enabled(LogLevel level) const override {return level >= min_level_;}

private:
  LogLevel min_level_;
  std::mutex mutex_;
};

/// Discards everything.
class NullLogger : public Logger
{
public:
  void log(LogLevel, std::string_view) override {}
  bool enabled(LogLevel) const override {return false;}
};

/// Prefixes every message with `[scope] ` and forwards to another logger.
class ScopedLogger : public Logger
{
public:
  ScopedLogger(LoggerPtr inner, std::string scope)
  : inner_(std::move(inner)), prefix_("[" + std::move(scope) + "] ") {}

  void log(LogLevel level, std::string_view message) override;
  bool enabled(LogLevel level) const override {return inner_->enabled(level);}

private:
  LoggerPtr inner_;
  std::string prefix_;
};

}  // namespace simport

#endif  // SIMPORT_CORE__LOGGER_HPP_
