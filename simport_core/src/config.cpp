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

#include "simport_core/config.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace fs = std::filesystem;

namespace simport
{

namespace
{

YAML::Node undefined_node() {return YAML::Node(YAML::NodeType::Undefined);}

std::string describe(const YAML::Node & n)
{
  switch (n.Type()) {
    case YAML::NodeType::Undefined:
      return "nothing";
    case YAML::NodeType::Null:
      return "null";
    case YAML::NodeType::Scalar:
      return "'" + n.Scalar() + "'";
    case YAML::NodeType::Sequence:
      return "a list";
    case YAML::NodeType::Map:
      return "a mapping";
  }
  return "unknown";
}

std::string format_mark(const std::string & source, const YAML::Mark & mark)
{
  if (mark.is_null()) {
    return source;
  }
  return source + ":" + std::to_string(mark.line + 1) + ":" + std::to_string(mark.column + 1);
}

std::size_t edit_distance(std::string_view a, std::string_view b)
{
  std::vector<std::size_t> row(b.size() + 1);
  for (std::size_t j = 0; j <= b.size(); ++j) {
    row[j] = j;
  }
  for (std::size_t i = 1; i <= a.size(); ++i) {
    std::size_t diag = row[0];
    row[0] = i;
    for (std::size_t j = 1; j <= b.size(); ++j) {
      const std::size_t up = row[j];
      row[j] = std::min({row[j] + 1, row[j - 1] + 1, diag + (a[i - 1] == b[j - 1] ? 0 : 1)});
      diag = up;
    }
  }
  return row[b.size()];
}

std::string lower(std::string_view s)
{
  std::string out(s);
  std::transform(
    out.begin(), out.end(), out.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return out;
}

template<typename T>
std::string integer_range()
{
  return "[" + std::to_string(std::numeric_limits<T>::min()) + ", " +
         std::to_string(std::numeric_limits<T>::max()) + "]";
}

}  // namespace

// -- parsing helpers ---------------------------------------------------------------------------

template<typename T>
T parse_integer(std::string_view text)
{
  static_assert(std::is_integral_v<T>&& !std::is_same_v<T, bool>);
  std::size_t i = 0;
  bool negative = false;
  if (i < text.size() && (text[i] == '+' || text[i] == '-')) {
    negative = text[i] == '-';
    ++i;
  }
  int base = 10;
  if (text.size() - i > 2 && text[i] == '0') {
    const char p = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i + 1])));
    if (p == 'x') {
      base = 16;
    } else if (p == 'o') {
      base = 8;
    } else if (p == 'b') {
      base = 2;
    }
    if (base != 10) {
      i += 2;
    }
  }
  const std::string_view digits = text.substr(i);
  if (digits.empty()) {
    throw std::invalid_argument("not an integer");
  }
  std::uint64_t magnitude = 0;
  const auto [end, ec] = std::from_chars(digits.data(), digits.data() + digits.size(), magnitude,
      base);
  if (ec == std::errc::result_out_of_range) {
    throw std::out_of_range("integer out of range");
  }
  if (ec != std::errc{} || end != digits.data() + digits.size()) {
    throw std::invalid_argument("not an integer");
  }

  using Lim = std::numeric_limits<T>;
  if (!negative) {
    if (magnitude > static_cast<std::uint64_t>(Lim::max())) {
      throw std::out_of_range("integer out of range");
    }
    return static_cast<T>(magnitude);
  }
  if constexpr (std::is_unsigned_v<T>) {
    if (magnitude != 0) {
      throw std::out_of_range("integer out of range");
    }
    return 0;
  } else {
    // |min| == max + 1 for two's complement.
    const auto limit = static_cast<std::uint64_t>(Lim::max()) + 1;
    if (magnitude > limit) {
      throw std::out_of_range("integer out of range");
    }
    if (magnitude == limit) {
      return Lim::min();
    }
    return static_cast<T>(-static_cast<std::int64_t>(magnitude));
  }
}

template std::int8_t parse_integer<std::int8_t>(std::string_view);
template std::int16_t parse_integer<std::int16_t>(std::string_view);
template std::int32_t parse_integer<std::int32_t>(std::string_view);
template std::int64_t parse_integer<std::int64_t>(std::string_view);
template std::uint8_t parse_integer<std::uint8_t>(std::string_view);
template std::uint16_t parse_integer<std::uint16_t>(std::string_view);
template std::uint32_t parse_integer<std::uint32_t>(std::string_view);
template std::uint64_t parse_integer<std::uint64_t>(std::string_view);

double parse_double(std::string_view text)
{
  const std::string l = lower(text);
  if (l == ".inf" || l == "+.inf") {
    return std::numeric_limits<double>::infinity();
  }
  if (l == "-.inf") {
    return -std::numeric_limits<double>::infinity();
  }
  if (l == ".nan") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Integer spellings (including hex/octal/binary).
  try {
    return static_cast<double>(parse_integer<std::int64_t>(text));
  } catch (const std::exception &) {
  }
  try {
    return static_cast<double>(parse_integer<std::uint64_t>(text));
  } catch (const std::exception &) {
  }
  std::string_view t = text;
  if (!t.empty() && t[0] == '+') {
    t.remove_prefix(1);
  }
  double value = 0.0;
  const auto [end, ec] = std::from_chars(t.data(), t.data() + t.size(), value);
  if (ec != std::errc{} || end != t.data() + t.size() || t.empty()) {
    throw std::invalid_argument("not a number");
  }
  return value;
}

// -- Config ------------------------------------------------------------------------------------

Config::Config()
: node_(undefined_node()), mark_(YAML::Mark::null_mark()) {}

Config::Config(
  YAML::Node node, std::shared_ptr<const Source> source, std::string path,
  YAML::Mark mark)
: node_(std::move(node)), source_(std::move(source)), path_(std::move(path)), mark_(mark) {}

Config & Config::operator=(const Config & other)
{
  if (this != &other) {
    node_.reset(other.node_);
    source_ = other.source_;
    path_ = other.path_;
    mark_ = other.mark_;
  }
  return *this;
}

Config & Config::operator=(Config && other)
{
  if (this != &other) {
    node_.reset(other.node_);
    source_ = std::move(other.source_);
    path_ = std::move(other.path_);
    mark_ = other.mark_;
  }
  return *this;
}

Config Config::load_file(const fs::path & file)
{
  const std::string name = file.string();
  YAML::Node root = [&]() {
      try {
        return YAML::LoadFile(name);
      } catch (const YAML::BadFile &) {
        throw ConfigError(name + ": cannot open file");
      } catch (const YAML::ParserException & e) {
        throw ConfigError(format_mark(name, e.mark) + ": YAML syntax error: " + e.msg);
      }
    }();
  auto source = std::make_shared<const Source>(Source{name, fs::absolute(file).parent_path()});
  const YAML::Mark mark = root.IsDefined() ? root.Mark() : YAML::Mark::null_mark();
  return Config(root, std::move(source), "", mark);
}

Config Config::from_string(const std::string & yaml, std::string source_name, fs::path base_dir)
{
  YAML::Node root = [&]() {
      try {
        return YAML::Load(yaml);
      } catch (const YAML::ParserException & e) {
        throw ConfigError(format_mark(source_name, e.mark) + ": YAML syntax error: " + e.msg);
      }
    }();
  auto source = std::make_shared<const Source>(Source{std::move(source_name), std::move(base_dir)});
  return Config(root, std::move(source), "", root.Mark());
}

bool Config::defined() const {return node_.IsDefined();}
bool Config::is_null() const {return node_.IsDefined() && node_.IsNull();}
bool Config::is_scalar() const {return node_.IsScalar();}
bool Config::is_sequence() const {return node_.IsSequence();}
bool Config::is_map() const {return node_.IsMap();}

std::string Config::location() const
{
  return format_mark(source_ ? source_->name : std::string("<unknown>"), mark_);
}

Config Config::child(const YAML::Node & node, std::string path_suffix, bool is_index) const
{
  std::string path;
  if (is_index) {
    path = path_ + "[" + path_suffix + "]";
  } else {
    path = path_.empty() ? std::move(path_suffix) : path_ + "." + path_suffix;
  }
  if (!node.IsDefined()) {
    return Config(undefined_node(), source_, std::move(path), mark_);
  }
  return Config(node, source_, std::move(path), node.Mark());
}

bool Config::has(std::string_view key) const
{
  if (!is_map()) {
    return false;
  }
  const YAML::Node & n = node_;
  return n[std::string(key)].IsDefined();
}

Config Config::operator[](std::string_view key) const
{
  if (!is_map()) {
    return child(undefined_node(), std::string(key), false);
  }
  const YAML::Node & n = node_;
  return child(n[std::string(key)], std::string(key), false);
}

Config Config::required(std::string_view key) const
{
  if (defined() && !is_null() && !is_map()) {
    error("expected a mapping with key '" + std::string(key) + "', got " + describe(node_));
  }
  Config c = (*this)[key];
  if (!c.defined()) {
    c.error("required key is missing");
  }
  return c;
}

std::size_t Config::size() const
{
  return (is_sequence() || is_map()) ? node_.size() : 0;
}

Config Config::at(std::size_t index) const
{
  if (!is_sequence()) {
    error("expected a list, got " + describe(node_));
  }
  if (index >= node_.size()) {
    error("index " + std::to_string(index) + " out of range (size " +
      std::to_string(node_.size()) + ")");
  }
  const YAML::Node & n = node_;
  return child(n[index], std::to_string(index), true);
}

std::vector<Config> Config::items() const
{
  if (!defined() || is_null()) {
    return {};
  }
  if (!is_sequence()) {
    error("expected a list, got " + describe(node_));
  }
  std::vector<Config> out;
  out.reserve(node_.size());
  const YAML::Node & n = node_;
  for (std::size_t i = 0; i < n.size(); ++i) {
    out.push_back(child(n[i], std::to_string(i), true));
  }
  return out;
}

std::vector<std::pair<std::string, Config>> Config::entries() const
{
  if (!defined() || is_null()) {
    return {};
  }
  if (!is_map()) {
    error("expected a mapping, got " + describe(node_));
  }
  std::vector<std::pair<std::string, Config>> out;
  const YAML::Node & n = node_;
  for (auto it = n.begin(); it != n.end(); ++it) {
    if (!it->first.IsScalar()) {
      Config(it->first, source_, path_, it->first.Mark()).error("mapping keys must be scalars");
    }
    const std::string & key = it->first.Scalar();
    out.emplace_back(key, child(it->second, key, false));
  }
  return out;
}

std::vector<std::string> Config::keys() const
{
  std::vector<std::string> out;
  for (auto & [key, value] : entries()) {
    out.push_back(key);
  }
  return out;
}

const std::string & Config::scalar() const
{
  if (!is_scalar()) {
    error("expected a scalar value, got " + describe(node_));
  }
  return node_.Scalar();
}

void Config::expect_keys(std::initializer_list<std::string_view> allowed) const
{
  expect_keys_impl(std::vector<std::string_view>(allowed));
}

void Config::expect_keys(const std::vector<std::string> & allowed) const
{
  expect_keys_impl(std::vector<std::string_view>(allowed.begin(), allowed.end()));
}

void Config::expect_keys_impl(const std::vector<std::string_view> & allowed) const
{
  if (!defined() || is_null()) {
    return;
  }
  if (!is_map()) {
    error("expected a mapping, got " + describe(node_));
  }
  const YAML::Node & n = node_;
  for (auto it = n.begin(); it != n.end(); ++it) {
    const std::string key = it->first.IsScalar() ? it->first.Scalar() : std::string();
    if (std::find(allowed.begin(), allowed.end(), key) != allowed.end()) {
      continue;
    }
    std::string message = "unknown key '" + key + "'";
    std::string_view best;
    std::size_t best_distance = std::numeric_limits<std::size_t>::max();
    for (const auto candidate : allowed) {
      const std::size_t d = edit_distance(lower(key), lower(candidate));
      if (d < best_distance) {
        best_distance = d;
        best = candidate;
      }
    }
    if (!best.empty() && best_distance <= std::max<std::size_t>(2, key.size() / 3)) {
      message += " (did you mean '" + std::string(best) + "'?)";
    }
    message += "; allowed keys:";
    for (const auto candidate : allowed) {
      message += " " + std::string(candidate);
    }
    child(it->first, key, false).error(message);
  }
}

void Config::error(std::string_view message) const
{
  std::string text = location() + ": ";
  if (!path_.empty()) {
    text += path_ + ": ";
  }
  text += message;
  throw ConfigError(text);
}

fs::path Config::resolve_path(std::string_view path) const
{
  const std::string text(path);
  constexpr std::string_view kPackagePrefix = "package://";
  if (text.starts_with(kPackagePrefix)) {
    const std::string rest = text.substr(kPackagePrefix.size());
    const auto slash = rest.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 == rest.size()) {
      error("invalid package path '" + text + "'; expected package://<package>/<path>");
    }
    const std::string package = rest.substr(0, slash);
    try {
      return fs::path(ament_index_cpp::get_package_share_directory(package)) /
             rest.substr(slash + 1);
    } catch (const ament_index_cpp::PackageNotFoundError &) {
      error("package '" + package + "' not found (is the workspace sourced?)");
    }
  }
  if (text == "~" || text.starts_with("~/")) {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      error("cannot expand '~': HOME is not set");
    }
    return fs::path(home) / text.substr(std::min<std::size_t>(2, text.size()));
  }
  const fs::path p(text);
  if (p.is_absolute()) {
    return p;
  }
  const fs::path base = source_ ? source_->base_dir : fs::current_path();
  return (base / p).lexically_normal();
}

Config Config::inline_or_file() const
{
  if (!is_scalar()) {
    return *this;
  }
  const fs::path file = resolve_path(scalar());
  Config loaded;
  try {
    loaded = load_file(file);
  } catch (const ConfigError & e) {
    error(std::string("cannot include file: ") + e.what());
  }
  loaded.path_ = path_;
  return loaded;
}

// -- scalar conversions ------------------------------------------------------------------------

namespace detail
{

template<>
std::string convert_scalar<std::string>(const Config & cfg)
{
  return cfg.scalar();
}

template<>
bool convert_scalar<bool>(const Config & cfg)
{
  const std::string l = lower(cfg.scalar());
  if (l == "true" || l == "yes" || l == "on") {
    return true;
  }
  if (l == "false" || l == "no" || l == "off") {
    return false;
  }
  cfg.error("expected a boolean (true/false), got '" + cfg.scalar() + "'");
}

template<>
double convert_scalar<double>(const Config & cfg)
{
  try {
    return parse_double(cfg.scalar());
  } catch (const std::invalid_argument &) {
    cfg.error("expected a number, got '" + cfg.scalar() + "'");
  }
}

template<>
float convert_scalar<float>(const Config & cfg)
{
  const double d = convert_scalar<double>(cfg);
  if (std::isfinite(d) && std::abs(d) > static_cast<double>(std::numeric_limits<float>::max())) {
    cfg.error("number '" + cfg.scalar() + "' is out of range for a 32-bit float");
  }
  return static_cast<float>(d);
}

#define SIMPORT_CONVERT_INTEGER(T) \
  template<> \
  T convert_scalar<T>(const Config & cfg) \
  { \
    try { \
      return parse_integer<T>(cfg.scalar()); \
    } catch (const std::out_of_range &) { \
      cfg.error("integer '" + cfg.scalar() + "' is out of range " + integer_range<T>()); \
    } catch (const std::invalid_argument &) { \
      cfg.error("expected an integer, got '" + cfg.scalar() + "'"); \
    } \
  }

SIMPORT_CONVERT_INTEGER(std::int8_t)
SIMPORT_CONVERT_INTEGER(std::int16_t)
SIMPORT_CONVERT_INTEGER(std::int32_t)
SIMPORT_CONVERT_INTEGER(std::int64_t)
SIMPORT_CONVERT_INTEGER(std::uint8_t)
SIMPORT_CONVERT_INTEGER(std::uint16_t)
SIMPORT_CONVERT_INTEGER(std::uint32_t)
SIMPORT_CONVERT_INTEGER(std::uint64_t)

#undef SIMPORT_CONVERT_INTEGER

}  // namespace detail

}  // namespace simport
