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

#ifndef SIMPORT_CORE__CONFIG_HPP_
#define SIMPORT_CORE__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace simport
{

/// Error in user configuration. The message carries file, line, column and config path.
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Read-only view of a YAML configuration node that remembers where it came from.
///
/// Every accessor that fails throws ConfigError with a message of the form
///   `<file>:<line>:<col>: <path>: <what went wrong>`
/// e.g. `imu0.yaml:12:16: send[0].rate: expected a number, got 'fast'`.
///
/// Scalars are converted by simport rather than yaml-cpp so that behaviour is predictable:
/// integers accept decimal, `0x` hex, `0o` octal and `0b` binary with an optional sign, and are
/// range-checked against the target type; floats additionally accept `.inf`, `-.inf`, `.nan`.
class Config
{
public:
  /// An undefined (missing) node.
  Config();

  Config(const Config &) = default;
  Config(Config &&) = default;
  // yaml-cpp's Node::operator= assigns *through* the handle, mutating the shared document.
  // These rebind the handle instead.
  Config & operator=(const Config & other);
  Config & operator=(Config && other);
  ~Config() = default;

  /// Parse a YAML file. Relative paths inside it resolve against the file's directory.
  static Config load_file(const std::filesystem::path & file);

  /// Parse YAML text. `source_name` is used in error messages; relative paths resolve against
  /// `base_dir`.
  static Config from_string(
    const std::string & yaml, std::string source_name = "<string>",
    std::filesystem::path base_dir = std::filesystem::current_path());

  // -- shape ---------------------------------------------------------------------------------

  /// False if this node is missing from the document.
  bool defined() const;
  bool is_null() const;
  bool is_scalar() const;
  bool is_sequence() const;
  bool is_map() const;

  /// Dotted path from the document root, e.g. `send[0].fields[2]`. Empty for the root.
  const std::string & path() const {return path_;}

  /// `file:line:col` of this node (or of the closest defined ancestor if missing).
  std::string location() const;

  // -- navigation ----------------------------------------------------------------------------

  /// True if this is a map containing `key`.
  bool has(std::string_view key) const;

  /// Child by key. Never throws; the result may be undefined. Use required() to insist.
  Config operator[](std::string_view key) const;

  /// Child by key; throws if this is not a map or the key is missing.
  Config required(std::string_view key) const;

  /// Number of elements of a sequence or map (0 for scalars).
  std::size_t size() const;

  /// Sequence element; throws if this is not a sequence or out of range.
  Config at(std::size_t index) const;

  /// Elements of a sequence; throws if this is not a sequence. An undefined node yields {}.
  std::vector<Config> items() const;

  /// Entries of a map in document order; throws if this is not a map. Undefined yields {}.
  std::vector<std::pair<std::string, Config>> entries() const;

  /// Keys of a map in document order.
  std::vector<std::string> keys() const;

  // -- values --------------------------------------------------------------------------------

  /// Convert this node. Supported: bool, std::string, all fixed-width integer types, float,
  /// double, and std::vector of those.
  template<typename T>
  T as() const;

  /// Required child value.
  template<typename T>
  T get(std::string_view key) const
  {
    return required(key).as<T>();
  }

  /// Optional child value; `fallback` if the key is missing or null.
  template<typename T>
  T get(std::string_view key, T fallback) const
  {
    const Config child = (*this)[key];
    if (!child.defined() || child.is_null()) {
      return fallback;
    }
    return child.as<T>();
  }

  /// The raw scalar text (throws if not a scalar).
  const std::string & scalar() const;

  // -- validation and helpers -----------------------------------------------------------------

  /// Throw if this map contains a key not in `allowed`, suggesting the closest allowed key.
  /// An undefined node passes.
  void expect_keys(std::initializer_list<std::string_view> allowed) const;
  void expect_keys(const std::vector<std::string> & allowed) const;

  /// Throw a ConfigError located at this node.
  [[noreturn]] void error(std::string_view message) const;

  /// Resolve a path relative to the directory of the file this node was loaded from.
  /// Supports absolute paths, `~/` and `package://<pkg>/<relative path>`.
  std::filesystem::path resolve_path(std::string_view path) const;

  /// Content that may be given inline or as a path to a YAML file. If this node is a string
  /// scalar, the file is loaded (relative to this file) and returned, keeping this node's
  /// config path; otherwise this node is returned as-is.
  Config inline_or_file() const;

  /// Escape hatch for plugins that need yaml-cpp directly.
  const YAML::Node & node() const {return node_;}

private:
  struct Source
  {
    std::string name;
    std::filesystem::path base_dir;
  };

  Config(
    YAML::Node node, std::shared_ptr<const Source> source, std::string path,
    YAML::Mark mark);

  Config child(const YAML::Node & node, std::string path_suffix, bool is_index) const;
  void expect_keys_impl(const std::vector<std::string_view> & allowed) const;

  YAML::Node node_;
  std::shared_ptr<const Source> source_;
  std::string path_;
  YAML::Mark mark_;
};

/// Parse an integer literal: decimal, `0x`, `0o`, `0b`, optional sign. Throws
/// std::invalid_argument (bad syntax) or std::out_of_range.
template<typename T>
T parse_integer(std::string_view text);

/// Parse a floating-point literal, including YAML's `.inf`, `-.inf` and `.nan` spellings and
/// any integer literal accepted by parse_integer. Throws std::invalid_argument.
double parse_double(std::string_view text);

// -- template definitions ----------------------------------------------------------------------

namespace detail
{
template<typename T>
struct is_vector : std::false_type {};
template<typename T, typename A>
struct is_vector<std::vector<T, A>>: std::true_type {};

template<typename T>
T convert_scalar(const Config & cfg);

template<>
std::string convert_scalar<std::string>(const Config & cfg);
template<>
bool convert_scalar<bool>(const Config & cfg);
template<>
float convert_scalar<float>(const Config & cfg);
template<>
double convert_scalar<double>(const Config & cfg);
template<>
std::int8_t convert_scalar<std::int8_t>(const Config & cfg);
template<>
std::int16_t convert_scalar<std::int16_t>(const Config & cfg);
template<>
std::int32_t convert_scalar<std::int32_t>(const Config & cfg);
template<>
std::int64_t convert_scalar<std::int64_t>(const Config & cfg);
template<>
std::uint8_t convert_scalar<std::uint8_t>(const Config & cfg);
template<>
std::uint16_t convert_scalar<std::uint16_t>(const Config & cfg);
template<>
std::uint32_t convert_scalar<std::uint32_t>(const Config & cfg);
template<>
std::uint64_t convert_scalar<std::uint64_t>(const Config & cfg);
}  // namespace detail

template<typename T>
T Config::as() const
{
  if constexpr (detail::is_vector<T>::value) {
    T out;
    for (const Config & item : items()) {
      out.push_back(item.template as<typename T::value_type>());
    }
    return out;
  } else {
    return detail::convert_scalar<T>(*this);
  }
}

}  // namespace simport

#endif  // SIMPORT_CORE__CONFIG_HPP_
