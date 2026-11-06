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

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "simport_core/config.hpp"

namespace simport
{
namespace
{

/// Run `fn`, expect a ConfigError, and return its message.
template<typename Fn>
std::string error_of(Fn && fn)
{
  try {
    fn();
  } catch (const ConfigError & e) {
    return e.what();
  }
  ADD_FAILURE() << "expected ConfigError";
  return {};
}

Config parse(const std::string & yaml)
{
  return Config::from_string(yaml, "test.yaml", "/base");
}

TEST(ParseInteger, Bases)
{
  EXPECT_EQ(parse_integer<std::int32_t>("42"), 42);
  EXPECT_EQ(parse_integer<std::int32_t>("-42"), -42);
  EXPECT_EQ(parse_integer<std::int32_t>("+7"), 7);
  EXPECT_EQ(parse_integer<std::uint16_t>("0xAA55"), 0xAA55);
  EXPECT_EQ(parse_integer<std::uint16_t>("0XaA55"), 0xAA55);
  EXPECT_EQ(parse_integer<std::uint8_t>("0b1010"), 10);
  EXPECT_EQ(parse_integer<std::uint8_t>("0o17"), 15);
  EXPECT_EQ(parse_integer<std::int8_t>("-0x80"), -128);
}

TEST(ParseInteger, Limits)
{
  EXPECT_EQ(parse_integer<std::int64_t>("-9223372036854775808"),
    std::numeric_limits<std::int64_t>::min());
  EXPECT_EQ(parse_integer<std::uint64_t>("0xFFFFFFFFFFFFFFFF"),
    std::numeric_limits<std::uint64_t>::max());
  EXPECT_THROW(parse_integer<std::uint8_t>("256"), std::out_of_range);
  EXPECT_THROW(parse_integer<std::int8_t>("-129"), std::out_of_range);
  EXPECT_THROW(parse_integer<std::uint32_t>("-1"), std::out_of_range);
  EXPECT_THROW(parse_integer<std::uint64_t>("18446744073709551616"), std::out_of_range);
  EXPECT_EQ(parse_integer<std::uint32_t>("-0"), 0u);
}

TEST(ParseInteger, BadSyntax)
{
  for (const char * bad : {"", "-", "0x", "1.5", "12abc", "abc", "--1", "0xZZ", " 1"}) {
    EXPECT_THROW(parse_integer<std::int32_t>(bad), std::invalid_argument) << bad;
  }
}

TEST(ParseDouble, Forms)
{
  EXPECT_DOUBLE_EQ(parse_double("1.5"), 1.5);
  EXPECT_DOUBLE_EQ(parse_double("+2.5e3"), 2500.0);
  EXPECT_DOUBLE_EQ(parse_double("-1e-3"), -0.001);
  EXPECT_DOUBLE_EQ(parse_double("0x10"), 16.0);
  EXPECT_TRUE(std::isinf(parse_double(".inf")));
  EXPECT_LT(parse_double("-.Inf"), 0.0);
  EXPECT_TRUE(std::isnan(parse_double(".NaN")));
  EXPECT_THROW(parse_double("fast"), std::invalid_argument);
  EXPECT_THROW(parse_double(""), std::invalid_argument);
}

TEST(Config, ScalarConversions)
{
  const Config c = parse(
    "i: -3\n"
    "h: 0xFF\n"
    "f: 2.5\n"
    "b: yes\n"
    "s: hello\n"
    "l: [1, 2, 3]\n");
  EXPECT_EQ(c.get<int>("i"), -3);
  EXPECT_EQ(c.get<std::uint8_t>("h"), 255);
  EXPECT_DOUBLE_EQ(c.get<double>("f"), 2.5);
  EXPECT_FLOAT_EQ(c.get<float>("f"), 2.5f);
  EXPECT_TRUE(c.get<bool>("b"));
  EXPECT_EQ(c.get<std::string>("s"), "hello");
  EXPECT_EQ(c.get<std::vector<int>>("l"), (std::vector<int>{1, 2, 3}));
}

TEST(Config, DefaultsForMissingOrNull)
{
  const Config c = parse("a: 1\nn: ~\n");
  EXPECT_EQ(c.get<int>("missing", 9), 9);
  EXPECT_EQ(c.get<int>("n", 4), 4);
  EXPECT_EQ(c.get<int>("a", 9), 1);
  // Navigating through missing nodes is safe.
  EXPECT_FALSE(c["x"]["y"]["z"].defined());
  EXPECT_EQ(c["x"]["y"].get<int>("z", 5), 5);
  EXPECT_EQ(c["x"]["y"]["z"].path(), "x.y.z");
}

TEST(Config, ErrorsCarryLocationAndPath)
{
  const Config c = parse(
    "send:\n"
    "  - name: a\n"
    "    from:\n"
    "      depth: deep\n");
  const Config t = c["send"].at(0)["from"];
  EXPECT_EQ(t.path(), "send[0].from");
  EXPECT_EQ(
    error_of([&] {t.get<double>("depth");}),
    "test.yaml:4:14: send[0].from.depth: expected a number, got 'deep'");
}

TEST(Config, MissingRequiredKey)
{
  const Config c = parse("send:\n  - name: a\n");
  const std::string msg = error_of([&] {c["send"].at(0).required("fields");});
  EXPECT_EQ(msg, "test.yaml:2:5: send[0].fields: required key is missing");
}

TEST(Config, IntegerRangeError)
{
  const Config c = parse("v: 300\n");
  EXPECT_EQ(
    error_of([&] {c.get<std::uint8_t>("v");}),
    "test.yaml:1:4: v: integer '300' is out of range [0, 255]");
}

TEST(Config, TypeShapeErrors)
{
  const Config c = parse("m: {a: 1}\nl: [1]\ns: x\n");
  EXPECT_NE(error_of([&] {c["m"].as<int>();}).find("expected a scalar value, got a mapping"),
    std::string::npos);
  EXPECT_NE(error_of([&] {c["s"].items();}).find("expected a list, got 'x'"), std::string::npos);
  EXPECT_NE(error_of([&] {c["l"].entries();}).find("expected a mapping, got a list"),
    std::string::npos);
  EXPECT_NE(error_of([&] {c["l"].at(3);}).find("index 3 out of range"), std::string::npos);
  EXPECT_NE(error_of([&] {c["s"].required("k");}).find("expected a mapping with key 'k'"),
    std::string::npos);
}

TEST(Config, ExpectKeysSuggestsClosest)
{
  const Config c = parse("plugin: x\nrat_hz: 5\n");
  const std::string msg = error_of([&] {c.expect_keys({"plugin", "rate_hz", "stale_timeout"});});
  EXPECT_EQ(
    msg,
    "test.yaml:2:1: rat_hz: unknown key 'rat_hz' (did you mean 'rate_hz'?); "
    "allowed keys: plugin rate_hz stale_timeout");
  EXPECT_NO_THROW(c.expect_keys({"plugin", "rat_hz"}));
  EXPECT_NO_THROW(c["missing"].expect_keys({"a"}));
}

TEST(Config, ExpectKeysNoSuggestionWhenFar)
{
  const Config c = parse("zzzzzz: 1\n");
  const std::string msg = error_of([&] {c.expect_keys({"plugin"});});
  EXPECT_EQ(msg.find("did you mean"), std::string::npos);
}

TEST(Config, EntriesKeepDocumentOrder)
{
  const Config c = parse("map:\n  z: 1\n  a: 2\n  m: 3\n");
  EXPECT_EQ(c["map"].keys(), (std::vector<std::string>{"z", "a", "m"}));
  const auto entries = c["map"].entries();
  EXPECT_EQ(entries[1].second.path(), "map.a");
  EXPECT_EQ(entries[1].second.as<int>(), 2);
}

TEST(Config, AssignmentDoesNotMutateDocument)
{
  const Config c = parse("a: 1\nb: 2\n");
  Config x = c["a"];
  x = c["b"];
  EXPECT_EQ(x.as<int>(), 2);
  EXPECT_EQ(c.get<int>("a"), 1);  // yaml-cpp's Node::operator= would have overwritten this
}

TEST(Config, SyntaxErrorHasLocation)
{
  const std::string msg = error_of([] {Config::load_file("data/broken.yaml");});
  EXPECT_NE(msg.find("data/broken.yaml:"), std::string::npos) << msg;
  EXPECT_NE(msg.find("YAML syntax error"), std::string::npos) << msg;
}

TEST(Config, MissingFile)
{
  EXPECT_EQ(error_of([] {Config::load_file("data/nope.yaml");}),
    "data/nope.yaml: cannot open file");
}

TEST(Config, ResolvePath)
{
  const Config c = parse("x: 1\n");
  EXPECT_EQ(c.resolve_path("profiles/a.yaml"), std::filesystem::path("/base/profiles/a.yaml"));
  EXPECT_EQ(c.resolve_path("../a.yaml"), std::filesystem::path("/a.yaml"));
  EXPECT_EQ(c.resolve_path("/abs/a.yaml"), std::filesystem::path("/abs/a.yaml"));
  EXPECT_NE(error_of([&] {c.resolve_path("package://no_such_pkg_xyz/a.yaml");})
    .find("package 'no_such_pkg_xyz' not found"), std::string::npos);
  EXPECT_NE(error_of([&] {c.resolve_path("package://only_pkg");})
    .find("expected package://<package>/<path>"), std::string::npos);
}

TEST(Config, InlineOrFileLoadsRelativeToIncludingFile)
{
  const Config root = Config::load_file("data/device_imu.yaml");
  const Config packet = root["packet"];
  ASSERT_TRUE(packet.is_scalar());
  const Config loaded = packet.inline_or_file();
  ASSERT_TRUE(loaded.is_map());
  EXPECT_EQ(loaded.get<std::string>("endian"), "little");
  // Paths continue from the including node; locations point into the included file.
  const Config field = loaded["fields"].at(0)["length"];
  EXPECT_EQ(field.path(), "packet.fields[0].length");
  EXPECT_NE(field.location().find("packet.yaml:4:"), std::string::npos) << field.location();

  // Inline content is returned unchanged.
  const Config inline_cfg = parse("packet: {endian: big}\n")["packet"].inline_or_file();
  EXPECT_EQ(inline_cfg.get<std::string>("endian"), "big");
}

TEST(Config, InlineOrFileMissingInclude)
{
  const Config c = parse("packet: missing.yaml\n");
  const std::string msg = error_of([&] {c["packet"].inline_or_file();});
  EXPECT_NE(msg.find("test.yaml:1:9: packet: cannot include file: /base/missing.yaml"),
    std::string::npos) << msg;
}

}  // namespace
}  // namespace simport
