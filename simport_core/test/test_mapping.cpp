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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "simport_core/mapping.hpp"

using namespace std::chrono_literals;

namespace simport
{
namespace
{

SchemaPtr imu_schema()
{
  return std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"sync", ValueKind::kUint, FieldRole::kComputed},
        {"gx", ValueKind::kInt, FieldRole::kInput},
        {"ax", ValueKind::kFloat, FieldRole::kInput},
        {"t_us", ValueKind::kUint, FieldRole::kInput},
        {"id", ValueKind::kUint, FieldRole::kInput},
        {"label", ValueKind::kString, FieldRole::kInput},
        {"spare", ValueKind::kInt, FieldRole::kInput},
        {"crc", ValueKind::kUint, FieldRole::kComputed},
    });
}

/// Field descriptions as a codec reports them: `{ field: description, ... }`.
MappingSpec parse(const std::string & yaml, SchemaPtr schema = imu_schema())
{
  return MappingSpec::from_sources(Config::from_string(yaml, "m.yaml").entries(), schema);
}

TEST(FieldPath, Parses)
{
  EXPECT_EQ(parse_field_path("x"), (std::vector<PathElement>{{"x", std::nullopt}}));
  EXPECT_EQ(
    parse_field_path("pose.position.x"),
    (std::vector<PathElement>{{"pose", {}}, {"position", {}}, {"x", {}}}));
  EXPECT_EQ(
    parse_field_path("poses[12].pose"),
    (std::vector<PathElement>{{"poses", 12}, {"pose", {}}}));
  EXPECT_EQ(parse_field_path("_a1[0]"), (std::vector<PathElement>{{"_a1", 0}}));
}

TEST(FieldPath, RejectsBadSyntax)
{
  for (const char * bad : {"", ".x", "x.", "x..y", "1x", "x[", "x[]", "x[1", "x[a]", "x y",
      "x[1]y"})
  {
    EXPECT_THROW(parse_field_path(bad), std::invalid_argument) << bad;
  }
}

TEST(Builtin, NamesRoundTrip)
{
  for (const auto name : builtin_names()) {
    const auto b = parse_builtin(name);
    ASSERT_TRUE(b.has_value()) << name;
    EXPECT_EQ(to_string(*b), name);
  }
  EXPECT_FALSE(parse_builtin("$time_fortnights").has_value());
}

TEST(Builtin, Evaluate)
{
  const BuiltinInputs in{1'500'000'000ns, 7s, 2'000'000ns};
  EXPECT_EQ(std::get<std::int64_t>(evaluate(Builtin::kTimeNs, in)), 1'500'000'000);
  EXPECT_EQ(std::get<std::int64_t>(evaluate(Builtin::kTimeUs, in)), 1'500'000);
  EXPECT_EQ(std::get<std::int64_t>(evaluate(Builtin::kTimeMs, in)), 1'500);
  EXPECT_DOUBLE_EQ(std::get<double>(evaluate(Builtin::kTimeS, in)), 1.5);
  EXPECT_EQ(std::get<std::int64_t>(evaluate(Builtin::kStampMs, in)), 2);
  EXPECT_EQ(std::get<std::int64_t>(evaluate(Builtin::kWallTimeMs, in)), 7000);
}

TEST(ValueTransform, IdentityPreservesIntegers)
{
  const ValueTransform id;
  const std::uint64_t big = (std::uint64_t{1} << 60) + 1;
  EXPECT_EQ(std::get<std::uint64_t>(id.apply(Value{big})), big);
  ValueTransform t;
  t.scale = 1000.0;
  t.offset = 1.0;
  EXPECT_DOUBLE_EQ(std::get<double>(t.apply(Value{0.5})), 501.0);
  EXPECT_THROW(t.apply(Value{std::string("x")}), ValueError);
}

TEST(ValueTransform, NonFiniteReplacement)
{
  ValueTransform t;
  t.scale = 1000.0;
  t.nonfinite = 0.0;
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(std::get<double>(t.apply(Value{inf})), 0.0);
  EXPECT_DOUBLE_EQ(std::get<double>(t.apply(Value{std::nan("")})), 0.0);
  EXPECT_DOUBLE_EQ(std::get<double>(t.apply(Value{1.5})), 1500.0);
  ValueTransform only;
  only.nonfinite = -1.0;
  EXPECT_EQ(std::get<std::int64_t>(only.apply(Value{std::int64_t{7}})), 7);
}

TEST(ValueTransform, ArraySelection)
{
  const Value in = std::vector<double>{0, 1, 2, 3, 4, 5, 6};
  const auto run = [&](auto setup) {
      ValueTransform t;
      setup(t);
      return std::get<std::vector<double>>(t.apply(in));
    };
  EXPECT_EQ(run([](ValueTransform & t) {t.reverse = true;}),
    (std::vector<double>{6, 5, 4, 3, 2, 1, 0}));
  EXPECT_EQ(run([](ValueTransform & t) {t.step = 3;}), (std::vector<double>{0, 3, 6}));
  EXPECT_EQ(run([](ValueTransform & t) {t.reverse = true; t.step = 2;}),
    (std::vector<double>{6, 4, 2, 0}));
  EXPECT_EQ(run([](ValueTransform & t) {t.resize = 4;}), (std::vector<double>{0, 2, 4, 6}));
  EXPECT_EQ(run([](ValueTransform & t) {t.resize = 1;}), (std::vector<double>{0}));
  EXPECT_EQ(run([](ValueTransform & t) {t.resize = 10;}).size(), 10u);
  EXPECT_EQ(run([](ValueTransform & t) {t.scale = 2.0; t.offset = 1.0;}),
    (std::vector<double>{1, 3, 5, 7, 9, 11, 13}));

  ValueTransform rev;
  rev.reverse = true;
  EXPECT_THROW(rev.apply(Value{1.0}), ValueError);
}

TEST(ValueTransform, TransposeAndRamp)
{
  // 2 rows x 3 columns, row-major -> column-major.
  ValueTransform t;
  t.transpose_rows = 2;
  EXPECT_EQ(std::get<std::vector<double>>(t.apply(Value{std::vector<double>{1, 2, 3, 4, 5, 6}})),
    (std::vector<double>{1, 4, 2, 5, 3, 6}));
  t.reverse = true;  // after the transpose
  EXPECT_EQ(std::get<std::vector<double>>(t.apply(Value{std::vector<double>{1, 2, 3, 4, 5, 6}})),
    (std::vector<double>{6, 3, 5, 2, 4, 1}));
  EXPECT_THROW(t.apply(Value{std::vector<double>{1, 2, 3}}), ValueError);

  ValueTransform r;
  r.scale = 2.0;
  r.ramp = ValueTransform::Ramp{0.5, 4};
  EXPECT_EQ(std::get<std::vector<double>>(r.apply(Value{std::int64_t{10}})),
    (std::vector<double>{20, 20.5, 21, 21.5}));
  EXPECT_THROW(r.apply(Value{std::vector<double>{1}}), ValueError);
}

TEST(MappingSpec, FromFieldDescriptions)
{
  const MappingSpec m = parse(
    "t_us: { u32, from: $time_us }\n"
    "gx: { i16, from: angular_velocity.x, scale: 1000.0 }\n"
    "ax: linear_acceleration.x\n"          // text codecs: `name: path`
    "id: { u8, value: 7 }\n"               // no `from`: not mapped
    "crc: { from: x }\n");                 // computed: never mapped

  ASSERT_EQ(m.paths().size(), 2u);
  // Sorted by schema order, not document order.
  EXPECT_EQ(m.paths()[0].target_name, "gx");
  EXPECT_EQ(m.paths()[0].path, "angular_velocity.x");
  EXPECT_EQ(m.paths()[0].transform.scale, 1000.0);
  EXPECT_EQ(m.paths()[1].target_name, "ax");
  EXPECT_TRUE(m.paths()[1].transform.identity());
  EXPECT_EQ(m.paths()[1].path_elements.size(), 2u);

  ASSERT_EQ(m.builtins().size(), 1u);
  EXPECT_EQ(m.builtins()[0].builtin, Builtin::kTimeUs);
  EXPECT_EQ(m.unmapped(), (std::vector<std::size_t>{4, 5, 6}));  // id, label, spare
}

TEST(MappingSpec, MakeRecordAndBuiltins)
{
  const MappingSpec m = parse("t_us: { from: $stamp_us, offset: 5 }\n");
  Record r = m.make_record();
  m.apply_builtins(r, BuiltinInputs{0ns, 0ns, 3'000ns});
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("t_us")), 8.0);
  EXPECT_EQ(MappingSpec::from_sources({}, imu_schema()).unmapped().size(), 6u);
}

TEST(MappingSpec, Errors)
{
  const auto has = [](const std::string & yaml, const std::string & what) {
      try {
        parse(yaml);
        ADD_FAILURE() << "expected ConfigError for: " << yaml;
      } catch (const ConfigError & e) {
        EXPECT_NE(std::string(e.what()).find(what), std::string::npos) << e.what();
      }
    };
  has("gx: a..b\n", "invalid field path");
  has("gx: $time_weeks\n", "unknown builtin '$time_weeks'");
  has("label: $time_us\n", "builtin values are numeric");
  has("label: { from: a, scale: 2 }\n", "need a numeric field");
  // Location points at the offending entry.
  has("ax: x\ngx: { from: a..b }\n", "m.yaml:2:");
}

TEST(MappingSpec, ArrayFields)
{
  const auto schema = std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"dist", ValueKind::kFloatArray, FieldRole::kInput},
        {"speed", ValueKind::kUint, FieldRole::kInput},
    });
  const MappingSpec m = parse(
    "dist: { from: ranges, reverse: true, step: 2, resize: 360, scale: 4000, nonfinite: 0 }\n",
    schema);
  ASSERT_EQ(m.paths().size(), 1u);
  const ValueTransform & t = m.paths()[0].transform;
  EXPECT_TRUE(t.reverse);
  EXPECT_EQ(t.step, 2u);
  EXPECT_EQ(t.resize, 360u);
  EXPECT_EQ(t.nonfinite, 0.0);

  const auto error = [&](const std::string & yaml) -> std::string {
      try {
        parse(yaml, schema);
      } catch (const ConfigError & e) {
        return e.what();
      }
      return {};
    };
  EXPECT_NE(error("speed: { from: a, reverse: true }\n").find("rearrange array elements"),
    std::string::npos);
  EXPECT_NE(error("dist: { from: a, step: 0 }\n").find("at least 1"), std::string::npos);
  EXPECT_NE(error("dist: { from: $time_us, reverse: true }\n").find("need an array source"),
    std::string::npos);
  EXPECT_NE(error("speed: { from: $time_ns, ramp: { step: 1, count: 2 } }\n")
    .find("makes an array"), std::string::npos);
  EXPECT_NE(error("dist: { from: a, transpose: { rows: 0 } }\n").find("at least 1"),
    std::string::npos);
  const MappingSpec ramped = parse("dist: { from: $stamp_ns, ramp: { step: 10, count: 3 } }\n",
      schema);
  ASSERT_EQ(ramped.builtins().size(), 1u);
  Record rec = ramped.make_record();
  ramped.apply_builtins(rec, BuiltinInputs{0ns, 0ns, 100ns});
  EXPECT_EQ(std::get<std::vector<double>>(rec.at("dist")), (std::vector<double>{100, 110, 120}));
}

}  // namespace
}  // namespace simport
