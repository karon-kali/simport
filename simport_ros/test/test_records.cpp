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

#include <memory>
#include <string>
#include <vector>

#include "simport_ros/records.hpp"

namespace simport
{
namespace
{

using simport_msgs::msg::RecordField;

SchemaPtr schema()
{
  return std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"id", ValueKind::kUint}, {"temp", ValueKind::kInt}, {"ok", ValueKind::kBool},
        {"gain", ValueKind::kFloat}, {"label", ValueKind::kString}, {"data", ValueKind::kBytes},
        {"ranges", ValueKind::kFloatArray}});
}

TEST(Records, ToMessage)
{
  Record r = Record::make(schema());
  r.at("id") = std::uint64_t{11};
  r.at("temp") = std::int64_t{-4};
  r.at("ok") = true;
  r.at("gain") = 0.5;
  r.at("label") = std::string("hi");
  r.at("data") = Bytes{1, 2};
  r.at("ranges") = std::vector<double>{1.5, 2.5};
  r.stamp = std::chrono::nanoseconds(3'000'000'007);
  const auto msg = to_msg(r, "ping");
  EXPECT_EQ(msg.name, "ping");
  EXPECT_EQ(msg.header.stamp.sec, 3);
  EXPECT_EQ(msg.header.stamp.nanosec, 7u);
  ASSERT_EQ(msg.fields.size(), 7u);
  EXPECT_EQ(msg.fields[0].name, "id");
  EXPECT_EQ(msg.fields[0].kind, RecordField::KIND_UINT);
  EXPECT_EQ(msg.fields[0].number, 11.0);
  EXPECT_EQ(msg.fields[1].number, -4.0);
  EXPECT_EQ(msg.fields[2].number, 1.0);
  EXPECT_EQ(msg.fields[4].kind, RecordField::KIND_STRING);
  EXPECT_EQ(msg.fields[4].text, "hi");
  EXPECT_EQ(msg.fields[5].bytes, (std::vector<std::uint8_t>{1, 2}));
  EXPECT_EQ(msg.fields[6].array, (std::vector<double>{1.5, 2.5}));
}

TEST(Records, FromMessageConvertsToTheFieldKindAndReportsUnknownNames)
{
  simport_msgs::msg::Record msg;
  const auto add = [&](const std::string & name, double number) {
      RecordField f;
      f.name = name;
      f.number = number;
      msg.fields.push_back(f);
      return &msg.fields.back();
    };
  add("id", 300.7);
  add("temp", -2.0);
  add("ok", 1.0);
  add("ranges", 4.0);  // a single number for an array field
  add("nope", 1.0);
  RecordField data;
  data.name = "data";
  data.bytes = {9, 8};
  msg.fields.push_back(data);
  RecordField ranges;  // a list of numbers for a bytes field
  ranges.name = "data";
  ranges.kind = RecordField::KIND_FLOAT_ARRAY;
  ranges.array = {1, 300};
  msg.fields.push_back(ranges);
  RecordField label;
  label.name = "label";
  label.text = "x";
  msg.fields.push_back(label);

  std::vector<std::string> unknown;
  const FieldOverrides o = from_msg(msg, *schema(), &unknown);
  EXPECT_EQ(unknown, (std::vector<std::string>{"nope"}));
  ASSERT_EQ(o.size(), 7u);
  EXPECT_EQ(o[0].first, 0u);
  EXPECT_EQ(std::get<std::uint64_t>(o[0].second), 301u);
  EXPECT_EQ(std::get<std::int64_t>(o[1].second), -2);
  EXPECT_TRUE(std::get<bool>(o[2].second));
  EXPECT_EQ(std::get<double>(o[3].second), 4.0);
  EXPECT_EQ(std::get<Bytes>(o[4].second), (Bytes{9, 8}));
  EXPECT_EQ(std::get<Bytes>(o[5].second), (Bytes{1, 255}));
  EXPECT_EQ(std::get<std::string>(o[6].second), "x");
}

}  // namespace
}  // namespace simport
