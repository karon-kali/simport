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
#include <stdexcept>
#include <string>
#include <vector>

#include "simport_core/schema.hpp"

namespace simport
{
namespace
{

SchemaPtr make_schema()
{
  return std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"sync", ValueKind::kUint, FieldRole::kComputed},
        {"gx", ValueKind::kInt, FieldRole::kInput},
        {"name", ValueKind::kString, FieldRole::kInput},
    });
}

TEST(Schema, LookupByName)
{
  const auto s = make_schema();
  EXPECT_EQ(s->size(), 3u);
  EXPECT_EQ(s->find("gx"), 1u);
  EXPECT_FALSE(s->find("gy").has_value());
  EXPECT_EQ(s->index_of("name"), 2u);
  EXPECT_THROW(s->index_of("nope"), std::out_of_range);
}

TEST(Schema, NamesByRole)
{
  const auto s = make_schema();
  EXPECT_EQ(s->names(), (std::vector<std::string>{"sync", "gx", "name"}));
  EXPECT_EQ(s->names(FieldRole::kInput), (std::vector<std::string>{"gx", "name"}));
  EXPECT_EQ(s->names(FieldRole::kComputed), (std::vector<std::string>{"sync"}));
}

TEST(Schema, RejectsDuplicateAndEmptyNames)
{
  Schema s;
  s.add({"a", ValueKind::kFloat});
  EXPECT_THROW(s.add({"a", ValueKind::kInt}), std::invalid_argument);
  EXPECT_THROW(s.add({"", ValueKind::kInt}), std::invalid_argument);
}

TEST(Record, MakeUsesDefaultsPerKind)
{
  const auto s = make_schema();
  Record r = Record::make(s);
  ASSERT_EQ(r.values.size(), 3u);
  EXPECT_EQ(kind_of(r[0]), ValueKind::kUint);
  EXPECT_EQ(kind_of(r[1]), ValueKind::kInt);
  EXPECT_EQ(std::get<std::string>(r.at("name")), "");
  r.at("gx") = std::int64_t{5};
  EXPECT_EQ(std::get<std::int64_t>(r[1]), 5);
  EXPECT_THROW(r.at("missing"), std::out_of_range);
}

}  // namespace
}  // namespace simport
