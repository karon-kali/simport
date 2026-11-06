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

#include "simport_core/schema.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simport
{

Schema::Schema(std::vector<FieldSpec> fields)
{
  for (auto & f : fields) {
    add(std::move(f));
  }
}

std::size_t Schema::add(FieldSpec field)
{
  if (field.name.empty()) {
    throw std::invalid_argument("schema field name must not be empty");
  }
  if (index_.contains(field.name)) {
    throw std::invalid_argument("duplicate schema field '" + field.name + "'");
  }
  const std::size_t index = fields_.size();
  index_.emplace(field.name, index);
  fields_.push_back(std::move(field));
  return index;
}

std::optional<std::size_t> Schema::find(std::string_view name) const
{
  const auto it = index_.find(name);
  if (it == index_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::size_t Schema::index_of(std::string_view name) const
{
  const auto index = find(name);
  if (!index) {
    throw std::out_of_range("no schema field named '" + std::string(name) + "'");
  }
  return *index;
}

std::vector<std::string> Schema::names(std::optional<FieldRole> role) const
{
  std::vector<std::string> out;
  for (const auto & f : fields_) {
    if (!role || f.role == *role) {
      out.push_back(f.name);
    }
  }
  return out;
}

Record Record::make(SchemaPtr schema)
{
  Record r;
  r.values.reserve(schema->size());
  for (const auto & f : schema->fields()) {
    r.values.push_back(default_value(f.kind));
  }
  r.schema = std::move(schema);
  return r;
}

Value & Record::at(std::string_view name)
{
  return values.at(schema->index_of(name));
}

const Value & Record::at(std::string_view name) const
{
  return values.at(schema->index_of(name));
}

}  // namespace simport
