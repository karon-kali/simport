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

#include "simport_core/mapping.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simport
{

namespace
{

constexpr std::array<std::pair<Builtin, std::string_view>, 12> kBuiltins{{
  {Builtin::kTimeNs, "$time_ns"},
  {Builtin::kTimeUs, "$time_us"},
  {Builtin::kTimeMs, "$time_ms"},
  {Builtin::kTimeS, "$time_s"},
  {Builtin::kStampNs, "$stamp_ns"},
  {Builtin::kStampUs, "$stamp_us"},
  {Builtin::kStampMs, "$stamp_ms"},
  {Builtin::kStampS, "$stamp_s"},
  {Builtin::kWallTimeNs, "$wall_time_ns"},
  {Builtin::kWallTimeUs, "$wall_time_us"},
  {Builtin::kWallTimeMs, "$wall_time_ms"},
  {Builtin::kWallTimeS, "$wall_time_s"},
}};

bool is_ident_start(char c)
{
  return std::isalpha(static_cast<unsigned char>(c)) || c == '_';
}

bool is_ident_char(char c)
{
  return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

void set_path(FieldMapping & m, const Config & where, const std::string & text)
{
  try {
    m.path_elements = parse_field_path(text);
  } catch (const std::invalid_argument & e) {
    where.error(std::string("invalid field path: ") + e.what());
  }
  m.from = FieldMapping::From::kPath;
  m.path = text;
}

void set_builtin(FieldMapping & m, const Config & where, const std::string & text)
{
  const auto builtin = parse_builtin(text);
  if (!builtin) {
    std::string names;
    for (const auto n : builtin_names()) {
      names += " " + std::string(n);
    }
    where.error("unknown builtin '" + text + "'; available:" + names);
  }
  m.from = FieldMapping::From::kBuiltin;
  m.builtin = *builtin;
}

}  // namespace

// -- builtins ----------------------------------------------------------------------------------

std::optional<Builtin> parse_builtin(std::string_view name)
{
  for (const auto & [b, n] : kBuiltins) {
    if (n == name) {
      return b;
    }
  }
  return std::nullopt;
}

std::string_view to_string(Builtin builtin)
{
  for (const auto & [b, n] : kBuiltins) {
    if (b == builtin) {
      return n;
    }
  }
  return "$unknown";
}

std::vector<std::string_view> builtin_names()
{
  std::vector<std::string_view> out;
  for (const auto & entry : kBuiltins) {
    out.push_back(entry.second);
  }
  return out;
}

Value evaluate(Builtin builtin, const BuiltinInputs & in)
{
  const auto index = static_cast<std::uint8_t>(builtin);
  const Timestamp t = index < 4 ? in.now : (index < 8 ? in.stamp : in.wall);
  const std::int64_t ns = t.count();
  switch (index % 4) {
    case 0:
      return ns;
    case 1:
      return ns / 1000;
    case 2:
      return ns / 1000000;
    default:
      return static_cast<double>(ns) * 1e-9;
  }
}

// -- paths -------------------------------------------------------------------------------------

std::vector<PathElement> parse_field_path(std::string_view path)
{
  if (path.empty()) {
    throw std::invalid_argument("empty path");
  }
  std::vector<PathElement> out;
  std::size_t i = 0;
  const auto fail = [&](const std::string & what) {
      throw std::invalid_argument(
              what + " at position " + std::to_string(i) + " in '" + std::string(path) + "'");
    };
  while (true) {
    if (i >= path.size() || !is_ident_start(path[i])) {
      fail("expected a field name");
    }
    PathElement element;
    const std::size_t start = i;
    while (i < path.size() && is_ident_char(path[i])) {
      ++i;
    }
    element.name = std::string(path.substr(start, i - start));
    if (i < path.size() && path[i] == '[') {
      ++i;
      const std::size_t digits_start = i;
      while (i < path.size() && std::isdigit(static_cast<unsigned char>(path[i]))) {
        ++i;
      }
      if (i == digits_start) {
        fail("expected an array index");
      }
      if (i >= path.size() || path[i] != ']') {
        fail("expected ']'");
      }
      element.index = std::stoull(std::string(path.substr(digits_start, i - digits_start)));
      ++i;
    }
    out.push_back(std::move(element));
    if (i == path.size()) {
      return out;
    }
    if (path[i] != '.') {
      fail("expected '.'");
    }
    ++i;
  }
}

// -- transform ---------------------------------------------------------------------------------

Value ValueTransform::apply(const Value & in) const
{
  if (identity()) {
    return in;
  }
  const auto convert = [this](double x) {
      x = x * scale.value_or(1.0) + offset.value_or(0.0);
      return (nonfinite && !std::isfinite(x)) ? *nonfinite : x;
    };
  if (kind_of(in) != ValueKind::kFloatArray) {
    if (array_ops()) {
      throw ValueError("transpose/reverse/step/resize need an array value");
    }
    if (ramp) {
      const double first = convert(to_double(in));
      std::vector<double> out(ramp->count);
      for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = first + static_cast<double>(i) * ramp->step;
      }
      return out;
    }
    if (!scale && !offset && kind_of(in) != ValueKind::kFloat && is_numeric(kind_of(in))) {
      return in;  // only `nonfinite`, and integers are always finite
    }
    return convert(to_double(in));
  }
  if (ramp) {
    throw ValueError("ramp needs a single number, not an array");
  }
  const auto & original = std::get<std::vector<double>>(in);
  const std::vector<double> * src = &original;
  std::vector<double> transposed;
  if (transpose_rows) {
    const std::size_t rows = *transpose_rows;
    if (original.size() % rows != 0) {
      throw ValueError(
              "cannot transpose " + std::to_string(original.size()) + " elements as " +
              std::to_string(rows) + " rows");
    }
    const std::size_t cols = original.size() / rows;
    transposed.resize(original.size());
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        transposed[c * rows + r] = original[r * cols + c];
      }
    }
    src = &transposed;
  }
  std::vector<double> selected;
  selected.reserve((src->size() + step - 1) / step);
  for (std::size_t i = 0; i < src->size(); i += step) {
    selected.push_back(reverse ? (*src)[src->size() - 1 - i] : (*src)[i]);
  }
  std::vector<double> out;
  if (resize && !selected.empty()) {
    // Nearest-neighbour pick of `resize` evenly spaced elements, keeping both ends.
    const std::size_t n = *resize;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double pos = n == 1 ? 0.0 :
        static_cast<double>(i) * static_cast<double>(selected.size() - 1) /
        static_cast<double>(n - 1);
      out.push_back(selected[static_cast<std::size_t>(std::lround(pos))]);
    }
  } else {
    out = std::move(selected);
  }
  for (double & x : out) {
    x = convert(x);
  }
  return out;
}

// -- MappingSpec -------------------------------------------------------------------------------

MappingSpec MappingSpec::from_sources(
  const std::vector<std::pair<std::string, Config>> & sources, SchemaPtr schema)
{
  MappingSpec spec;
  spec.schema_ = std::move(schema);
  const Schema & s = *spec.schema_;
  std::vector<bool> mapped(s.size(), false);

  for (const auto & [name, entry] : sources) {
    const auto index = s.find(name);
    if (!index || s.field(*index).role == FieldRole::kComputed) {
      continue;
    }
    const FieldSpec & field = s.field(*index);
    FieldMapping m;
    m.target = *index;
    m.target_name = name;
    m.config = entry;
    Config from;
    if (entry.is_scalar()) {
      from = entry;  // `name: path` (text fields)
    } else if (entry.is_map() && entry.has("from")) {
      from = entry["from"];
    } else {
      continue;
    }
    const std::string text = from.as<std::string>();
    if (!text.empty() && text[0] == '$') {
      set_builtin(m, from, text);
    } else {
      set_path(m, from, text);
    }
    if (entry.is_map()) {
      if (entry.has("scale")) {
        m.transform.scale = entry.get<double>("scale");
      }
      if (entry.has("offset")) {
        m.transform.offset = entry.get<double>("offset");
      }
      if (entry.has("nonfinite")) {
        m.transform.nonfinite = entry.get<double>("nonfinite");
      }
      if (entry.has("transpose")) {
        const Config t = entry["transpose"];
        t.expect_keys({"rows"});
        m.transform.transpose_rows = t.get<std::uint32_t>("rows");
        if (*m.transform.transpose_rows == 0) {
          t["rows"].error("rows must be at least 1");
        }
      }
      if (entry.has("ramp")) {
        const Config r = entry["ramp"];
        r.expect_keys({"step", "count"});
        m.transform.ramp = ValueTransform::Ramp{r.get<double>("step"),
          r.get<std::uint32_t>("count")};
        if (m.transform.ramp->count == 0) {
          r["count"].error("count must be at least 1");
        }
        if (field.kind != ValueKind::kFloatArray) {
          r.error("'ramp' makes an array, but field '" + name + "' is " +
            std::string(to_string(field.kind)));
        }
      }
      m.transform.reverse = entry.get<bool>("reverse", false);
      m.transform.step = entry.get<std::uint32_t>("step", 1);
      if (m.transform.step == 0) {
        entry["step"].error("step must be at least 1");
      }
      if (entry.has("resize")) {
        m.transform.resize = entry.get<std::uint32_t>("resize");
        if (*m.transform.resize == 0) {
          entry["resize"].error("resize must be at least 1");
        }
      }
    }
    if (m.transform.array_ops() && field.kind != ValueKind::kFloatArray) {
      entry.error("'transpose', 'reverse', 'step' and 'resize' rearrange array elements, but '" +
        name + "' is not an array (put it in a group)");
    }
    if (m.transform.array_ops() && m.from == FieldMapping::From::kBuiltin) {
      entry.error("'transpose', 'reverse', 'step' and 'resize' need an array source");
    }
    if (m.transform.array_ops() && m.transform.ramp) {
      entry.error("'ramp' makes an array from a number; it cannot be combined with "
        "'transpose', 'reverse', 'step' or 'resize'");
    }
    const bool numeric_target = is_numeric(field.kind) || field.kind == ValueKind::kFloatArray;
    if (m.from == FieldMapping::From::kBuiltin && !numeric_target) {
      entry.error("builtin values are numeric but field '" + name + "' is " +
        std::string(to_string(field.kind)));
    }
    if (!m.transform.identity() && !numeric_target) {
      entry.error("'scale' and the like need a numeric field, but '" + name + "' is " +
        std::string(to_string(field.kind)));
    }
    mapped[*index] = true;
    (m.from == FieldMapping::From::kPath ? spec.paths_ : spec.builtins_).push_back(std::move(m));
  }

  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s.field(i).role == FieldRole::kInput && !mapped[i]) {
      spec.unmapped_.push_back(i);
    }
  }
  const auto by_target = [](const FieldMapping & x, const FieldMapping & y) {
      return x.target < y.target;
    };
  std::sort(spec.paths_.begin(), spec.paths_.end(), by_target);
  std::sort(spec.builtins_.begin(), spec.builtins_.end(), by_target);
  return spec;
}

Record MappingSpec::make_record() const
{
  return Record::make(schema_);
}

void MappingSpec::apply_builtins(Record & record, const BuiltinInputs & inputs) const
{
  for (const auto & m : builtins_) {
    record[m.target] = m.transform.apply(evaluate(m.builtin, inputs));
  }
}

}  // namespace simport
