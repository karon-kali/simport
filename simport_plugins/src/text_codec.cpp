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

#include "simport_plugins/text_codec.hpp"

#include <cctype>
#include <cinttypes>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

namespace
{

template<typename ... Args>
std::string cformat(const std::string & fmt, Args... args)
{
  const int n = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
  if (n < 0) {
    return {};
  }
  std::string out(static_cast<std::size_t>(n), '\0');
  std::snprintf(out.data(), out.size() + 1, fmt.c_str(), args ...);
  return out;
}

std::string_view trim(std::string_view s)
{
  while (!s.empty() && s.front() == ' ') {
    s.remove_prefix(1);
  }
  while (!s.empty() && s.back() == ' ') {
    s.remove_suffix(1);
  }
  return s;
}

// Days since 1970-01-01 <-> civil date (H. Hinnant's algorithms).
std::int64_t days_from_civil(std::int64_t y, unsigned m, unsigned d)
{
  y -= m <= 2;
  const std::int64_t era = (y >= 0 ? y : y - 399) / 400;
  const auto yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<std::int64_t>(doe) - 719468;
}

void civil_from_days(std::int64_t z, std::int64_t & y, unsigned & m, unsigned & d)
{
  z += 719468;
  const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const auto doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = static_cast<std::int64_t>(yoe) + era * 400 + (m <= 2);
}

int parse_digits(std::string_view s, std::size_t pos, std::size_t n)
{
  int v = 0;
  if (pos + n > s.size()) {
    throw std::invalid_argument("too short");
  }
  for (std::size_t i = pos; i < pos + n; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) {
      throw std::invalid_argument("expected digits");
    }
    v = v * 10 + (s[i] - '0');
  }
  return v;
}

std::uint64_t parse_unsigned(std::string_view s, int base)
{
  std::uint64_t v = 0;
  const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v, base);
  if (ec != std::errc{} || end != s.data() + s.size() || s.empty()) {
    throw std::invalid_argument("not a number");
  }
  return v;
}

}  // namespace

// -- TextFormat ---------------------------------------------------------------------------------

TextFormat TextFormat::parse(std::string_view spec)
{
  TextFormat f;
  const auto with_decimals = [&](std::string_view prefix, Kind kind, int default_decimals) {
      f.kind = kind;
      f.decimals = default_decimals;
      const std::string_view rest = spec.substr(prefix.size());
      if (!rest.empty()) {
        if (rest.size() < 2 || rest[0] != '.') {
          throw std::invalid_argument("expected '" + std::string(prefix) + ".<decimals>'");
        }
        f.decimals = static_cast<int>(parse_unsigned(rest.substr(1), 10));
        if (f.decimals > 9) {
          throw std::invalid_argument("at most 9 decimals");
        }
      }
      return f;
    };
  if (spec.starts_with("nmea_lat")) {
    return with_decimals("nmea_lat", Kind::kNmeaLat, 4);
  }
  if (spec.starts_with("nmea_lon")) {
    return with_decimals("nmea_lon", Kind::kNmeaLon, 4);
  }
  if (spec.starts_with("hhmmss")) {
    f.kind = Kind::kTimeOfDay;
    const std::string_view rest = spec.substr(6);
    if (!rest.empty()) {
      if (rest[0] != '.' || rest.size() < 2 || rest.find_first_not_of('s', 1) != rest.npos) {
        throw std::invalid_argument("expected hhmmss, hhmmss.s, hhmmss.ss, ...");
      }
      f.decimals = static_cast<int>(rest.size() - 1);
    }
    return f;
  }
  if (spec == "ddmmyy") {
    f.kind = Kind::kDate;
    return f;
  }
  if (spec.empty() || spec[0] != '%') {
    throw std::invalid_argument(
            "unknown format '" + std::string(spec) +
            "'; use a printf spec (%d %u %x %X %o %f %e %g %s), nmea_lat, nmea_lon, hhmmss[.ss] "
            "or ddmmyy");
  }
  std::size_t i = 1;
  while (i < spec.size() && std::string_view("-+ #0").find(spec[i]) != std::string_view::npos) {
    ++i;
  }
  while (i < spec.size() && std::isdigit(static_cast<unsigned char>(spec[i]))) {
    ++i;
  }
  if (i < spec.size() && spec[i] == '.') {
    ++i;
    while (i < spec.size() && std::isdigit(static_cast<unsigned char>(spec[i]))) {
      ++i;
    }
  }
  if (i + 1 != spec.size() ||
    std::string_view("diuxXofFeEgGs").find(spec[i]) == std::string_view::npos)
  {
    throw std::invalid_argument("invalid printf spec '" + std::string(spec) + "'");
  }
  f.conversion = spec[i];
  const std::string head(spec.substr(0, i));
  switch (f.conversion) {
    case 'd':
    case 'i':
      f.printf_spec = head + PRId64;
      break;
    case 'u':
      f.printf_spec = head + PRIu64;
      break;
    case 'x':
      f.printf_spec = head + PRIx64;
      break;
    case 'X':
      f.printf_spec = head + PRIX64;
      break;
    case 'o':
      f.printf_spec = head + PRIo64;
      break;
    default:
      f.printf_spec = std::string(spec);
  }
  return f;
}

ValueKind TextFormat::kind_of_value() const
{
  if (kind != Kind::kPrintf) {
    return ValueKind::kFloat;
  }
  switch (conversion) {
    case 'd':
    case 'i':
      return ValueKind::kInt;
    case 'u':
    case 'x':
    case 'X':
    case 'o':
      return ValueKind::kUint;
    case 's':
      return ValueKind::kString;
    default:
      return ValueKind::kFloat;
  }
}

std::string TextFormat::render(const Value & value) const
{
  switch (kind) {
    case Kind::kPrintf:
      switch (kind_of_value()) {
        case ValueKind::kInt:
          return cformat(printf_spec, saturate_cast<std::int64_t>(value));
        case ValueKind::kUint:
          return cformat(printf_spec, saturate_cast<std::uint64_t>(value));
        case ValueKind::kString: {
            const std::string s = kind_of(value) == ValueKind::kString ?
              std::get<std::string>(value) : to_display_string(value);
            return cformat(printf_spec, s.c_str());
          }
        default:
          return cformat(printf_spec, to_double(value));
      }
    case Kind::kNmeaLat:
    case Kind::kNmeaLon: {
        const double v = to_double(value);
        if (!std::isfinite(v)) {
          return ",";
        }
        const bool lat = kind == Kind::kNmeaLat;
        const double scale = std::pow(10.0, decimals);
        // Round in integer units of the last minute decimal to avoid "60.0000" minutes.
        const auto units = static_cast<std::int64_t>(std::llround(std::abs(v) * 60.0 * scale));
        const std::int64_t per_degree = static_cast<std::int64_t>(60 * scale);
        const std::int64_t deg = units / per_degree;
        const double minutes = static_cast<double>(units % per_degree) / scale;
        const char hemi = lat ? (v < 0 ? 'S' : 'N') : (v < 0 ? 'W' : 'E');
        const int min_width = decimals > 0 ? 3 + decimals : 2;
        return cformat(
          std::string(lat ? "%02" PRId64 : "%03" PRId64) + "%0*.*f,%c", deg, min_width,
          decimals, minutes, hemi);
      }
    case Kind::kTimeOfDay: {
        const double v = to_double(value);
        if (!std::isfinite(v)) {
          return {};
        }
        const auto scale = static_cast<std::int64_t>(std::llround(std::pow(10.0, decimals)));
        const std::int64_t day = 86400 * scale;
        std::int64_t t = std::llround(v * static_cast<double>(scale)) % day;
        if (t < 0) {
          t += day;
        }
        const std::int64_t s = t / scale;
        std::string out = cformat(
          "%02" PRId64 "%02" PRId64 "%02" PRId64, s / 3600, s / 60 % 60, s % 60);
        if (decimals > 0) {
          out += cformat(".%0*" PRId64, decimals, t % scale);
        }
        return out;
      }
    case Kind::kDate: {
        const double v = to_double(value);
        if (!std::isfinite(v)) {
          return {};
        }
        std::int64_t y;
        unsigned m;
        unsigned d;
        civil_from_days(static_cast<std::int64_t>(std::floor(v / 86400.0)), y, m, d);
        return cformat("%02u%02u%02" PRId64, d, m, ((y % 100) + 100) % 100);
      }
  }
  return {};
}

std::optional<Value> TextFormat::parse_value(std::string_view token) const
{
  token = trim(token);
  if (token.empty()) {
    return std::nullopt;
  }
  switch (kind) {
    case Kind::kPrintf:
      switch (conversion) {
        case 'd':
        case 'i':
          return parse_integer<std::int64_t>(token);
        case 'u':
          return parse_integer<std::uint64_t>(token);
        case 'x':
        case 'X':
          return parse_unsigned(token, 16);
        case 'o':
          return parse_unsigned(token, 8);
        case 's':
          return std::string(token);
        default:
          return parse_double(token);
      }
    case Kind::kNmeaLat:
    case Kind::kNmeaLon: {
        const auto comma = token.find(',');
        if (comma == std::string_view::npos) {
          throw std::invalid_argument("expected '<ddmm.mmmm>,<hemisphere>'");
        }
        const std::string_view number = token.substr(0, comma);
        const std::string_view hemi = token.substr(comma + 1);
        if (number.empty()) {
          return std::nullopt;
        }
        const std::size_t deg_digits = kind == Kind::kNmeaLat ? 2 : 3;
        const int deg = parse_digits(number, 0, deg_digits);
        const double minutes = parse_double(number.substr(deg_digits));
        double v = deg + minutes / 60.0;
        if (hemi == "S" || hemi == "W") {
          v = -v;
        } else if (hemi != "N" && hemi != "E" && !hemi.empty()) {
          throw std::invalid_argument("bad hemisphere '" + std::string(hemi) + "'");
        }
        return v;
      }
    case Kind::kTimeOfDay: {
        const int h = parse_digits(token, 0, 2);
        const int m = parse_digits(token, 2, 2);
        const double s = parse_double(token.substr(4));
        return h * 3600.0 + m * 60.0 + s;
      }
    case Kind::kDate: {
        if (token.size() != 6) {
          throw std::invalid_argument("expected ddmmyy");
        }
        const int d = parse_digits(token, 0, 2);
        const int m = parse_digits(token, 2, 2);
        const int y = parse_digits(token, 4, 2);
        return static_cast<double>(
          days_from_civil(2000 + y, static_cast<unsigned>(m), static_cast<unsigned>(d)) * 86400);
      }
  }
  return std::nullopt;
}

// -- TextCodec ----------------------------------------------------------------------------------

void TextCodec::configure(const Config & config, const Config & envelope, Context & /*context*/)
{
  if (envelope.defined() && !envelope.is_null()) {
    config["format"].error("text messages cannot use the port's binary packet envelope");
  }
  const Config fmt_node = config.required("format");
  const std::string fmt = fmt_node.as<std::string>();
  if (fmt.empty()) {
    fmt_node.error("format must not be empty");
  }
  auto schema = std::make_shared<Schema>();
  segments_.clear();

  const auto fail = [&](std::size_t pos, const std::string & what) {
      fmt_node.error(what + " (at character " + std::to_string(pos) + " of the format)");
    };
  int n_begin = 0;
  int n_end = 0;
  int n_checksum = 0;
  std::string literal;
  const auto flush_literal = [&]() {
      if (!literal.empty()) {
        segments_.push_back(Segment{Segment::Kind::kLiteral, literal, 0, {}});
        literal.clear();
      }
    };

  for (std::size_t i = 0; i < fmt.size(); ++i) {
    const char c = fmt[i];
    if (c == '}') {
      if (i + 1 < fmt.size() && fmt[i + 1] == '}') {
        literal += '}';
        ++i;
        continue;
      }
      fail(i, "unmatched '}' (write '}}' for a literal brace)");
    }
    if (c != '{') {
      literal += c;
      continue;
    }
    if (i + 1 < fmt.size() && fmt[i + 1] == '{') {
      literal += '{';
      ++i;
      continue;
    }
    const auto close = fmt.find('}', i);
    if (close == std::string::npos) {
      fail(i, "unterminated '{'");
    }
    const std::string body = fmt.substr(i + 1, close - i - 1);
    const auto colon = body.find(':');
    const std::string name = body.substr(0, colon);
    const std::string spec = colon == std::string::npos ? "" : body.substr(colon + 1);
    flush_literal();

    if (name == "$begin" || name == "$end") {
      if (!spec.empty()) {
        fail(i, name + " takes no format");
      }
      const bool begin = name == "$begin";
      ++(begin ? n_begin : n_end);
      segments_.push_back(
        Segment{begin ? Segment::Kind::kBegin : Segment::Kind::kEnd, {}, 0, {}});
    } else if (name == "$checksum") {
      ++n_checksum;
      Segment seg{Segment::Kind::kChecksum, {}, 0, {}};
      try {
        seg.format = TextFormat::parse(spec.empty() ? "%02X" : spec);
      } catch (const std::invalid_argument & e) {
        fail(i, e.what());
      }
      if (seg.format.kind_of_value() != ValueKind::kUint) {
        fail(i, "the checksum format must be an unsigned conversion (%u, %x, %X, %o)");
      }
      segments_.push_back(std::move(seg));
    } else {
      if (name.empty() || !(std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_') ||
        name.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
      {
        fail(i, "invalid placeholder '{" + body + "}'");
      }
      Segment seg{Segment::Kind::kField, {}, 0, {}};
      try {
        seg.format = TextFormat::parse(spec.empty() ? "%g" : spec);
      } catch (const std::invalid_argument & e) {
        fail(i, "field '" + name + "': " + e.what());
      }
      const ValueKind kind = seg.format.kind_of_value();
      if (const auto existing = schema->find(name)) {
        if (schema->field(*existing).kind != kind) {
          fail(i, "field '" + name + "' is used with formats of different kinds");
        }
        seg.field = *existing;
      } else {
        seg.field = schema->add(FieldSpec{name, kind, FieldRole::kInput});
      }
      segments_.push_back(std::move(seg));
    }
    i = close;
  }
  flush_literal();

  if (n_begin > 1 || n_end > 1 || n_checksum > 1) {
    fmt_node.error("{$begin}, {$end} and {$checksum} may each appear at most once");
  }
  // Ordering: begin < end <= checksum.
  std::optional<std::size_t> begin_at, end_at, checksum_at;
  for (std::size_t s = 0; s < segments_.size(); ++s) {
    switch (segments_[s].kind) {
      case Segment::Kind::kBegin: begin_at = s; break;
      case Segment::Kind::kEnd: end_at = s; break;
      case Segment::Kind::kChecksum: checksum_at = s; break;
      default: break;
    }
  }
  if (begin_at && end_at && *end_at < *begin_at) {
    fmt_node.error("{$end} must come after {$begin}");
  }
  if (checksum_at && ((begin_at && *begin_at > *checksum_at) ||
    (end_at && *end_at > *checksum_at)))
  {
    fmt_node.error("{$checksum} must come after the checksummed text ({$begin}..{$end})");
  }
  if (n_checksum && !config.has("checksum")) {
    fmt_node.error("the format uses {$checksum}; set 'checksum' to an algorithm, e.g. nmea");
  }
  if (!n_checksum && config.has("checksum")) {
    config["checksum"].error("'checksum' is set but the format has no {$checksum} placeholder");
  }
  if (n_checksum) {
    checksum_ = Checksum::parse(config["checksum"]);
  } else {
    checksum_.reset();
  }

  // Decoding splits a field at the next literal, so fields need a literal after them.
  for (std::size_t s = 0; s < segments_.size(); ++s) {
    const Segment & seg = segments_[s];
    const bool self_delimiting = seg.kind == Segment::Kind::kField &&
      (seg.format.kind == TextFormat::Kind::kNmeaLat ||
      seg.format.kind == TextFormat::Kind::kNmeaLon);
    if ((seg.kind != Segment::Kind::kField && seg.kind != Segment::Kind::kChecksum) ||
      self_delimiting)
    {
      continue;
    }
    for (std::size_t t = s + 1; t < segments_.size(); ++t) {
      const auto k = segments_[t].kind;
      if (k == Segment::Kind::kLiteral) {
        break;
      }
      if (k == Segment::Kind::kField || k == Segment::Kind::kChecksum) {
        fmt_node.error("two placeholders must be separated by literal text");
      }
    }
  }

  // Where the placeholders get their values: `name: path` or `name: { from: ..., ... }`, as a
  // mapping or as a list with one placeholder per line.
  sources_.clear();
  const Config fields = config["fields"];
  std::vector<std::pair<std::string, Config>> entries;
  if (fields.is_map()) {
    entries = fields.entries();
  } else {
    for (const Config & item : fields.items()) {
      if (!item.is_map() || item.size() != 1) {
        item.error("expected one placeholder per line: `name: path` or `name: { from: ... }`");
      }
      entries.push_back(item.entries().front());
    }
  }
  for (const auto & [name, spec] : entries) {
    if (!schema->find(name)) {
      spec.error("the format has no placeholder {" + name + "}");
    }
    if (spec.is_map()) {
      spec.expect_keys({"from", "scale", "offset", "nonfinite", "transpose", "reverse", "step",
          "resize", "ramp"});
    }
    sources_.emplace_back(name, spec);
  }
  schema_ = std::move(schema);
}

std::string TextCodec::render(const Record & record) const
{
  std::string out;
  std::size_t begin = 0;
  std::optional<std::size_t> end;
  std::optional<std::size_t> checksum_pos;
  const TextFormat * checksum_format = nullptr;
  for (const Segment & seg : segments_) {
    switch (seg.kind) {
      case Segment::Kind::kLiteral: out += seg.literal; break;
      case Segment::Kind::kField: out += seg.format.render(record[seg.field]); break;
      case Segment::Kind::kBegin: begin = out.size(); break;
      case Segment::Kind::kEnd: end = out.size(); break;
      case Segment::Kind::kChecksum:
        checksum_pos = out.size();
        checksum_format = &seg.format;
        break;
    }
  }
  if (checksum_pos) {
    const std::size_t e = end.value_or(*checksum_pos);
    const auto * bytes = reinterpret_cast<const std::uint8_t *>(out.data());
    const std::uint64_t cs = checksum_->compute(ByteView(bytes + begin, e - begin));
    out.insert(*checksum_pos, checksum_format->render(Value{cs}));
  }
  return out;
}

void TextCodec::encode(const Record & record, std::vector<Frame> & out)
{
  const std::string text = render(record);
  out.push_back(Frame{Bytes(text.begin(), text.end()), {}, {}});
}

DecodeResult TextCodec::decode(const Frame & frame, Record & out)
{
  using Code = DecodeResult::Code;
  const std::string_view text(reinterpret_cast<const char *>(frame.data.data()), frame.data.size());
  std::size_t pos = 0;
  std::size_t begin = 0;
  std::optional<std::size_t> end;
  std::optional<std::size_t> checksum_start;
  std::string_view checksum_token;
  const TextFormat * checksum_format = nullptr;

  for (std::size_t s = 0; s < segments_.size(); ++s) {
    const Segment & seg = segments_[s];
    switch (seg.kind) {
      case Segment::Kind::kLiteral:
        if (text.compare(pos, seg.literal.size(), seg.literal) != 0) {
          return DecodeResult::failure(
            Code::kNoMatch, std::format("expected '{}' at offset {}", seg.literal, pos));
        }
        pos += seg.literal.size();
        continue;
      case Segment::Kind::kBegin: begin = pos; continue;
      case Segment::Kind::kEnd: end = pos; continue;
      default: break;
    }
    // A field or the checksum: find where it stops.
    std::size_t stop = text.size();
    const bool nmea = seg.kind == Segment::Kind::kField &&
      (seg.format.kind == TextFormat::Kind::kNmeaLat ||
      seg.format.kind == TextFormat::Kind::kNmeaLon);
    if (nmea) {
      const auto comma = text.find(',', pos);
      if (comma == std::string_view::npos) {
        return DecodeResult::failure(Code::kNoMatch, "expected '<ddmm.mmmm>,<hemisphere>'");
      }
      stop = comma + 1;
      if (stop < text.size() && std::isalpha(static_cast<unsigned char>(text[stop]))) {
        ++stop;
      }
    } else {
      for (std::size_t t = s + 1; t < segments_.size(); ++t) {
        if (segments_[t].kind == Segment::Kind::kLiteral) {
          stop = text.find(segments_[t].literal, pos);
          if (stop == std::string_view::npos) {
            return DecodeResult::failure(
              Code::kNoMatch, std::format("expected '{}' after offset {}",
              segments_[t].literal, pos));
          }
          break;
        }
      }
    }
    const std::string_view token = text.substr(pos, stop - pos);
    if (seg.kind == Segment::Kind::kChecksum) {
      checksum_start = pos;
      checksum_token = token;
      checksum_format = &seg.format;
    } else {
      try {
        if (auto v = seg.format.parse_value(token)) {
          out[seg.field] = std::move(*v);
        }
      } catch (const std::exception & e) {
        return DecodeResult::failure(
          Code::kBadValue, std::format(
            "field '{}': cannot parse '{}': {}", schema_->field(seg.field).name, token,
            e.what()));
      }
    }
    pos = stop;
  }
  if (pos != text.size()) {
    return DecodeResult::failure(
      Code::kBadLength, std::format("{} unexpected trailing bytes", text.size() - pos));
  }
  if (checksum_format != nullptr) {
    std::uint64_t got = 0;
    try {
      const auto v = checksum_format->parse_value(checksum_token);
      if (!v) {
        return DecodeResult::failure(Code::kBadChecksum, "checksum missing");
      }
      got = saturate_cast<std::uint64_t>(*v);
    } catch (const std::exception &) {
      return DecodeResult::failure(Code::kBadChecksum, "unreadable checksum");
    }
    const std::size_t e = end.value_or(*checksum_start);
    const auto * bytes = reinterpret_cast<const std::uint8_t *>(text.data());
    const std::uint64_t expected = checksum_->compute(ByteView(bytes + begin, e - begin));
    if (got != expected) {
      return DecodeResult::failure(
        Code::kBadChecksum, std::format("expected {:#x}, got {:#x}", expected, got));
    }
  }
  out.stamp = frame.stamp;
  return DecodeResult::success();
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::TextCodec, simport::Codec)
