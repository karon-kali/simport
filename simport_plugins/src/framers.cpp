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

#include "simport_plugins/framers.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "simport_plugins/binary_codec.hpp"

namespace simport::plugins
{

Bytes parse_bytes(const Config & node)
{
  if (node.is_sequence()) {
    return node.as<Bytes>();
  }
  const std::string s = node.as<std::string>();
  return Bytes(s.begin(), s.end());
}

namespace
{
void emit(Bytes & buffer, std::size_t n, Timestamp stamp, const FrameCallback & on_frame)
{
  Frame f;
  f.data.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
  f.stamp = stamp;
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
  on_frame(std::move(f));
}
}  // namespace

// -- FixedLength -------------------------------------------------------------------------------

void FixedLength::configure(const Config & config, Context &)
{
  config.expect_keys({"plugin", "size"});
  size_ = config.get<std::uint32_t>("size");
  if (size_ == 0) {
    config["size"].error("size must be at least 1");
  }
}

void FixedLength::feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame)
{
  buffer_.insert(buffer_.end(), data.begin(), data.end());
  while (buffer_.size() >= size_) {
    ++stats_.frames;
    emit(buffer_, size_, stamp, on_frame);
  }
}

// -- Delimiter ---------------------------------------------------------------------------------

void Delimiter::configure(const Config & config, Context &)
{
  config.expect_keys({"plugin", "delimiter", "include_delimiter", "max_size"});
  delimiter_ = parse_bytes(config.required("delimiter"));
  if (delimiter_.empty()) {
    config["delimiter"].error("delimiter must not be empty");
  }
  include_ = config.get<bool>("include_delimiter", true);
  max_size_ = config.get<std::uint32_t>("max_size", 1024);
}

void Delimiter::feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame)
{
  buffer_.insert(buffer_.end(), data.begin(), data.end());
  for (;; ) {
    const auto it = std::search(buffer_.begin(), buffer_.end(), delimiter_.begin(),
        delimiter_.end());
    if (it == buffer_.end()) {
      if (buffer_.size() > max_size_) {
        stats_.discarded_bytes += buffer_.size();
        buffer_.clear();
      }
      return;
    }
    const auto end = static_cast<std::size_t>(it - buffer_.begin()) + delimiter_.size();
    ++stats_.frames;
    Frame f;
    f.data.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(
        include_ ? end : end - delimiter_.size()));
    f.stamp = stamp;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(end));
    on_frame(std::move(f));
  }
}

// -- SyncLength --------------------------------------------------------------------------------

SyncLength::Case SyncLength::parse_case(const Config & node, bool allow_when)
{
  Case c;
  if (allow_when && node.has("when")) {
    const Config when = node["when"];
    when.expect_keys({"offset", "mask", "equals"});
    c.when_offset = when.get<std::uint32_t>("offset");
    c.when_mask = when.get<std::uint8_t>("mask", 0xFF);
    c.when_equals = when.get<std::uint8_t>("equals");
    if ((c.when_equals & ~c.when_mask) != 0) {
      when["equals"].error("'equals' has bits outside 'mask', so it can never match");
    }
  }
  if (node.has("size") == node.has("length")) {
    node.error("specify either a fixed 'size' or a 'length' field");
  }
  if (node.has("size")) {
    if (node.has("adjust")) {
      node["adjust"].error("'adjust' applies to 'length', not to a fixed 'size'");
    }
    c.size = node.get<std::uint32_t>("size");
    if (*c.size == 0) {
      node["size"].error("size must be at least 1");
    }
    return c;
  }
  const Config length = node["length"];
  length.expect_keys({"offset", "type", "endian"});
  c.length_offset = length.get<std::uint32_t>("offset");
  const std::string type = length.get<std::string>("type", "u8");
  if (type == "u8") {
    c.length_size = 1;
  } else if (type == "u16") {
    c.length_size = 2;
  } else if (type == "u32") {
    c.length_size = 4;
  } else {
    length["type"].error("expected u8, u16 or u32");
  }
  const std::string endian = length.get<std::string>("endian", "little");
  if (endian != "little" && endian != "big") {
    length["endian"].error("expected 'little' or 'big'");
  }
  c.length_big = endian == "big";
  c.adjust = node.get<std::int64_t>("adjust", 0);
  return c;
}

void SyncLength::derive_from_envelope(const Config & envelope)
{
  // Sizes of the envelope's items; the body's size is whatever the length says.
  const auto size_of = [](const Config & spec) -> std::optional<std::size_t> {
      std::string type;
      Config value;
      if (spec.is_scalar()) {
        type = spec.scalar();
      } else if (spec.is_map()) {
        for (const auto & [key, v] : spec.entries()) {
          if (find_wire_type(key)) {
            type = key;
            value = v;
          }
        }
        if (spec.has("type")) {
          type = spec.get<std::string>("type");
        }
        if (!value.defined() || value.is_null()) {
          value = spec["value"];
        }
      }
      const auto info = find_wire_type(type);
      if (!info) {
        return std::nullopt;
      }
      if (info->type == WireType::kBytes) {
        if (!value.defined() || value.is_null()) {
          return std::nullopt;  // variable length
        }
        return value.as<Bytes>().size();
      }
      return info->size;
    };
  if (!envelope.has("start")) {
    envelope.error("to cut the byte stream into packets, the envelope needs 'start' bytes "
      "(or give the device a 'framer:')");
  }
  sync_ = envelope["start"].as<Bytes>();
  const bool default_big = envelope.has("endian") && envelope.get<std::string>("endian") == "big";

  Case c;
  std::size_t offset = sync_.size();
  std::size_t uncounted = sync_.size();
  std::optional<std::vector<std::string>> parts;
  std::vector<std::pair<std::string, std::size_t>> sizes;
  bool after_body = false;
  for (const Config & item : envelope["fields"].items()) {
    if (item.is_scalar() && item.scalar() == "body") {
      after_body = true;
      continue;
    }
    const auto [name, spec] = item.entries().front();
    const std::optional<std::size_t> size = name == "pad" ?
      std::optional<std::size_t>(spec.as<std::uint32_t>()) : size_of(spec);
    if (!size) {
      item.error("the framer needs fixed-size envelope fields around the body");
    }
    sizes.emplace_back(name, *size);
    if (spec.is_map() && spec.has("length_of")) {
      if (after_body) {
        spec.error("the framer needs the length field before the body");
      }
      const Config l = spec["length_of"];
      parts = l.is_scalar() ? std::vector<std::string>{l.scalar()} :
      l.as<std::vector<std::string>>();
      c.length_offset = offset;
      c.length_size = *size;
      c.length_big = spec.has("endian") ? spec.get<std::string>("endian") == "big" : default_big;
    }
    offset += after_body ? 0 : *size;
  }
  if (!parts) {
    envelope.error("to cut the byte stream into packets, the envelope needs a length field "
      "(`length: { u16, length_of: [body, crc] }`), or give the device a 'framer:'");
  }
  if (std::find(parts->begin(), parts->end(), "body") == parts->end()) {
    envelope.error("the length field must count the body for the framer to find packet ends");
  }
  for (const auto & [name, size] : sizes) {
    if (std::find(parts->begin(), parts->end(), name) == parts->end()) {
      uncounted += size;
    }
  }
  c.adjust = static_cast<std::int64_t>(uncounted);
  cases_.push_back(c);
}

void SyncLength::configure(const Config & config, Context &)
{
  cases_.clear();
  if (config.has("fields") || config.has("start")) {
    // A packet envelope: derive everything from it.
    derive_from_envelope(config);
    max_size_ = 65536;
    return;
  }
  config.expect_keys({"plugin", "sync", "length", "adjust", "size", "cases", "max_size"});
  sync_ = parse_bytes(config.required("sync"));
  if (sync_.empty()) {
    config["sync"].error("sync must not be empty");
  }
  if (config.has("cases")) {
    for (const char * key : {"length", "adjust", "size"}) {
      if (config.has(key)) {
        config[key].error(std::string("'") + key + "' belongs inside a case when 'cases' is used");
      }
    }
    const auto items = config["cases"].items();
    if (items.empty()) {
      config["cases"].error("give at least one case");
    }
    for (const Config & item : items) {
      item.expect_keys({"when", "size", "length", "adjust"});
      cases_.push_back(parse_case(item, true));
    }
  } else {
    if (!config.has("length")) {
      config.error("missing 'length' (or 'cases')");
    }
    cases_.push_back(parse_case(config, false));
  }
  max_size_ = config.get<std::uint32_t>("max_size", 1024);
}

void SyncLength::feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame)
{
  buffer_.insert(buffer_.end(), data.begin(), data.end());
  for (;; ) {
    // Align the buffer on the next sync pattern.
    const auto it = std::search(buffer_.begin(), buffer_.end(), sync_.begin(), sync_.end());
    const bool found = it != buffer_.end();
    // If not found, keep a possible partial sync pattern at the end.
    const std::size_t skip = found ? static_cast<std::size_t>(it - buffer_.begin()) :
      (buffer_.size() >= sync_.size() ? buffer_.size() - (sync_.size() - 1) : 0);
    if (skip > 0) {
      stats_.discarded_bytes += skip;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(skip));
    }
    if (!found || buffer_.size() < sync_.size()) {
      return;
    }

    // Pick the first case whose condition holds. Wait for more bytes if a condition byte has
    // not arrived yet.
    const Case * selected = nullptr;
    bool need_more = false;
    for (const Case & c : cases_) {
      if (!c.when_offset) {
        selected = &c;
        break;
      }
      if (buffer_.size() <= *c.when_offset) {
        need_more = true;
        break;
      }
      if ((buffer_[*c.when_offset] & c.when_mask) == c.when_equals) {
        selected = &c;
        break;
      }
    }
    if (need_more) {
      return;
    }

    std::int64_t total = 0;
    if (selected != nullptr && selected->size) {
      total = static_cast<std::int64_t>(*selected->size);
    } else if (selected != nullptr) {
      const std::size_t header = selected->length_offset + selected->length_size;
      if (buffer_.size() < header) {
        return;
      }
      std::uint64_t length = 0;
      for (std::size_t i = 0; i < selected->length_size; ++i) {
        const std::size_t shift = 8 * (selected->length_big ? selected->length_size - 1 - i : i);
        length |= static_cast<std::uint64_t>(buffer_[selected->length_offset + i]) << shift;
      }
      total = static_cast<std::int64_t>(length) + selected->adjust;
      if (total < static_cast<std::int64_t>(header)) {
        total = 0;  // implausible
      }
    }
    if (selected == nullptr || total < static_cast<std::int64_t>(sync_.size()) ||
      total > static_cast<std::int64_t>(max_size_))
    {
      // No case matches, or an implausible length: probably a false sync. Skip one byte.
      ++stats_.discarded_bytes;
      buffer_.erase(buffer_.begin());
      continue;
    }
    if (buffer_.size() < static_cast<std::size_t>(total)) {
      return;
    }
    ++stats_.frames;
    emit(buffer_, static_cast<std::size_t>(total), stamp, on_frame);
  }
}

// -- Http --------------------------------------------------------------------------------------

void Http::configure(const Config & config, Context &)
{
  config.expect_keys({"plugin", "max_size"});
  max_size_ = config.get<std::uint32_t>("max_size", 1 << 20);
}

void Http::feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame)
{
  static constexpr std::string_view kEnd = "\r\n\r\n";
  buffer_.insert(buffer_.end(), data.begin(), data.end());
  for (;; ) {
    const auto it = std::search(buffer_.begin(), buffer_.end(), kEnd.begin(), kEnd.end());
    if (it == buffer_.end()) {
      if (buffer_.size() > max_size_) {
        stats_.discarded_bytes += buffer_.size();
        buffer_.clear();
      }
      return;
    }
    const auto header_size = static_cast<std::size_t>(it - buffer_.begin()) + kEnd.size();
    // Header names are case-insensitive.
    std::string header(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header_size));
    std::string lower = header;
    std::transform(lower.begin(), lower.end(), lower.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    std::size_t body = 0;
    bool bad = lower.find("\r\ntransfer-encoding: chunked") != std::string::npos;
    if (const auto pos = lower.find("\r\ncontent-length:"); pos != std::string::npos) {
      try {
        body = std::stoul(lower.substr(pos + 17));
      } catch (const std::exception &) {
        bad = true;
      }
    }
    if (bad || header_size + body > max_size_) {
      stats_.discarded_bytes += header_size;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header_size));
      continue;
    }
    if (buffer_.size() < header_size + body) {
      return;
    }
    ++stats_.frames;
    emit(buffer_, header_size + body, stamp, on_frame);
  }
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::FixedLength, simport::Framer)
PLUGINLIB_EXPORT_CLASS(simport::plugins::Delimiter, simport::Framer)
PLUGINLIB_EXPORT_CLASS(simport::plugins::SyncLength, simport::Framer)
PLUGINLIB_EXPORT_CLASS(simport::plugins::Http, simport::Framer)
