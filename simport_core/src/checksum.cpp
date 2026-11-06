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

#include "simport_core/checksum.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace simport
{

namespace
{

struct CrcPreset
{
  std::string_view name;
  CrcParams params;
};

constexpr std::array<CrcPreset, 11> kCrcPresets{{
  {"crc8", {8, 0x07, 0x00, false, false, 0x00}},
  {"crc8_maxim", {8, 0x31, 0x00, true, true, 0x00}},
  {"crc16_ccitt_false", {16, 0x1021, 0xFFFF, false, false, 0x0000}},
  {"crc16_xmodem", {16, 0x1021, 0x0000, false, false, 0x0000}},
  {"crc16_kermit", {16, 0x1021, 0x0000, true, true, 0x0000}},
  {"crc16_modbus", {16, 0x8005, 0xFFFF, true, true, 0x0000}},
  {"crc16_arc", {16, 0x8005, 0x0000, true, true, 0x0000}},
  {"crc16_buypass", {16, 0x8005, 0x0000, false, false, 0x0000}},  // Dynamixel Protocol 2.0
  {"crc32", {32, 0x04C11DB7, 0xFFFFFFFF, true, true, 0xFFFFFFFF}},
  {"crc32c", {32, 0x1EDC6F41, 0xFFFFFFFF, true, true, 0xFFFFFFFF}},
  {"crc64_xz", {64, 0x42F0E1EBA9EA3693, ~std::uint64_t{0}, true, true, ~std::uint64_t{0}}},
}};

std::uint64_t reflect(std::uint64_t v, int bits)
{
  std::uint64_t r = 0;
  for (int i = 0; i < bits; ++i) {
    r = (r << 1) | ((v >> i) & 1);
  }
  return r;
}

std::uint64_t mask_for(int width)
{
  return width >= 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << width) - 1;
}

}  // namespace

Checksum::Checksum(Kind kind, std::string name, std::size_t width_bytes)
: kind_(kind), name_(std::move(name)), width_bytes_(width_bytes) {}

std::vector<std::string_view> Checksum::preset_names()
{
  std::vector<std::string_view> names{"xor8", "nmea", "sum8", "sum16", "lrc8", "fletcher8"};
  for (const auto & p : kCrcPresets) {
    names.push_back(p.name);
  }
  return names;
}

Checksum Checksum::preset(std::string_view name)
{
  if (name == "xor8" || name == "nmea") {
    return Checksum(Kind::kXor8, std::string(name), 1);
  }
  if (name == "sum8") {
    return Checksum(Kind::kSum8, "sum8", 1);
  }
  if (name == "sum16") {
    return Checksum(Kind::kSum16, "sum16", 2);
  }
  if (name == "lrc8") {
    return Checksum(Kind::kLrc8, "lrc8", 1);
  }
  if (name == "fletcher8") {
    return Checksum(Kind::kFletcher8, "fletcher8", 2);
  }
  for (const auto & p : kCrcPresets) {
    if (p.name == name) {
      Checksum c = crc(p.params);
      c.name_ = std::string(name);
      return c;
    }
  }
  std::string names;
  for (const auto n : preset_names()) {
    names += " " + std::string(n);
  }
  throw std::invalid_argument("unknown checksum '" + std::string(name) + "'; available:" +
          names + " crc (custom)");
}

Checksum Checksum::crc(const CrcParams & p)
{
  if (p.width < 8 || p.width > 64 || p.width % 8 != 0) {
    throw std::invalid_argument("CRC width must be a multiple of 8 between 8 and 64");
  }
  Checksum c(Kind::kCrc, "crc", static_cast<std::size_t>(p.width / 8));
  c.crc_ = p;
  c.mask_ = mask_for(p.width);
  const std::uint64_t top = std::uint64_t{1} << (p.width - 1);
  const std::uint64_t rpoly = reflect(p.poly & c.mask_, p.width);
  for (std::uint64_t i = 0; i < 256; ++i) {
    std::uint64_t r = 0;
    if (p.refin) {
      r = i;
      for (int b = 0; b < 8; ++b) {
        r = (r & 1) ? (r >> 1) ^ rpoly : r >> 1;
      }
    } else {
      r = i << (p.width - 8);
      for (int b = 0; b < 8; ++b) {
        r = (r & top) ? ((r << 1) ^ p.poly) : (r << 1);
      }
    }
    c.table_[i] = r & c.mask_;
  }
  return c;
}

Checksum Checksum::parse(const Config & node, std::initializer_list<std::string_view> extra_keys)
{
  if (node.is_scalar()) {
    try {
      return preset(node.scalar());
    } catch (const std::invalid_argument & e) {
      node.error(e.what());
    }
  }
  if (!node.is_map()) {
    node.error("expected a checksum name such as crc16_modbus, or { algo: ..., ... }");
  }
  std::vector<std::string> keys{"algo"};
  for (const auto k : extra_keys) {
    keys.emplace_back(k);
  }
  const std::string algo = node.get<std::string>("algo");
  if (algo != "crc") {
    node.expect_keys(keys);
    try {
      return preset(algo);
    } catch (const std::invalid_argument & e) {
      node["algo"].error(e.what());
    }
  }
  for (const char * k : {"width", "poly", "init", "refin", "refout", "xorout"}) {
    keys.emplace_back(k);
  }
  node.expect_keys(keys);
  CrcParams p;
  p.width = node.get<int>("width");
  p.poly = node.get<std::uint64_t>("poly");
  p.init = node.get<std::uint64_t>("init", 0);
  p.refin = node.get<bool>("refin", false);
  p.refout = node.get<bool>("refout", false);
  p.xorout = node.get<std::uint64_t>("xorout", 0);
  try {
    return crc(p);
  } catch (const std::invalid_argument & e) {
    node["width"].error(e.what());
  }
}

std::uint64_t Checksum::compute(ByteView data) const
{
  switch (kind_) {
    case Kind::kXor8: {
        std::uint8_t x = 0;
        for (const auto b : data) {
          x ^= b;
        }
        return x;
      }
    case Kind::kSum8:
    case Kind::kSum16:
    case Kind::kLrc8: {
        std::uint64_t s = 0;
        for (const auto b : data) {
          s += b;
        }
        if (kind_ == Kind::kSum8) {
          return s & 0xFF;
        }
        if (kind_ == Kind::kSum16) {
          return s & 0xFFFF;
        }
        return (~s + 1) & 0xFF;  // two's complement of the 8-bit sum
      }
    case Kind::kFletcher8: {
        std::uint8_t a = 0;
        std::uint8_t b = 0;
        for (const auto v : data) {
          a = static_cast<std::uint8_t>(a + v);
          b = static_cast<std::uint8_t>(b + a);
        }
        return (std::uint64_t{a} << 8) | b;
      }
    case Kind::kCrc:
      break;
  }
  const int w = crc_.width;
  std::uint64_t r;
  if (crc_.refin) {
    r = reflect(crc_.init & mask_, w);
    for (const auto b : data) {
      r = table_[(r ^ b) & 0xFF] ^ (r >> 8);
    }
    if (!crc_.refout) {
      r = reflect(r, w);
    }
  } else {
    r = crc_.init & mask_;
    for (const auto b : data) {
      r = (table_[((r >> (w - 8)) ^ b) & 0xFF] ^ (r << 8)) & mask_;
    }
    if (crc_.refout) {
      r = reflect(r, w);
    }
  }
  return (r ^ crc_.xorout) & mask_;
}

}  // namespace simport
