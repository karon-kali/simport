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

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/checksum.hpp"

namespace simport
{
namespace
{

const Bytes kCheck{'1', '2', '3', '4', '5', '6', '7', '8', '9'};

TEST(Checksum, CatalogueCheckValues)
{
  const std::vector<std::pair<const char *, std::uint64_t>> expected{
    {"xor8", 0x31}, {"nmea", 0x31}, {"sum8", 0xDD}, {"sum16", 0x01DD}, {"lrc8", 0x23},
    {"fletcher8", 0xDD15},
    {"crc8", 0xF4}, {"crc8_maxim", 0xA1},
    {"crc16_ccitt_false", 0x29B1}, {"crc16_xmodem", 0x31C3}, {"crc16_kermit", 0x2189},
    {"crc16_modbus", 0x4B37}, {"crc16_arc", 0xBB3D}, {"crc16_buypass", 0xFEE8},
    {"crc32", 0xCBF43926}, {"crc32c", 0xE3069283}, {"crc64_xz", 0x995DC9BBDF1939FA},
  };
  for (const auto & [name, value] : expected) {
    EXPECT_EQ(Checksum::preset(name).compute(kCheck), value) << name;
  }
  EXPECT_EQ(Checksum::preset("crc32").width_bytes(), 4u);
  EXPECT_EQ(Checksum::preset("fletcher8").width_bytes(), 2u);
}

TEST(Checksum, CustomCrcMatchesPreset)
{
  // CRC-16/MODBUS spelled out; and CRC-64/XZ as a 64-bit sanity check.
  const Checksum modbus = Checksum::crc({16, 0x8005, 0xFFFF, true, true, 0});
  EXPECT_EQ(modbus.compute(kCheck), 0x4B37u);
  const Checksum xz = Checksum::crc(
    {64, 0x42F0E1EBA9EA3693, ~0ULL, true, true, ~0ULL});
  EXPECT_EQ(xz.compute(kCheck), 0x995DC9BBDF1939FAULL);
  // Non-reflected 32-bit: CRC-32/BZIP2.
  const Checksum bzip2 = Checksum::crc({32, 0x04C11DB7, 0xFFFFFFFF, false, false, 0xFFFFFFFF});
  EXPECT_EQ(bzip2.compute(kCheck), 0xFC891918u);
  EXPECT_THROW(Checksum::crc({12, 0x80F, 0, false, false, 0}), std::invalid_argument);
}

TEST(Checksum, EmptyInput)
{
  EXPECT_EQ(Checksum::preset("xor8").compute({}), 0u);
  EXPECT_EQ(Checksum::preset("crc16_ccitt_false").compute({}), 0xFFFFu);
}

TEST(Checksum, ParseForms)
{
  EXPECT_EQ(Checksum::parse(Config::from_string("crc16_modbus")).compute(kCheck), 0x4B37u);
  const Config m = Config::from_string("{algo: sum8, from: a, to: b}");
  EXPECT_EQ(Checksum::parse(m, {"from", "to"}).compute(kCheck), 0xDDu);
  const Config custom = Config::from_string(
    "{algo: crc, width: 16, poly: 0x1021, init: 0xFFFF, from: a}");
  EXPECT_EQ(Checksum::parse(custom, {"from"}).compute(kCheck), 0x29B1u);
}

TEST(Checksum, ParseErrors)
{
  auto error_of = [](const std::string & yaml, std::initializer_list<std::string_view> extra) {
      try {
        Checksum::parse(Config::from_string(yaml, "c.yaml"), extra);
      } catch (const ConfigError & e) {
        return std::string(e.what());
      }
      return std::string();
    };
  EXPECT_NE(error_of("crc17", {}).find("unknown checksum 'crc17'; available: xor8"),
    std::string::npos);
  EXPECT_NE(error_of("{algo: sum8, form: a}", {"from"}).find("did you mean 'from'"),
    std::string::npos);
  EXPECT_NE(error_of("{algo: crc, width: 12, poly: 1}", {}).find("multiple of 8"),
    std::string::npos);
  EXPECT_NE(error_of("{algo: crc, width: 16}", {}).find("poly: required key"),
    std::string::npos);
  EXPECT_NE(error_of("[1]", {}).find("expected a checksum name"), std::string::npos);
}

}  // namespace
}  // namespace simport
