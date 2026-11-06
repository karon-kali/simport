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
#include <memory>
#include <string>
#include <vector>

#include "simport_plugins/text_codec.hpp"

namespace simport::plugins
{
namespace
{

TextCodec make_codec(const std::string & yaml)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  TextCodec codec;
  codec.configure(Config::from_string(yaml, "codec.yaml"), Config(), ctx);
  return codec;
}

std::string error_of(const std::string & yaml)
{
  try {
    make_codec(yaml);
  } catch (const ConfigError & e) {
    return e.what();
  }
  ADD_FAILURE() << "expected ConfigError for:\n" << yaml;
  return {};
}

// The classic GGA example sentence, checksum 47.
const char * const kGga =
  "format: \"${$begin}GPGGA,{utc:hhmmss},{lat:nmea_lat.3},{lon:nmea_lon.3},{fix:%d},"
  "{sats:%02d},{hdop:%.1f},{alt:%.1f},M,{geoid:%.1f},M,,{$end}*{$checksum}\"\n"
  "checksum: nmea\n";
const char * const kGgaText =
  "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

Record gga_record(const TextCodec & c)
{
  Record r = Record::make(c.schema());
  r.at("utc") = 12 * 3600.0 + 35 * 60 + 19;
  r.at("lat") = 48.0 + 7.038 / 60.0;
  r.at("lon") = 11.0 + 31.0 / 60.0;
  r.at("fix") = 1.0;
  r.at("sats") = 8.0;
  r.at("hdop") = 0.9;
  r.at("alt") = 545.4;
  r.at("geoid") = 46.9;
  return r;
}

TEST(TextCodec, NmeaGgaGolden)
{
  TextCodec c = make_codec(kGga);
  EXPECT_EQ(c.schema()->names(), (std::vector<std::string>{
        "utc", "lat", "lon", "fix", "sats", "hdop", "alt", "geoid"}));
  EXPECT_EQ(c.schema()->field(3).kind, ValueKind::kInt);
  EXPECT_EQ(c.render(gga_record(c)), kGgaText);
}

TEST(TextCodec, NmeaGgaDecode)
{
  TextCodec c = make_codec(kGga);
  const std::string text = kGgaText;
  Record r = Record::make(c.schema());
  const DecodeResult res = c.decode(Frame{Bytes(text.begin(), text.end()), {}, {}}, r);
  ASSERT_TRUE(res.ok()) << res.detail;
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("utc")), 45319.0);
  EXPECT_NEAR(std::get<double>(r.at("lat")), 48.1173, 1e-9);
  EXPECT_NEAR(std::get<double>(r.at("lon")), 11.516666666, 1e-8);
  EXPECT_EQ(std::get<std::int64_t>(r.at("sats")), 8);
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("alt")), 545.4);

  std::string bad = text;
  bad.back() = '8';
  EXPECT_EQ(c.decode(Frame{Bytes(bad.begin(), bad.end()), {}, {}}, r).code,
    DecodeResult::Code::kBadChecksum);
  const std::string other = "$GPRMC,1,2*00";
  EXPECT_EQ(c.decode(Frame{Bytes(other.begin(), other.end()), {}, {}}, r).code,
    DecodeResult::Code::kNoMatch);
}

TEST(TextCodec, SouthWestAndRounding)
{
  TextCodec c = make_codec("format: \"{lat:nmea_lat},{lon:nmea_lon.2};\"\n");
  Record r = Record::make(c.schema());
  r.at("lat") = -33.8568;          // Sydney
  r.at("lon") = -(151.0 + 59.9999 / 60.0);  // minutes round up to 60 -> carries into degrees
  EXPECT_EQ(c.render(r), "3351.4080,S,15200.00,W;");

  Record back = Record::make(c.schema());
  const std::string text = c.render(r);
  ASSERT_TRUE(c.decode(Frame{Bytes(text.begin(), text.end()), {}, {}}, back).ok());
  EXPECT_NEAR(std::get<double>(back.at("lat")), -33.8568, 1e-6);
}

TEST(TextCodec, EmptyNmeaFieldsDecodeAsAbsent)
{
  TextCodec c = make_codec("format: \"{lat:nmea_lat},{n:%d}\"\n");
  Record r = Record::make(c.schema());
  r.at("lat") = 5.0;
  const std::string text = ",,7";
  ASSERT_TRUE(c.decode(Frame{Bytes(text.begin(), text.end()), {}, {}}, r).ok());
  EXPECT_DOUBLE_EQ(std::get<double>(r.at("lat")), 5.0);  // untouched
  EXPECT_EQ(std::get<std::int64_t>(r.at("n")), 7);
  r.at("lat") = std::nan("");
  EXPECT_EQ(c.render(r), ",,7");
}

TEST(TextCodec, TimeAndDateFormats)
{
  TextCodec c = make_codec("format: \"{t:hhmmss.ss} {d:ddmmyy}\"\n");
  Record r = Record::make(c.schema());
  // 2026-11-04 01:02:03.456 UTC
  const double unix_s = 1793754123.456;
  r.at("t") = unix_s;
  r.at("d") = unix_s;
  EXPECT_EQ(c.render(r), "010203.46 041126");
  Record back = Record::make(c.schema());
  const std::string text = c.render(r);
  ASSERT_TRUE(c.decode(Frame{Bytes(text.begin(), text.end()), {}, {}}, back).ok());
  EXPECT_NEAR(std::get<double>(back.at("t")), 3723.46, 1e-9);
  EXPECT_DOUBLE_EQ(std::get<double>(back.at("d")), 1793750400.0);  // midnight of that day
}

TEST(TextCodec, PrintfKindsAndBraces)
{
  TextCodec c = make_codec(
    "format: \"{{{a:%+05d}}} {b:%04X} {c:%.2e} {s:%s}|{a:%d}\\r\\n\"\n");
  Record r = Record::make(c.schema());
  r.at("a") = -42.4;
  r.at("b") = 255.0;
  r.at("c") = 12345.0;
  r.at("s") = std::string("ok");
  EXPECT_EQ(c.render(r), "{-0042} 00FF 1.23e+04 ok|-42\r\n");
}

TEST(TextCodec, CustomChecksumRange)
{
  // Checksum over everything before it, rendered as decimal.
  TextCodec c = make_codec("format: \"AB{v:%d}{$end}#{$checksum:%u}\"\nchecksum: sum8\n");
  Record r = Record::make(c.schema());
  r.at("v") = 1.0;
  // 'A' + 'B' + '1' = 65 + 66 + 49 = 180
  EXPECT_EQ(c.render(r), "AB1#180");
}

TEST(TextCodec, ConfigErrors)
{
  EXPECT_NE(error_of("format: \"{a:%q}\"\n").find("invalid printf spec '%q'"),
    std::string::npos);
  EXPECT_NE(error_of("format: \"{a\"\n").find("unterminated '{'"), std::string::npos);
  EXPECT_NE(error_of("format: \"a}\"\n").find("unmatched '}'"), std::string::npos);
  EXPECT_NE(error_of("format: \"{1a}\"\n").find("invalid placeholder"), std::string::npos);
  EXPECT_NE(error_of("format: \"{a}{b}\"\n").find("separated by literal text"),
    std::string::npos);
  EXPECT_NE(error_of("format: \"{a:%d},{a:%f}\"\n").find("different kinds"), std::string::npos);
  EXPECT_NE(error_of("format: \"x*{$checksum}\"\n").find("set 'checksum'"), std::string::npos);
  EXPECT_NE(error_of("format: \"x\"\nchecksum: nmea\n").find("no {$checksum}"),
    std::string::npos);
  EXPECT_NE(error_of("format: \"{$checksum:%f}x\"\nchecksum: nmea\n").find("unsigned"),
    std::string::npos);
  EXPECT_NE(error_of("format: \"x{$checksum}{$begin}y\"\nchecksum: nmea\n")
    .find("must come after the checksummed text"), std::string::npos);
  EXPECT_NE(error_of("format: \"{a:nmea_latt}\"\n").find("expected 'nmea_lat.<decimals>'"),
    std::string::npos);
}

}  // namespace
}  // namespace simport::plugins
