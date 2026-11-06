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

#include "simport_plugins/framers.hpp"

namespace simport::plugins
{
namespace
{

template<typename F>
std::vector<Bytes> run(F & framer, const std::string & yaml, const std::vector<Bytes> & chunks)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  framer.configure(Config::from_string(yaml, "f.yaml"), ctx);
  std::vector<Bytes> out;
  for (const Bytes & c : chunks) {
    framer.feed(c, Timestamp{}, [&](Frame && f) {out.push_back(f.data);});
  }
  return out;
}

Bytes text(const std::string & s) {return Bytes(s.begin(), s.end());}

TEST(FixedLength, SplitsAcrossChunks)
{
  FixedLength f;
  const auto frames = run(f, "size: 3", {{1, 2}, {3, 4, 5, 6, 7}});
  EXPECT_EQ(frames, (std::vector<Bytes>{{1, 2, 3}, {4, 5, 6}}));
  EXPECT_EQ(f.stats().frames, 2u);
  f.reset();  // drops the pending 7
  f.feed(Bytes{8, 9, 10}, Timestamp{}, [&](Frame && fr) {EXPECT_EQ(fr.data, (Bytes{8, 9, 10}));});
}

TEST(Delimiter, TextLines)
{
  Delimiter f;
  const auto frames = run(
    f, "delimiter: \"\\r\\n\"", {text("$A*1\r"), text("\n$B*2\r\n$C")});
  EXPECT_EQ(frames, (std::vector<Bytes>{text("$A*1\r\n"), text("$B*2\r\n")}));
}

TEST(Delimiter, ExcludeAndOverflow)
{
  Delimiter f;
  const auto frames = run(
    f, "delimiter: [0x00]\ninclude_delimiter: false\nmax_size: 4",
    {{1, 2, 0}, {9, 9, 9, 9, 9, 9}, {3, 0}});
  EXPECT_EQ(frames, (std::vector<Bytes>{{1, 2}, {3}}));
  EXPECT_EQ(f.stats().discarded_bytes, 6u);
}

TEST(SyncLength, FramesSkipsNoiseAndFalseSyncs)
{
  // [AA 55 len payload... crc]: len counts the payload; 4 bytes outside it.
  SyncLength f;
  const auto frames = run(
    f, "sync: [0xAA, 0x55]\nlength: {offset: 2, type: u8}\nadjust: 4\nmax_size: 32",
      {
        {0x00, 0x13},                  // noise
        {0xAA, 0x55, 0x02, 0x10},      // frame 1, split
        {0x11, 0xEE},
        {0xAA, 0x55, 0xFF, 0x01},      // false sync: 255 + 4 > max_size
        {0xAA},                        // partial sync across chunks
        {0x55, 0x00, 0xEE},            // frame 2, empty payload
    });
  EXPECT_EQ(frames, (std::vector<Bytes>{
        {0xAA, 0x55, 0x02, 0x10, 0x11, 0xEE}, {0xAA, 0x55, 0x00, 0xEE}}));
  EXPECT_EQ(f.stats().frames, 2u);
  EXPECT_GE(f.stats().discarded_bytes, 2u + 4u);
}

TEST(SyncLength, BigEndianU16LengthAndStringSync)
{
  SyncLength f;
  const auto frames = run(
    f, "sync: \"$\"\nlength: {offset: 1, type: u16, endian: big}\nadjust: 3",
    {text("$"), {0x00, 0x02, 'h', 'i'}});
  EXPECT_EQ(frames, (std::vector<Bytes>{{'$', 0x00, 0x02, 'h', 'i'}}));
}

TEST(SyncLength, CasesChooseTheFrameSizeFromAHeaderByte)
{
  // RPLIDAR-style: A5 cmd (cmd < 0x80), or A5 cmd len payload checksum (cmd >= 0x80).
  SyncLength f;
  const auto frames = run(
    f,
    "sync: [0xA5]\n"
    "cases:\n"
    "  - when: {offset: 1, mask: 0x80, equals: 0x00}\n"
    "    size: 2\n"
    "  - length: {offset: 2}\n"
    "    adjust: 4\n",
      {
        {0xA5, 0x25, 0xA5},            // STOP, then a split frame
        {0x50},                        // GET_INFO
        {0xA5, 0x84, 0x04, 0x7C, 0x00},
        {0x00, 0x00, 0x5D},            // GET_CONF with a 4-byte payload
    });
  EXPECT_EQ(frames, (std::vector<Bytes>{
        {0xA5, 0x25}, {0xA5, 0x50}, {0xA5, 0x84, 0x04, 0x7C, 0x00, 0x00, 0x00, 0x5D}}));
}

TEST(SyncLength, CasesWithoutDefaultSkipUnmatchedHeaders)
{
  // BNO055-style: AA 01 reg len (read), AA 00 reg len data... (write); anything else is noise.
  SyncLength f;
  const auto frames = run(
    f,
    "sync: [0xAA]\n"
    "cases:\n"
    "  - when: {offset: 1, equals: 0x01}\n"
    "    size: 4\n"
    "  - when: {offset: 1, equals: 0x00}\n"
    "    length: {offset: 3}\n"
    "    adjust: 4\n",
    {{0xAA, 0x07, 0xAA, 0x01, 0x08, 0x2D, 0xAA, 0x00, 0x3D, 0x01, 0x0C}});
  EXPECT_EQ(frames, (std::vector<Bytes>{
        {0xAA, 0x01, 0x08, 0x2D}, {0xAA, 0x00, 0x3D, 0x01, 0x0C}}));
  EXPECT_EQ(f.stats().discarded_bytes, 2u);
}

TEST(SyncLength, DerivedFromThePacketEnvelope)
{
  // Dynamixel Protocol 2.0: FF FF FD 00, id, length (u16 LE, counts instruction..crc).
  SyncLength f;
  const char * const envelope =
    "start: [0xFF, 0xFF, 0xFD, 0x00]\n"
    "fields:\n"
    "  - id: u8\n"
    "  - length: { u16, length_of: [body, crc] }\n"
    "  - body\n"
    "  - crc: { u16, checksum: crc16_buypass }\n";
  const Bytes ping{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};
  const Bytes read{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D,
    0x15};
  Bytes stream{0x00, 0xFF};  // noise first
  stream.insert(stream.end(), ping.begin(), ping.end());
  stream.insert(stream.end(), read.begin(), read.end());
  const auto frames = run(f, envelope, {Bytes(stream.begin(), stream.begin() + 9),
        Bytes(stream.begin() + 9, stream.end())});
  EXPECT_EQ(frames, (std::vector<Bytes>{ping, read}));

  // A length that counts only the body, with a one-byte trailer after it.
  SyncLength g;
  const auto small = run(g, "start: [0x5A]\nfields: [len: { u8, length_of: body }, body,"
      " crc: { u8, checksum: xor8 }]\n", {{0x5A, 0x02, 0x10, 0x01, 0x13, 0x5A, 0x00, 0x00}});
  EXPECT_EQ(small, (std::vector<Bytes>{{0x5A, 0x02, 0x10, 0x01, 0x13}, {0x5A, 0x00, 0x00}}));
}

TEST(Framers, ConfigErrors)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  FixedLength fl;
  EXPECT_THROW(fl.configure(Config::from_string("size: 0"), ctx), ConfigError);
  Delimiter d;
  EXPECT_THROW(d.configure(Config::from_string("delimiter: \"\""), ctx), ConfigError);
  SyncLength s;
  EXPECT_THROW(s.configure(Config::from_string("sync: [1]\nlength: {offset: 1, type: u24}"), ctx),
    ConfigError);
  EXPECT_THROW(s.configure(Config::from_string("sync: [1]"), ctx), ConfigError);
  EXPECT_THROW(
    s.configure(Config::from_string("sync: [1]\ncases: [{size: 2, length: {offset: 1}}]"), ctx),
    ConfigError);
  EXPECT_THROW(
    s.configure(Config::from_string("sync: [1]\nsize: 2\ncases: [{size: 2}]"), ctx),
    ConfigError);
  EXPECT_THROW(
    s.configure(
      Config::from_string("sync: [1]\ncases: [{when: {offset: 1, mask: 0x0F, equals: 0x10}, "
      "size: 2}]"), ctx),
    ConfigError);
}

}  // namespace
}  // namespace simport::plugins
