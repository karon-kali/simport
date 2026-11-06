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

#include "simport_plugins/binary_codec.hpp"

namespace simport::plugins
{
namespace
{

Context make_context()
{
  return Context("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
}

/// A codec for `message` (YAML of a message: fields, frames, endian) inside `envelope`.
BinaryCodec make_codec(const std::string & message, const std::string & envelope = "")
{
  Context ctx = make_context();
  BinaryCodec codec;
  const Config env = envelope.empty() ? Config() : Config::from_string(envelope, "packet.yaml");
  codec.configure(Config::from_string(message, "message.yaml"), env, ctx);
  return codec;
}

std::string error_of(const std::string & message, const std::string & envelope = "")
{
  try {
    make_codec(message, envelope);
  } catch (const ConfigError & e) {
    return e.what();
  }
  ADD_FAILURE() << "expected ConfigError for:\n" << message;
  return {};
}

std::vector<Bytes> encode_all(BinaryCodec & c, const Record & r)
{
  std::vector<Frame> frames;
  c.encode(r, frames);
  std::vector<Bytes> out;
  for (const Frame & f : frames) {
    out.push_back(f.data);
  }
  return out;
}

Bytes encode_one(BinaryCodec & c, const Record & r)
{
  return encode_all(c, r).at(0);
}

const char * const kImu =
  "endian: little\n"
  "fields:\n"
  "  - sync: { u16: 0xAA55, endian: big }\n"
  "  - gx: i16\n"
  "  - pad: 1\n"
  "  - t: { u32, endian: big }\n"
  "  - ax: f32\n"
  "  - ok: bool\n";

// -- fields --------------------------------------------------------------------------------------

TEST(BinaryCodec, SchemaAndSize)
{
  const BinaryCodec c = make_codec(kImu);
  EXPECT_EQ(c.frame_size(), 2u + 2 + 1 + 4 + 4 + 1);
  const Schema & s = *c.schema();
  EXPECT_EQ(s.names(), (std::vector<std::string>{"sync", "gx", "t", "ax", "ok"}));
  EXPECT_EQ(s.field(0).role, FieldRole::kComputed);
  EXPECT_EQ(s.field(1).kind, ValueKind::kInt);
  EXPECT_EQ(s.field(2).kind, ValueKind::kUint);
  EXPECT_EQ(s.field(3).kind, ValueKind::kFloat);
  EXPECT_EQ(s.field(4).kind, ValueKind::kBool);
}

TEST(BinaryCodec, GoldenBytes)
{
  BinaryCodec c = make_codec(kImu);
  Record r = Record::make(c.schema());
  r.at("gx") = -2.0;                       // -> 0xFFFE little endian
  r.at("t") = std::uint64_t{0x01020304};   // big endian
  r.at("ax") = 1.0;                        // 0x3F800000 little endian
  r.at("ok") = true;
  EXPECT_EQ(encode_one(c, r),
    (Bytes{0xAA, 0x55, 0xFE, 0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x80, 0x3F, 0x01}));
}

TEST(BinaryCodec, FieldForms)
{
  // `name: type`, `{ type }`, `{ type: const }`, `{ type: u8 }`, `{ u8, value: ... }`.
  BinaryCodec c = make_codec(
    "fields: [a: u8, b: { u8 }, c: { u8: 7 }, d: { type: u8 }, e: { u8, value: 9 }]\n");
  EXPECT_EQ(c.schema()->field(2).role, FieldRole::kComputed);
  Record r = Record::make(c.schema());
  r.at("a") = 1.0;
  r.at("b") = 2.0;
  r.at("d") = 4.0;
  EXPECT_EQ(encode_one(c, r), (Bytes{1, 2, 7, 4, 9}));
}

TEST(BinaryCodec, SaturatesOutOfRangeValues)
{
  BinaryCodec c = make_codec("fields: [a: u8, b: i8]\n");
  Record r = Record::make(c.schema());
  r[0] = 300.0;
  r[1] = -1000.0;
  EXPECT_EQ(encode_one(c, r), (Bytes{0xFF, 0x80}));
}

TEST(BinaryCodec, ModbusRtuGolden)
{
  // Read Holding Registers request: slave 1, function 3, start 0, count 10 -> CRC C5 CD.
  BinaryCodec c = make_codec(
    "endian: big\n"
    "fields:\n"
    "  - slave: { u8: 1 }\n"
    "  - func: { u8: 3 }\n"
    "  - start: { u16: 0 }\n"
    "  - count: u16\n"
    "  - crc: { u16, endian: little, checksum: crc16_modbus }\n");
  Record r = Record::make(c.schema());
  r.at("count") = 10.0;
  const Bytes frame = encode_one(c, r);
  EXPECT_EQ(frame, (Bytes{0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD}));

  Record back = Record::make(c.schema());
  EXPECT_TRUE(c.decode(Frame{frame, {}, {}}, back).ok());
  EXPECT_EQ(std::get<std::uint64_t>(back.at("count")), 10u);
  Bytes corrupt = frame;
  corrupt[5] = 0x0B;
  const DecodeResult bad = c.decode(Frame{corrupt, {}, {}}, back);
  EXPECT_EQ(bad.code, DecodeResult::Code::kBadChecksum);
  EXPECT_NE(bad.detail.find("checksum 'crc' mismatch"), std::string::npos);
}

TEST(BinaryCodec, UbxFletcherGolden)
{
  // UBX-CFG-RATE poll: B5 62 06 08 00 00, checksum 0E 30 over class..payload.
  BinaryCodec c = make_codec(
    "fields:\n"
    "  - sync: { bytes: [0xB5, 0x62] }\n"
    "  - cls: { u8: 0x06 }\n"
    "  - id: { u8: 0x08 }\n"
    "  - len: { u16: 0 }\n"
    "  - ck: { u16, endian: big, checksum: fletcher8, over: cls }\n");
  EXPECT_EQ(encode_one(c, Record::make(c.schema())),
    (Bytes{0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30}));
}

TEST(BinaryCodec, CountersAdvanceWrapAndReset)
{
  BinaryCodec c = make_codec(
    "fields:\n"
    "  - seq: { u8, counter: { start: 254 } }\n"
    "  - odd: { u16, counter: { start: 1, step: 2 }, endian: big }\n");
  const Record r = Record::make(c.schema());
  EXPECT_EQ(encode_one(c, r), (Bytes{254, 0, 1}));
  EXPECT_EQ(encode_one(c, r), (Bytes{255, 0, 3}));
  EXPECT_EQ(encode_one(c, r), (Bytes{0, 0, 5}));
  c.reset();
  EXPECT_EQ(encode_one(c, r), (Bytes{254, 0, 1}));
}

TEST(BinaryCodec, ChecksumRanges)
{
  // A header checksum over one field, and a trailer covering everything before it.
  BinaryCodec c = make_codec(
    "fields:\n"
    "  - seq: { u8, counter: true }\n"
    "  - hcs: { u8, checksum: xor8, over: [seq, seq] }\n"
    "  - v: u8\n"
    "  - tcs: { u8, checksum: sum8 }\n");
  Record r = Record::make(c.schema());
  r.at("v") = 7.0;
  encode_one(c, r);  // seq 0
  // seq=1, hcs=xor(1)=1, v=7, tcs=sum(1,1,7)=9
  EXPECT_EQ(encode_one(c, r), (Bytes{1, 1, 7, 9}));
}

TEST(BinaryCodec, WrapOverflow)
{
  BinaryCodec c = make_codec(
    "fields:\n"
    "  - a: { u8, overflow: wrap }\n"
    "  - b: { i8, overflow: wrap }\n"
    "  - t: { u32, overflow: wrap, endian: big }\n");
  Record r = Record::make(c.schema());
  r[0] = std::uint64_t{300};      // 300 mod 256 = 44
  r[1] = -1.0;                    // 0xFF
  r[2] = std::int64_t{1'793'730'092'886};  // unix ms: low 32 bits
  const std::uint32_t low = static_cast<std::uint32_t>(1'793'730'092'886ULL);
  EXPECT_EQ(encode_one(c, r), (Bytes{44, 0xFF,
      static_cast<std::uint8_t>(low >> 24), static_cast<std::uint8_t>(low >> 16),
      static_cast<std::uint8_t>(low >> 8), static_cast<std::uint8_t>(low)}));
}

TEST(BinaryCodec, AllTypesRoundTrip)
{
  BinaryCodec c = make_codec(
    "endian: big\n"
    "fields: [u8: u8, u16: u16, u32: u32, u64: u64, i8: i8, i16: i16, i32: i32, i64: i64,"
    " f32: f32, f64: f64, b: bool]\n");
  Record in = Record::make(c.schema());
  in.at("u8") = std::uint64_t{200};
  in.at("u16") = std::uint64_t{60000};
  in.at("u32") = std::uint64_t{4000000000};
  in.at("u64") = std::uint64_t{0xFEDCBA9876543210};
  in.at("i8") = std::int64_t{-100};
  in.at("i16") = std::int64_t{-30000};
  in.at("i32") = std::int64_t{-2000000000};
  in.at("i64") = std::int64_t{-0x123456789ABCDEF};
  in.at("f32") = 0.5;
  in.at("f64") = -1.25e-300;
  in.at("b") = true;
  const Bytes frame = encode_one(c, in);
  ASSERT_EQ(frame.size(), c.frame_size());
  Record out = Record::make(c.schema());
  ASSERT_TRUE(c.decode(Frame{frame, {}, {}}, out).ok());
  for (std::size_t i = 0; i < in.values.size(); ++i) {
    EXPECT_EQ(to_display_string(out[i]), to_display_string(in[i])) << c.schema()->field(i).name;
  }
}

TEST(BinaryCodec, DecodeChecksLengthAndConstants)
{
  BinaryCodec c = make_codec(kImu);
  Record r = Record::make(c.schema());
  Frame f{Bytes(c.frame_size(), 0), {}, {}};
  EXPECT_EQ(c.decode(f, r).code, DecodeResult::Code::kNoMatch);  // sync is 0x0000
  f.data[0] = 0xAA;
  f.data[1] = 0x55;
  EXPECT_TRUE(c.decode(f, r).ok());
  EXPECT_EQ(std::get<std::uint64_t>(r.at("sync")), 0xAA55u);
  f.data.pop_back();
  const DecodeResult bad = c.decode(f, r);
  EXPECT_EQ(bad.code, DecodeResult::Code::kBadLength);
  EXPECT_EQ(bad.detail, "expected 14 bytes, got 13");
}

// -- the envelope ----------------------------------------------------------------------------

const char * const kDynamixel =
  "start: [0xFF, 0xFF, 0xFD, 0x00]\n"
  "fields:\n"
  "  - id: u8\n"
  "  - length: { u16, length_of: [body, crc] }\n"
  "  - body\n"
  "  - crc: { u16, checksum: crc16_buypass }\n"
  "stuffing: { after: [0xFF, 0xFF, 0xFD], insert: 0xFD }\n";

TEST(BinaryCodec, DynamixelPacketsMatchTheProtocolDocumentation)
{
  // ROBOTIS e-manual, Protocol 2.0: ping ID 1; read Present Position (132, 4 bytes) of ID 1.
  BinaryCodec instruction = make_codec(
    "fields: [instruction: u8, params: bytes]\n", kDynamixel);
  Record r = Record::make(instruction.schema());
  r.at("id") = std::uint64_t{1};
  r.at("instruction") = std::uint64_t{0x01};
  EXPECT_EQ(encode_one(instruction, r),
    (Bytes{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E}));
  r.at("instruction") = std::uint64_t{0x02};
  r.at("params") = Bytes{0x84, 0x00, 0x04, 0x00};
  const Bytes read = encode_one(instruction, r);
  EXPECT_EQ(read,
    (Bytes{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15}));

  // ...and decoding it gives the fields back.
  Record back = Record::make(instruction.schema());
  ASSERT_TRUE(instruction.decode(Frame{read, {}, {}}, back).ok());
  EXPECT_EQ(std::get<std::uint64_t>(back.at("id")), 1u);
  EXPECT_EQ(std::get<std::uint64_t>(back.at("instruction")), 2u);
  EXPECT_EQ(std::get<Bytes>(back.at("params")), (Bytes{0x84, 0x00, 0x04, 0x00}));
  Bytes bad_length = read;
  bad_length[5] = 0x08;
  bad_length[12] ^= 0;  // the CRC now fails first
  EXPECT_FALSE(instruction.decode(Frame{bad_length, {}, {}}, back).ok());

  // A status packet: XM430-W350 (model 1020, firmware 52) answering a ping as ID 11.
  BinaryCodec status = make_codec(
    "fields: [instruction: { u8: 0x55 }, error: u8, data: bytes]\n", kDynamixel);
  Record s = Record::make(status.schema());
  s.at("id") = std::uint64_t{11};
  s.at("data") = Bytes{0xFC, 0x03, 0x34};
  EXPECT_EQ(encode_one(status, s),
    (Bytes{0xFF, 0xFF, 0xFD, 0x00, 0x0B, 0x07, 0x00, 0x55, 0x00, 0xFC, 0x03, 0x34, 0x7E, 0x63}));
}

TEST(BinaryCodec, ByteStuffing)
{
  BinaryCodec c = make_codec("fields: [instruction: u8, params: bytes]\n", kDynamixel);
  Record r = Record::make(c.schema());
  r.at("id") = std::uint64_t{1};
  r.at("instruction") = std::uint64_t{0x03};
  r.at("params") = Bytes{0xFF, 0xFF, 0xFD, 0x01};
  const Bytes p = encode_one(c, r);
  // FF FF FD in the body gets an extra FD; the length counts the stuffed body.
  EXPECT_EQ(Bytes(p.begin() + 7, p.end() - 2), (Bytes{0x03, 0xFF, 0xFF, 0xFD, 0xFD, 0x01}));
  EXPECT_EQ(p[5], 8);
  Record back = Record::make(c.schema());
  ASSERT_TRUE(c.decode(Frame{p, {}, {}}, back).ok());
  EXPECT_EQ(std::get<Bytes>(back.at("params")), (Bytes{0xFF, 0xFF, 0xFD, 0x01}));

  EXPECT_EQ(BinaryCodec::stuff({1, 0xFF, 0xFF, 0xFD, 2}, {0xFF, 0xFF, 0xFD}, 0xFD),
    (Bytes{1, 0xFF, 0xFF, 0xFD, 0xFD, 2}));
  EXPECT_FALSE(BinaryCodec::unstuff(Bytes{0xFF, 0xFF, 0xFD, 2}, {0xFF, 0xFF, 0xFD}, 0xFD));
}

TEST(BinaryCodec, LengthFieldCountsTheBody)
{
  // A sensor envelope: 5A, length of the body, body, CRC over length and body.
  const char * const envelope =
    "start: [0x5A]\n"
    "fields:\n"
    "  - length: { u8, length_of: body }\n"
    "  - body\n"
    "  - crc: { u8, checksum: xor8, over: [length, body] }\n";
  BinaryCodec c = make_codec("fields: [cmd: u8, arg: u8]\n", envelope);
  Record r = Record::make(c.schema());
  r.at("cmd") = std::uint64_t{0x10};
  r.at("arg") = std::uint64_t{0x01};
  const Bytes p = encode_one(c, r);
  EXPECT_EQ(p, (Bytes{0x5A, 0x02, 0x10, 0x01, 0x02 ^ 0x10 ^ 0x01}));
  Record back = Record::make(c.schema());
  EXPECT_TRUE(c.decode(Frame{p, {}, {}}, back).ok());
  Bytes wrong = p;
  wrong[1] = 0x03;
  wrong[4] = 0x03 ^ 0x10 ^ 0x01;  // valid CRC, wrong length
  EXPECT_EQ(c.decode(Frame{wrong, {}, {}}, back).code, DecodeResult::Code::kBadLength);
  // A different message type does not decode as this one.
  BinaryCodec other = make_codec("fields: [cmd: { u8: 0x20 }, arg: u8]\n", envelope);
  EXPECT_EQ(other.decode(Frame{p, {}, {}}, back).code, DecodeResult::Code::kNoMatch);
}

TEST(BinaryCodec, RepeatRestAndMeta)
{
  // A sync read: address, length, then any number of ids; on CAN, the id comes from the frame.
  BinaryCodec c = make_codec(
    "fields: [can_id: meta, address: u16, count: u16, ids: { u8, repeat: rest }]\n");
  Record r = Record::make(c.schema());
  r.at("can_id") = std::uint64_t{0x11};
  r.at("address") = std::uint64_t{634};
  r.at("count") = std::uint64_t{8};
  r.at("ids") = std::vector<double>{11, 12, 13};
  std::vector<Frame> frames;
  c.encode(r, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].data, (Bytes{0x7A, 0x02, 0x08, 0x00, 11, 12, 13}));
  EXPECT_EQ(frames[0].meta.can_id, 0x11u);

  Frame in{Bytes{0x7A, 0x02, 0x08, 0x00, 21, 22}, {}, {}};
  in.meta.can_id = 0x7FF;
  Record back = Record::make(c.schema());
  ASSERT_TRUE(c.decode(in, back).ok());
  EXPECT_EQ(std::get<std::uint64_t>(back.at("can_id")), 0x7FFu);
  EXPECT_EQ(std::get<std::vector<double>>(back.at("ids")), (std::vector<double>{21, 22}));
}

TEST(BinaryCodec, LengthInABitField)
{
  // RPLIDAR answer descriptor: A5 5A, then size (30 bits) | mode (2 bits) as u32, type, data.
  BinaryCodec c = make_codec(
    "fields:\n"
    "  - sync: { bytes: [0xA5, 0x5A] }\n"
    "  - size_mode: { u32, bits: [size: { width: 30, length_of: data }, mode: 2] }\n"
    "  - type: u8\n"
    "  - data: bytes\n");
  Record r = Record::make(c.schema());
  r.at("mode") = std::uint64_t{1};
  r.at("type") = std::uint64_t{0x81};
  EXPECT_EQ(encode_one(c, r), (Bytes{0xA5, 0x5A, 0x00, 0x00, 0x00, 0x40, 0x81}));
  r.at("mode") = std::uint64_t{0};
  r.at("type") = std::uint64_t{0x06};
  r.at("data") = Bytes{0, 0, 0};
  const Bytes health = encode_one(c, r);
  EXPECT_EQ(health, (Bytes{0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0, 0, 0}));
  Record back = Record::make(c.schema());
  ASSERT_TRUE(c.decode(Frame{health, {}, {}}, back).ok());
  EXPECT_EQ(std::get<std::uint64_t>(back.at("size")), 3u);
  Bytes wrong = health;
  wrong[2] = 4;
  EXPECT_EQ(c.decode(Frame{wrong, {}, {}}, back).code, DecodeResult::Code::kBadLength);
}

// -- bits, groups, several packets per record --------------------------------------------------

TEST(BinaryCodec, OnePacketPerElementWithBitFields)
{
  // RPLIDAR-style sample: [quality:6 | !start:1 | start:1] [angle_q6:15 | check:1] [dist_q2].
  BinaryCodec c = make_codec(
    "frames: auto\n"
    "fields:\n"
    "  - group:\n"
    "      count: 1\n"
    "      fields:\n"
    "        - flags: { u8, bits: [start: { width: 1, first_element: [1, 0] },"
    " not_start: { width: 1, first_element: [0, 1] }, quality: 6] }\n"
    "        - angle: { u16, bits: [check: { width: 1, value: 1 },"
    " angle_q6: { width: 15, element_index: { scale: 64 } }] }\n"
    "        - dist_q2: u16\n");
  EXPECT_EQ(c.frame_size(), 5u);
  EXPECT_FALSE(c.can_decode());
  EXPECT_EQ(c.schema()->names(FieldRole::kInput),
    (std::vector<std::string>{"quality", "dist_q2"}));
  Record r = Record::make(c.schema());
  r.at("quality") = 47.0;  // a single number fills every element
  r.at("dist_q2") = std::vector<double>{4000, 0, 8000};
  EXPECT_EQ(encode_all(c, r), (std::vector<Bytes>{
        {0xBD, 0x01, 0x00, 0xA0, 0x0F},
        {0xBE, 0x81, 0x00, 0x00, 0x00},
        {0xBE, 0x01, 0x01, 0x40, 0x1F}}));
  r.at("dist_q2") = std::vector<double>{};
  EXPECT_TRUE(encode_all(c, r).empty());
}

TEST(BinaryCodec, ArraysSpanFixedPacketsWithIndexAndChecksum)
{
  // LDS-style packet: FA, A0 + packet number, `count` readings, checksum over the packet.
  BinaryCodec c = make_codec(
    "frames: 2\n"
    "fields:\n"
    "  - sync: { u8: 0xFA }\n"
    "  - index: { u8, frame_index: { offset: 0xA0 } }\n"
    "  - group: { count: 2, fields: [range: u16] }\n"
    "  - cs: { u8, checksum: sum8 }\n");
  EXPECT_EQ(c.frame_size(), 7u);
  Record r = Record::make(c.schema());
  r.at("range") = std::vector<double>{1, 2, 3, 4, 5};  // the fifth element does not fit
  EXPECT_EQ(encode_all(c, r), (std::vector<Bytes>{
        {0xFA, 0xA0, 0x01, 0x00, 0x02, 0x00, 0x9D},
        {0xFA, 0xA1, 0x03, 0x00, 0x04, 0x00, 0xA2}}));
}

TEST(BinaryCodec, NestedGroupsCarryARangeImage)
{
  // Ouster-style: columns (header + beams), 2 columns per packet, 3 beams per column.
  BinaryCodec c = make_codec(
    "frames: auto\n"
    "fields:\n"
    "  - frame_id: { u16, counter: { per: record } }\n"
    "  - group:\n"
    "      count: 2\n"
    "      fields:\n"
    "        - measurement_id: { u16, element_index: true }\n"
    "        - group: { count: 3, fields: [range: u8] }\n"
    "  - crc: { u8, checksum: sum8 }\n");
  Record r = Record::make(c.schema());
  r.at("range") = std::vector<double>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  EXPECT_EQ(encode_all(c, r), (std::vector<Bytes>{
        {0, 0, 0, 0, 1, 2, 3, 1, 0, 4, 5, 6, 22},
        {0, 0, 2, 0, 7, 8, 9, 3, 0, 10, 11, 12, 62}}));
  EXPECT_EQ(encode_all(c, r)[1][0], 1);  // the next record

  BinaryCodec d = make_codec(
    "fields: [group: { count: 2, fields: [col: u8, group: { count: 2, fields: [px: u8] }] }]\n");
  Record out = Record::make(d.schema());
  ASSERT_TRUE(d.decode(Frame{{1, 10, 11, 2, 20, 21}, {}, {}}, out).ok());
  EXPECT_EQ(std::get<std::vector<double>>(out.at("col")), (std::vector<double>{1, 2}));
  EXPECT_EQ(std::get<std::vector<double>>(out.at("px")), (std::vector<double>{10, 11, 20, 21}));
}

TEST(BinaryCodec, DecodesBits)
{
  BinaryCodec c =
    make_codec("fields: [head: { u8, bits: [op: 4, magic: { width: 4, value: 0xA }] }]\n");
  Record r = Record::make(c.schema());
  ASSERT_TRUE(c.decode(Frame{{0xA5}, {}, {}}, r).ok());
  EXPECT_EQ(std::get<std::uint64_t>(r.at("op")), 5u);
  EXPECT_EQ(c.decode(Frame{{0xB5}, {}, {}}, r).code, DecodeResult::Code::kNoMatch);
  r.at("op") = std::uint64_t{99};  // saturates at 15
  EXPECT_EQ(encode_one(c, r), (Bytes{0xAF}));
}

TEST(BinaryCodec, SourcesAreTheFieldDescriptions)
{
  BinaryCodec c = make_codec(
    "fields: [a: u8, b: { i16, from: x.y, scale: 2 }, c: { u8: 1 }, d: { u8, bits: [e: { width: 4,"
    " from: z }, f: 4] }]\n");
  const auto sources = c.sources();
  ASSERT_EQ(sources.size(), 2u);  // b and e; a and f have no description to map from
  EXPECT_EQ(sources[0].first, "b");
  EXPECT_EQ(sources[0].second.get<std::string>("from"), "x.y");
  EXPECT_EQ(sources[1].first, "e");
}

// -- errors ------------------------------------------------------------------------------------

TEST(BinaryCodec, ConfigErrors)
{
  const auto has = [](const std::string & message, const std::string & what,
    const std::string & envelope = "") {
      const std::string e = error_of(message, envelope);
      EXPECT_NE(e.find(what), std::string::npos) << "got: " << e;
    };
  has("{}\n", "fields: required key is missing");
  has("fields: []\n", "at least one field");
  has("fields: [a: u12]\n", "unknown type 'u12'; types: u8,");
  has("fields: [a: u8, a: u8]\n", "duplicate field name 'a'");
  has("fields: [a: { u8: 256 }]\n", "out of range [0, 255]");
  has("endian: middle\nfields: [a: u8]\n", "expected 'little' or 'big'");
  has("fields: [a: { u8, valeu: 1 }]\n", "did you mean 'value'");
  has("fields: [a: { from: x }]\n", "give the field's type");
  has("fields: [a: { u8, i8 }]\n", "a field has one type");
  has("fields: [{ a: u8, b: u8 }]\n", "one field per line");
  has("fields: [a: { u8: 1, counter: true }]\n", "only one of");
  has("fields: [a: { f32, counter: true }]\n", "counters must be integer");
  has("fields: [a: u8, c: { u16, checksum: crc32 }]\n", "is 4 bytes but field 'c' holds only 2");
  has("fields: [a: u8, c: { i16, checksum: sum8 }]\n", "unsigned integer");
  has("fields: [a: u8, c: { u8, checksum: sum8, over: [b, a] }]\n", "no field named 'b'");
  has("fields: [a: u8, c: { u8, checksum: sum8, over: [a, c] }]\n", "must not include the "
    "checksum itself");
  has("fields: [c: { u8, checksum: sum8 }, a: u8]\n", "needs 'over: [first, last]'");
  has("fields: [a: u8, c: { u8, checksum: crc99 }]\n", "unknown checksum 'crc99'");
  has("fields: [a: { f32, overflow: wrap }]\n", "integer types only");
  has("fields: [a: bytes, b: bytes]\n", "only one variable-length field");
  has("fields: [a: { u8, repeat: twice }]\n", "expected 'rest'");
  has("fields: [id: meta]\n", "the only one is 'can_id'");
  has("fields: [b: { u8, bits: [x: 5, y: 4] }]\n", "add up to 9 bits");
  has("fields: [b: { i8, bits: [x: 1] }]\n", "unsigned integer type");
  has("fields: [a: { u8, element_index: true }]\n", "inside a 'group'");
  has("frames: 2\nfields: [a: u8]\n", "need a 'group'");
  has("fields: [a: u8]\n", "needs a 'body' entry", "fields: [id: u8]\n");
  has("fields: [a: u8]\n", "no field named 'nope'", "fields: [l: { u8, length_of: nope }, body]\n");
  has("fields: [a: { u8, length_of: body }]\n", "'body' is only defined");
}

TEST(WireType, Lookup)
{
  ASSERT_TRUE(find_wire_type("u16"));
  EXPECT_EQ(find_wire_type("u16")->size, 2u);
  EXPECT_EQ(find_wire_type("bytes")->kind, ValueKind::kBytes);
  EXPECT_FALSE(find_wire_type("pad"));
  EXPECT_FALSE(find_wire_type("u24"));
}

}  // namespace
}  // namespace simport::plugins
