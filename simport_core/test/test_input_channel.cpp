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
#include <utility>
#include <vector>

#include "simport_core/input_channel.hpp"

namespace simport
{
namespace
{

/// Frames of exactly two bytes: [cmd, arg].
class TwoFieldCodec : public Codec
{
public:
  TwoFieldCodec()
  : schema_(std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"cmd", ValueKind::kUint}, {"arg", ValueKind::kUint}})) {}
  void configure(const Config &, const Config &, Context &) override {}
  SchemaPtr schema() const override {return schema_;}
  bool can_decode() const override {return true;}
  void encode(const Record & r, std::vector<Frame> & out) override
  {
    out.push_back(Frame{{saturate_cast<std::uint8_t>(r[0]), saturate_cast<std::uint8_t>(r[1])},
        {}, {}});
  }
  DecodeResult decode(const Frame & f, Record & out) override
  {
    if (f.data.size() != 2) {
      return DecodeResult::failure(DecodeResult::Code::kBadLength);
    }
    out[0] = std::uint64_t{f.data[0]};
    out[1] = std::uint64_t{f.data[1]};
    return DecodeResult::success();
  }

private:
  SchemaPtr schema_;
};

/// Splits on every second byte.
class PairFramer : public Framer
{
public:
  void configure(const Config &, Context &) override {}
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override
  {
    for (const auto b : data) {
      pending_.push_back(b);
      if (pending_.size() == 2) {
        on_frame(Frame{pending_, {}, stamp});
        pending_.clear();
      }
    }
  }
  void reset() override {pending_.clear();}
  FramerStats stats() const override {return {};}

private:
  Bytes pending_;
};

struct Device
{
  Context ctx{"dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>()};
  std::vector<std::pair<RecordPtr, Frame>> handled;

  InputChannel make_input(const std::string & match_yaml)
  {
    auto codec = std::make_shared<TwoFieldCodec>();
    auto match = parse_conditions(Config::from_string(match_yaml), *codec->schema());
    return InputChannel(
      "in", ctx, codec, std::move(match),
      [this](const RecordPtr & r, const Frame & f) {handled.emplace_back(r, f);});
  }
};

TEST(InputChannel, HandsMatchingRecordsWithTheirFrameToTheHandler)
{
  Device d;
  InputChannel ping = d.make_input("{cmd: 0x01}");
  FrameMeta requester;
  requester.endpoint = Endpoint{"10.0.0.9", 1234};
  EXPECT_TRUE(ping.handle(Frame{{0x01, 0x42}, requester, {}}));
  ASSERT_EQ(d.handled.size(), 1u);
  EXPECT_EQ(std::get<std::uint64_t>(d.handled[0].first->at("arg")), 0x42u);
  // The frame travels along, so a reply can go back to whoever asked.
  EXPECT_EQ(d.handled[0].second.meta.endpoint, requester.endpoint);

  EXPECT_FALSE(ping.handle(Frame{{0x02, 0}, {}, {}}));     // cmd mismatch: not claimed
  EXPECT_FALSE(ping.handle(Frame{{0x01, 0, 0}, {}, {}}));  // does not decode
  EXPECT_EQ(d.handled.size(), 1u);
  EXPECT_EQ(ping.stats().frames, 1u);
}

TEST(InboundRouter, FramesDispatchesAndCountsUnmatched)
{
  Device d;
  InputChannel a = d.make_input("{cmd: 1}");
  InputChannel b = d.make_input("{cmd: 2}");
  InboundRouter router(d.ctx, std::make_shared<PairFramer>(), {&a, &b});
  int tapped = 0;
  router.set_frame_tap([&](const Frame &) {++tapped;});

  const Bytes stream{1, 0, 2, 0, 9, 9, 2};
  router.on_receive(stream, {}, {});  // inactive: ignored
  EXPECT_EQ(router.stats().frames, 0u);

  router.set_active(true);
  router.on_receive(ByteView(stream).subspan(0, 3), {}, {});
  router.on_receive(ByteView(stream).subspan(3), {}, {});
  EXPECT_EQ(a.stats().frames, 1u);
  EXPECT_EQ(b.stats().frames, 1u);
  EXPECT_EQ(router.stats().frames, 3u);
  EXPECT_EQ(router.stats().unmatched, 1u);  // 9 9
  EXPECT_EQ(tapped, 3);

  // Re-activation drops a partial frame (the trailing "2").
  router.set_active(false);
  router.set_active(true);
  router.on_receive(Bytes{1, 0}, {}, {});
  EXPECT_EQ(a.stats().frames, 2u);
}

TEST(InboundRouter, FramesEachPeerSeparately)
{
  Device d;
  InputChannel a = d.make_input("{cmd: 1}");
  int created = 0;
  InboundRouter router(
    d.ctx, FramerFactory([&] {++created; return std::make_shared<PairFramer>();}), {&a});
  router.set_active(true);
  FrameMeta p1, p2;
  p1.endpoint = Endpoint{"127.0.0.1", 1000};
  p2.endpoint = Endpoint{"127.0.0.1", 2000};
  // Interleaved halves of a frame from two clients are not mixed up.
  router.on_receive(Bytes{1}, p1, {});
  router.on_receive(Bytes{1}, p2, {});
  router.on_receive(Bytes{0}, p1, {});
  router.on_receive(Bytes{0}, p2, {});
  EXPECT_EQ(a.stats().frames, 2u);
  EXPECT_EQ(created, 2);
  // A disconnect discards the peer's partial frame.
  router.on_receive(Bytes{1}, p1, {});
  FrameMeta gone = p1;
  gone.attributes[kDisconnectedAttribute] = "1";
  router.on_receive({}, gone, {});
  router.on_receive(Bytes{1, 0}, p1, {});
  EXPECT_EQ(a.stats().frames, 3u);
  EXPECT_EQ(created, 3);
}

TEST(InboundRouter, MessageTransportsNeedNoFramer)
{
  Device d;
  InputChannel a = d.make_input("{}");
  InboundRouter router(d.ctx, FramerFactory(), {&a});
  router.set_active(true);
  router.on_receive(Bytes{1, 2}, {}, {});
  router.on_receive(Bytes{1, 2, 3}, {}, {});  // wrong length for the codec
  EXPECT_EQ(a.stats().frames, 1u);
  EXPECT_EQ(router.stats().unmatched, 1u);
}

TEST(Conditions, ParseAndCompare)
{
  const Schema s({{"u", ValueKind::kUint}, {"i", ValueKind::kInt}, {"s", ValueKind::kString}});
  const auto c = parse_conditions(Config::from_string("{u: 0x10, i: -1, s: hi}"), s);
  ASSERT_EQ(c.size(), 3u);
  EXPECT_TRUE(values_equal(Value{std::uint64_t{16}}, c[0].expected[0]));
  EXPECT_TRUE(values_equal(Value{16.0}, c[0].expected[0]));
  EXPECT_FALSE(values_equal(Value{std::int64_t{-1}}, Value{std::uint64_t{~0ULL}}));
  EXPECT_TRUE(values_equal(Value{std::string("hi")}, c[2].expected[0]));
  EXPECT_THROW(parse_conditions(Config::from_string("{x: 1}"), s), ConfigError);
  EXPECT_THROW(parse_conditions(Config::from_string("{u: []}"), s), ConfigError);
  // A list matches any of its values.
  const auto any = parse_conditions(Config::from_string("{u: [1, 0x101]}"), s);
  EXPECT_TRUE(any[0].matches(Value{std::uint64_t{0x101}}));
  EXPECT_TRUE(any[0].matches(Value{std::uint64_t{1}}));
  EXPECT_FALSE(any[0].matches(Value{std::uint64_t{2}}));
}

}  // namespace
}  // namespace simport
