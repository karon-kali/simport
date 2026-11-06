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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/output_channel.hpp"

namespace simport
{
namespace
{

/// Clock frozen at a settable time.
class FakeClock : public Clock
{
public:
  Timestamp now() const override {return t;}
  bool sleep_until(Timestamp, std::stop_token) override {return true;}
  std::string describe() const override {return "fake";}
  Timestamp t{};
};

/// Encodes field 0 and field 1 as one byte each; throws if field 0 is 99.
class TwoByteCodec : public Codec
{
public:
  TwoByteCodec()
  : schema_(std::make_shared<const Schema>(std::vector<FieldSpec>{
        {"a", ValueKind::kUint}, {"t", ValueKind::kInt}})) {}
  void configure(const Config &, const Config &, Context &) override {}
  SchemaPtr schema() const override {return schema_;}
  void encode(const Record & r, std::vector<Frame> & out) override
  {
    if (saturate_cast<int>(r[0]) == 99) {
      throw std::runtime_error("boom");
    }
    out.push_back(Frame{{saturate_cast<std::uint8_t>(r[0]), saturate_cast<std::uint8_t>(r[1])},
        {}, {}});
  }
  void reset() override {++resets;}
  int resets = 0;

private:
  SchemaPtr schema_;
};

class DirectTrigger : public Trigger
{
public:
  void configure(const Config &, Context &) override {}
  void start(EmitCallback emit) override {emit_ = std::move(emit);}
  void stop() override {emit_ = nullptr;}
  void on_record(RecordPtr r) override
  {
    if (emit_) {emit_(r);}
  }

private:
  EmitCallback emit_;
};

/// Adds 1 to the first byte.
class IncFilter : public FrameFilter
{
public:
  void configure(const Config &, Context &) override {}
  void process(Frame && f, const FrameSink & next) override
  {
    f.data[0] = static_cast<std::uint8_t>(f.data[0] + 1);
    next(std::move(f));
  }
};

class RecordingTransport : public Transport
{
public:
  void configure(const Config &, Context &) override {}
  void set_receive_callback(ReceiveCallback) override {}
  void open() override {}
  void close() override {}
  bool is_open() const override {return true;}
  TransportCaps caps() const override {return {};}
  bool write(const Frame & f) override
  {
    if (refuse) {return false;}
    frames.push_back(f);
    return true;
  }
  TransportStats stats() const override {return {};}
  std::string describe() const override {return "rec";}
  std::vector<Frame> frames;
  bool refuse = false;
};

struct Fixture
{
  std::shared_ptr<FakeClock> clock = std::make_shared<FakeClock>();
  Context ctx{"dev", std::make_shared<NullLogger>(), clock};
  std::shared_ptr<TwoByteCodec> codec = std::make_shared<TwoByteCodec>();
  std::shared_ptr<RecordingTransport> transport = std::make_shared<RecordingTransport>();

  /// `sources_yaml`: field descriptions, `{ field: description }`.
  std::unique_ptr<OutputChannel> make(const std::string & sources_yaml, int n_filters)
  {
    MappingSpec mapping =
      MappingSpec::from_sources(Config::from_string(sources_yaml).entries(), codec->schema());
    std::vector<FrameFilterPtr> filters;
    for (int i = 0; i < n_filters; ++i) {
      filters.push_back(std::make_shared<IncFilter>());
    }
    return std::make_unique<OutputChannel>(
      "out", ctx, std::move(mapping), std::make_shared<DirectTrigger>(), codec,
      std::move(filters), transport);
  }

  RecordPtr record(const OutputChannel & ch, std::uint64_t a)
  {
    auto r = std::make_shared<Record>(ch.mapping().make_record());
    (*r)[0] = a;
    return r;
  }
};

TEST(OutputChannel, RunsPipelineThroughFilters)
{
  Fixture f;
  auto ch = f.make("{a: x}", 2);
  ch->push(f.record(*ch, 1));  // ignored: not started
  ch->start();
  EXPECT_EQ(f.codec->resets, 1);
  ch->push(f.record(*ch, 10));
  ASSERT_EQ(f.transport->frames.size(), 1u);
  EXPECT_EQ(f.transport->frames[0].data[0], 12);  // two IncFilters
  ch->stop();
  ch->push(f.record(*ch, 10));
  EXPECT_EQ(f.transport->frames.size(), 1u);
  const auto s = ch->stats();
  EXPECT_EQ(s.records_in, 1u);
  EXPECT_EQ(s.emitted, 1u);
  EXPECT_EQ(s.frames, 1u);
  EXPECT_EQ(s.bytes, 2u);
}

TEST(OutputChannel, AppliesBuiltinsAtEmitTime)
{
  Fixture f;
  auto ch = f.make("{a: x, t: $time_ms}", 0);
  ch->start();
  const RecordPtr r = f.record(*ch, 1);
  f.clock->t = std::chrono::milliseconds(5);
  ch->push(r);
  f.clock->t = std::chrono::milliseconds(7);
  ch->push(r);  // same source record, fresh timestamp
  ASSERT_EQ(f.transport->frames.size(), 2u);
  EXPECT_EQ(f.transport->frames[0].data[1], 5);
  EXPECT_EQ(f.transport->frames[1].data[1], 7);
  EXPECT_EQ(f.transport->frames[1].stamp, std::chrono::milliseconds(7));
  // The source's record is not modified.
  EXPECT_EQ(std::get<std::int64_t>((*r)[1]), 0);
}

TEST(OutputChannel, CountsDropsAndEncodeErrors)
{
  Fixture f;
  auto ch = f.make("{a: x}", 0);
  ch->start();
  ch->push(f.record(*ch, 99));
  f.transport->refuse = true;
  ch->push(f.record(*ch, 1));
  const auto s = ch->stats();
  EXPECT_EQ(s.encode_errors, 1u);
  EXPECT_EQ(s.dropped, 1u);
  EXPECT_EQ(s.frames, 0u);
}

TEST(OutputChannel, RepliesGoBackTheWayTheRequestCame)
{
  Fixture f;
  auto ch = f.make("{a: x}", 0);
  ch->start();
  FrameMeta requester;
  requester.endpoint = Endpoint{"10.0.0.9", 1234};
  requester.attributes["fd"] = "1";
  ch->request({{0, Value{std::uint64_t{42}}}}, requester);
  ASSERT_EQ(f.transport->frames.size(), 1u);
  const Frame & reply = f.transport->frames[0];
  EXPECT_EQ(reply.data[0], 42);
  EXPECT_EQ(reply.meta.endpoint, requester.endpoint);
  EXPECT_EQ(reply.meta.attributes.at("fd"), "1");
  EXPECT_EQ(ch->stats().requests, 1u);
}

}  // namespace
}  // namespace simport
