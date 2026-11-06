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

// Minimal plugins wired by hand into an outbound and an inbound pipeline. This keeps the
// interfaces honest: if writing a plugin gets awkward, it shows up here first.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "simport_core/codec.hpp"
#include "simport_core/frame_filter.hpp"
#include "simport_core/framer.hpp"
#include "simport_core/transport.hpp"
#include "simport_core/trigger.hpp"

namespace simport
{
namespace
{

/// One unsigned byte per record; decodes too.
class ByteCodec : public Codec
{
public:
  void configure(const Config & config, const Config &, Context &) override
  {
    config.expect_keys({"plugin"});
    schema_ = std::make_shared<const Schema>(
      std::vector<FieldSpec>{{"v", ValueKind::kUint, FieldRole::kInput}});
  }
  SchemaPtr schema() const override {return schema_;}
  bool can_decode() const override {return true;}
  void encode(const Record & record, std::vector<Frame> & out) override
  {
    out.push_back(Frame{{saturate_cast<std::uint8_t>(record[0])}, {}, record.stamp});
  }
  DecodeResult decode(const Frame & frame, Record & out) override
  {
    if (frame.data.size() != 1) {
      return DecodeResult::failure(DecodeResult::Code::kBadLength, "need 1 byte");
    }
    out[0] = std::uint64_t{frame.data[0]};
    return DecodeResult::success();
  }

private:
  SchemaPtr schema_;
};

class PassTrigger : public Trigger
{
public:
  void configure(const Config &, Context &) override {}
  void start(EmitCallback emit) override {emit_ = std::move(emit);}
  void stop() override {emit_ = nullptr;}
  void on_record(RecordPtr record) override
  {
    if (emit_) {
      emit_(record);
    }
  }

private:
  EmitCallback emit_;
};

/// Duplicates every frame.
class DupFilter : public FrameFilter
{
public:
  void configure(const Config &, Context &) override {}
  void process(Frame && frame, const FrameSink & next) override
  {
    Frame copy = frame;
    next(std::move(frame));
    next(std::move(copy));
  }
};

/// Loops written bytes back to the receive callback.
class LoopbackTransport : public Transport
{
public:
  void configure(const Config &, Context &) override {}
  void set_receive_callback(ReceiveCallback cb) override {cb_ = std::move(cb);}
  void open() override {open_ = true;}
  void close() override {open_ = false;}
  bool is_open() const override {return open_;}
  TransportCaps caps() const override {return TransportCaps{};}
  bool write(const Frame & frame) override
  {
    if (!open_) {
      ++stats_.tx_dropped;
      return false;
    }
    ++stats_.frames_tx;
    stats_.bytes_tx += frame.data.size();
    if (cb_) {
      cb_(frame.data, frame.meta, frame.stamp);
    }
    return true;
  }
  TransportStats stats() const override {return stats_;}
  std::string describe() const override {return "loopback";}

private:
  ReceiveCallback cb_;
  bool open_ = false;
  TransportStats stats_;
};

/// Each byte is a frame.
class ByteFramer : public Framer
{
public:
  void configure(const Config &, Context &) override {}
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override
  {
    for (const auto b : data) {
      ++stats_.frames;
      on_frame(Frame{{b}, {}, stamp});
    }
  }
  void reset() override {}
  FramerStats stats() const override {return stats_;}

private:
  FramerStats stats_;
};

TEST(Interfaces, HandWiredPipelineRoundTrips)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  const Config cfg = Config::from_string("plugin: x\n");

  ByteCodec codec;
  codec.configure(cfg, Config(), ctx);
  PassTrigger trigger;
  trigger.configure(cfg, ctx);
  DupFilter filter;
  filter.configure(cfg, ctx);
  LoopbackTransport transport;
  transport.configure(cfg, ctx);
  ByteFramer framer;
  framer.configure(cfg, ctx);

  std::vector<std::uint64_t> received;
  transport.set_receive_callback(
    [&](ByteView data, const FrameMeta &, Timestamp stamp) {
      framer.feed(
        data, stamp, [&](Frame && f) {
          Record r = Record::make(codec.schema());
          ASSERT_TRUE(codec.decode(f, r).ok());
          received.push_back(std::get<std::uint64_t>(r[0]));
        });
    });
  transport.open();

  std::vector<Frame> frames;
  trigger.start(
    [&](const RecordPtr & record) {
      frames.clear();
      codec.encode(*record, frames);
      for (auto & f : frames) {
        filter.process(std::move(f), [&](Frame && out) {transport.write(out);});
      }
    });

  for (const double v : {1.0, 300.0}) {
    auto r = std::make_shared<Record>(Record::make(codec.schema()));
    (*r)[0] = v;
    trigger.on_record(r);
  }
  trigger.stop();
  transport.close();

  EXPECT_EQ(received, (std::vector<std::uint64_t>{1, 1, 255, 255}));
  EXPECT_EQ(transport.stats().frames_tx, 4u);
  EXPECT_EQ(framer.stats().frames, 4u);
  EXPECT_FALSE(transport.write(Frame{}));
  EXPECT_EQ(transport.stats().tx_dropped, 1u);

  Record r = Record::make(codec.schema());
  EXPECT_EQ(codec.decode(Frame{{1, 2}, {}, {}}, r).code, DecodeResult::Code::kBadLength);
}

}  // namespace
}  // namespace simport
