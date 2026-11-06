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

#include "simport_core/output_channel.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace simport
{

namespace
{
Timestamp wall_now()
{
  return std::chrono::duration_cast<Timestamp>(
    std::chrono::system_clock::now().time_since_epoch());
}
}  // namespace

OutputChannel::OutputChannel(
  std::string name, Context context, MappingSpec mapping, TriggerPtr trigger, CodecPtr codec,
  std::vector<FrameFilterPtr> filters, TransportPtr transport)
: name_(std::move(name)), context_(std::move(context)), mapping_(std::move(mapping)),
  trigger_(std::move(trigger)), codec_(std::move(codec)), filters_(std::move(filters)),
  transport_(std::move(transport))
{
  sinks_.resize(filters_.size() + 1);
  sinks_.back() = [this](Frame && frame) {write(std::move(frame));};
  for (std::size_t i = filters_.size(); i-- > 0; ) {
    sinks_[i] = [this, i](Frame && frame) {
        filters_[i]->process(std::move(frame), sinks_[i + 1]);
      };
  }
}

OutputChannel::~OutputChannel()
{
  stop();
}

void OutputChannel::start()
{
  if (running_.exchange(true)) {
    return;
  }
  codec_->reset();
  for (auto & f : filters_) {
    f->start();
  }
  trigger_->start([this](const RecordPtr & record) {emit(record, nullptr, nullptr);});
}

void OutputChannel::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  trigger_->stop();
  {
    // Wait for an in-flight request() before stopping the filters it may be using.
    std::lock_guard<std::mutex> lock(emit_mutex_);
  }
  for (auto & f : filters_) {
    f->stop();
  }
}

void OutputChannel::push(RecordPtr record)
{
  if (!running_) {
    return;
  }
  ++records_in_;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_ = record;
  }
  trigger_->on_record(std::move(record));
}

void OutputChannel::request(const FieldOverrides & overrides, const FrameMeta & meta)
{
  if (!running_) {
    return;
  }
  ++requests_;
  RecordPtr record;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    record = latest_;
  }
  if (!record) {
    record = std::make_shared<Record>(mapping_.make_record());
  }
  emit(record, &overrides, &meta);
}

void OutputChannel::emit(
  const RecordPtr & record, const FieldOverrides * overrides, const FrameMeta * meta)
{
  std::lock_guard<std::mutex> lock(emit_mutex_);
  if (!running_) {
    return;
  }
  const bool requested = overrides != nullptr;
  ++emitted_;
  const Timestamp now = context_.clock().now();
  try {
    scratch_frames_.clear();
    const bool modify = !mapping_.builtins().empty() || (requested && !overrides->empty());
    if (!modify) {
      codec_->encode(*record, scratch_frames_);
    } else {
      Record r = *record;
      mapping_.apply_builtins(r, BuiltinInputs{now, wall_now(), record->stamp});
      if (requested) {
        for (const auto & [index, value] : *overrides) {
          r[index] = value;
        }
      }
      codec_->encode(r, scratch_frames_);
    }
  } catch (const std::exception & e) {
    if (encode_errors_++ == 0) {
      context_.logger().error("encode failed (further errors counted silently): {}", e.what());
    }
    return;
  }
  for (Frame & frame : scratch_frames_) {
    if (frame.stamp == Timestamp::zero()) {
      frame.stamp = now;
    }
    if (meta != nullptr) {
      // A reply goes back the way the request came: to its sender, in the same kind of frame
      // (e.g. CAN-FD), unless the codec says otherwise.
      if (!frame.meta.endpoint && meta->endpoint) {
        frame.meta.endpoint = meta->endpoint;
      }
      for (const auto & [key, value] : meta->attributes) {
        frame.meta.attributes.try_emplace(key, value);
      }
    }
    sinks_.front()(std::move(frame));
  }
}

void OutputChannel::write(Frame && frame)
{
  if (transport_->write(frame)) {
    ++frames_;
    bytes_ += frame.data.size();
    if (tap_) {
      tap_(frame);
    }
  } else {
    ++dropped_;
  }
}

OutputChannelStats OutputChannel::stats() const
{
  return OutputChannelStats{records_in_, emitted_, requests_, frames_, bytes_, dropped_,
    encode_errors_};
}

}  // namespace simport
