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

#ifndef SIMPORT_PLUGINS__FRAMERS_HPP_
#define SIMPORT_PLUGINS__FRAMERS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "simport_core/framer.hpp"

namespace simport::plugins
{

/// Bytes from YAML: a list of byte values (`[0xAA, 0x55]`) or a string (`"\r\n"`, `"$"`).
Bytes parse_bytes(const Config & node);

/// Frames of a constant size.
///
/// ```yaml
/// framer: { plugin: simport/FixedLength, size: 8 }
/// ```
class FixedLength : public Framer
{
public:
  void configure(const Config & config, Context & context) override;
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override;
  void reset() override {buffer_.clear();}
  FramerStats stats() const override {return stats_;}

private:
  std::size_t size_ = 1;
  Bytes buffer_;
  FramerStats stats_;
};

/// Frames ending in a delimiter, e.g. text lines.
///
/// ```yaml
/// framer:
///   plugin: simport/Delimiter
///   delimiter: "\r\n"          # string or byte list
///   include_delimiter: true    # keep it in the frame (TextCodec formats usually end with it)
///   max_size: 1024             # longer input is discarded
/// ```
class Delimiter : public Framer
{
public:
  void configure(const Config & config, Context & context) override;
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override;
  void reset() override {buffer_.clear();}
  FramerStats stats() const override {return stats_;}

private:
  Bytes delimiter_;
  bool include_ = true;
  std::size_t max_size_ = 1024;
  Bytes buffer_;
  FramerStats stats_;
};

/// Binary frames that start with a sync pattern and carry their length.
///
/// ```yaml
/// framer:
///   plugin: simport/SyncLength
///   sync: [0xAA, 0x55]
///   length: { offset: 2, type: u8, endian: little }  # where the length field is
///   adjust: 5          # frame size = length value + adjust (bytes the length doesn't count)
///   max_size: 1024
/// ```
///
/// When the frame layout depends on a header byte, give `cases` instead of `length`/`adjust`.
/// The first case whose `when` matches decides the frame size; a case without `when` matches
/// anything. Each case has either a fixed `size` or a `length` field with `adjust`.
///
/// ```yaml
/// framer:
///   plugin: simport/SyncLength
///   sync: [0xA5]
///   cases:
///     - when: { offset: 1, mask: 0x80, equals: 0x00 }   # (byte[1] & mask) == equals
///       size: 2                                        # A5 cmd
///     - length: { offset: 2, type: u8 }                # A5 cmd len payload... checksum
///       adjust: 4
/// ```
///
/// Bytes before a sync pattern, implausible lengths and frames that no case matches are
/// skipped (counted as discarded). Offsets count from the first sync byte.
///
/// Given a packet envelope (`packet:` of a device, with `start` and a `length_of` field that
/// counts the body) instead, it works out sync, length position and adjustment by itself.
class SyncLength : public Framer
{
public:
  void configure(const Config & config, Context & context) override;
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override;
  void reset() override {buffer_.clear();}
  FramerStats stats() const override {return stats_;}

private:
  struct Case
  {
    // Condition: (byte[when_offset] & when_mask) == when_equals. Unset: always matches.
    std::optional<std::size_t> when_offset;
    std::uint8_t when_mask = 0xFF;
    std::uint8_t when_equals = 0;
    // Frame size: `size`, or the length field plus `adjust`.
    std::optional<std::size_t> size;
    std::size_t length_offset = 0;
    std::size_t length_size = 1;
    bool length_big = false;
    std::int64_t adjust = 0;
  };

  static Case parse_case(const Config & node, bool allow_when);
  void derive_from_envelope(const Config & envelope);

  Bytes sync_;
  std::vector<Case> cases_;
  std::size_t max_size_ = 1024;
  Bytes buffer_;
  FramerStats stats_;
};

/// HTTP/1.x messages: a header ending in an empty line, then `Content-Length` bytes of body.
///
/// ```yaml
/// framer: { plugin: simport/Http, max_size: 1048576 }
/// ```
///
/// Chunked bodies are not supported; such messages and oversized ones are discarded.
class Http : public Framer
{
public:
  void configure(const Config & config, Context & context) override;
  void feed(ByteView data, Timestamp stamp, const FrameCallback & on_frame) override;
  void reset() override {buffer_.clear();}
  FramerStats stats() const override {return stats_;}

private:
  std::size_t max_size_ = 1 << 20;
  Bytes buffer_;
  FramerStats stats_;
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__FRAMERS_HPP_
