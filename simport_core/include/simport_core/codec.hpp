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

#ifndef SIMPORT_CORE__CODEC_HPP_
#define SIMPORT_CORE__CODEC_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/config.hpp"
#include "simport_core/context.hpp"
#include "simport_core/frame.hpp"
#include "simport_core/schema.hpp"

namespace simport
{

/// Outcome of Codec::decode().
struct DecodeResult
{
  enum class Code : std::uint8_t
  {
    kOk,
    /// Frame shorter or longer than the layout requires.
    kBadLength,
    /// Checksum/CRC mismatch.
    kBadChecksum,
    /// Constant/sync field or text literal did not match: probably a different message type.
    kNoMatch,
    /// A field could not be parsed.
    kBadValue,
    /// This codec cannot decode.
    kUnsupported,
  };

  Code code = Code::kOk;
  std::string detail;

  bool ok() const {return code == Code::kOk;}
  static DecodeResult success() {return {};}
  static DecodeResult failure(Code code, std::string detail = {})
  {
    return {code, std::move(detail)};
  }
};

/// Converts Records to wire frames and back. This is where a device's protocol lives.
///
/// A codec instance belongs to exactly one channel and is only called from one thread at a
/// time, so it may keep state (sequence counters, ...).
class Codec
{
public:
  virtual ~Codec() = default;

  /// Parse and validate the message description: `message` is the message (its `fields`, or
  /// for a plugin codec its `codec:` block); `envelope` is the port's `packet:` block, or
  /// undefined. Report problems with `config.error(...)` so they point at the YAML line.
  virtual void configure(const Config & message, const Config & envelope, Context & context) = 0;

  /// Where input fields get their values: (field name, the field's description). The device
  /// builds the mapping from the `from:`, `scale:`, ... keys of each description.
  virtual std::vector<std::pair<std::string, Config>> sources() const {return {};}

  /// Fields this codec consumes (encode) or produces (decode). Valid after configure(). Sources
  /// bind their mapping to the kInput fields of this schema.
  virtual SchemaPtr schema() const = 0;

  virtual bool can_encode() const {return true;}
  virtual bool can_decode() const {return false;}

  /// Append zero or more frames for `record` to `out`. `record` follows schema(). Only throws
  /// for programming errors; data problems (e.g. out-of-range values) saturate.
  virtual void encode(const Record & record, std::vector<Frame> & out) = 0;

  /// Decode one complete frame into `out`, which is pre-initialised with schema().
  virtual DecodeResult decode(const Frame & frame, Record & out)
  {
    (void)frame;
    (void)out;
    return DecodeResult::failure(DecodeResult::Code::kUnsupported, "codec cannot decode");
  }

  /// Reset protocol state such as sequence counters. Called when the device is activated.
  virtual void reset() {}
};

using CodecPtr = std::shared_ptr<Codec>;

}  // namespace simport

#endif  // SIMPORT_CORE__CODEC_HPP_
