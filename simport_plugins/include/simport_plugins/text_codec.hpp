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

#ifndef SIMPORT_PLUGINS__TEXT_CODEC_HPP_
#define SIMPORT_PLUGINS__TEXT_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <string_view>
#include <vector>

#include "simport_core/checksum.hpp"
#include "simport_core/codec.hpp"

namespace simport::plugins
{

/// How one placeholder is rendered and parsed.
struct TextFormat
{
  enum class Kind : std::uint8_t
  {
    kPrintf,    ///< %d %i %u %x %X %o %f %F %e %E %g %G %s with flags/width/precision
    kNmeaLat,   ///< degrees -> `ddmm.mmmm,N`   (`nmea_lat` or `nmea_lat.5` for 5 decimals)
    kNmeaLon,   ///< degrees -> `dddmm.mmmm,E`
    kTimeOfDay,  ///< seconds (Unix or of day) -> `hhmmss` / `hhmmss.ss` (one s per decimal)
    kDate,      ///< Unix seconds -> `ddmmyy`
  };

  Kind kind = Kind::kPrintf;
  std::string printf_spec;  ///< kPrintf: normalised C format, e.g. "%08.3f" / "%lld"
  char conversion = 'g';    ///< kPrintf: conversion character
  int decimals = 0;         ///< NMEA lat/lon and time of day

  /// Parse a spec such as `%.3f`, `nmea_lat`, `hhmmss.ss`. Throws std::invalid_argument.
  static TextFormat parse(std::string_view spec);

  /// Value kind this format produces/consumes.
  ValueKind kind_of_value() const;

  std::string render(const Value & value) const;

  /// Parse `token` (may be empty: returns nullopt, i.e. "field not present").
  /// Throws std::invalid_argument on malformed input.
  std::optional<Value> parse_value(std::string_view token) const;
};

/// ASCII frames rendered from a template, e.g. NMEA 0183 sentences.
///
/// ```yaml
/// send:
///   - name: gga
///     format: "${$begin}GPGGA,{utc:hhmmss.ss},{lat:nmea_lat},{lon:nmea_lon},{fix:%d},..."
///             "{$end}*{$checksum:%02X}\r\n"
///     checksum: nmea          # algorithm for {$checksum}
///     fields:                 # where placeholders get their values
///       - utc: $stamp_s
///       - lat: { from: latitude }
/// ```
///
/// - `{name}` / `{name:spec}`: a field (default spec `%g`); a name may appear more than once.
/// - `{$begin}`, `{$end}`: zero-width markers delimiting the checksummed text (defaults: start
///   of the frame, and the checksum placeholder).
/// - `{$checksum}` / `{$checksum:spec}`: the checksum (default spec `%02X`).
/// - `{{` and `}}`: literal braces.
///
/// Decoding matches literal text and splits fields at the next literal, so two fields must be
/// separated by literal text. Empty fields (as in NMEA `,,`) decode as absent and keep their
/// default value.
class TextCodec : public Codec
{
public:
  void configure(const Config & message, const Config & envelope, Context & context) override;
  std::vector<std::pair<std::string, Config>> sources() const override {return sources_;}
  SchemaPtr schema() const override {return schema_;}
  bool can_decode() const override {return true;}
  void encode(const Record & record, std::vector<Frame> & out) override;
  DecodeResult decode(const Frame & frame, Record & out) override;

  /// Render `record` to text (what encode() puts in the frame).
  std::string render(const Record & record) const;

private:
  std::vector<std::pair<std::string, Config>> sources_;
  struct Segment
  {
    enum class Kind : std::uint8_t {kLiteral, kField, kBegin, kEnd, kChecksum};
    Kind kind;
    std::string literal;     ///< kLiteral
    std::size_t field = 0;   ///< kField: schema index
    TextFormat format;       ///< kField / kChecksum
  };

  std::vector<Segment> segments_;
  std::optional<Checksum> checksum_;
  SchemaPtr schema_;
};

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__TEXT_CODEC_HPP_
