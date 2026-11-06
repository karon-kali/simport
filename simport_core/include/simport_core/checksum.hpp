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

#ifndef SIMPORT_CORE__CHECKSUM_HPP_
#define SIMPORT_CORE__CHECKSUM_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

#include "simport_core/config.hpp"
#include "simport_core/types.hpp"

namespace simport
{

/// Parameters of a CRC in the usual Rocksoft/"CRC catalogue" model.
struct CrcParams
{
  int width = 16;            ///< bits; 8, 16, 24, 32, 40, 48, 56 or 64
  std::uint64_t poly = 0;    ///< normal (MSB-first) representation, without the top bit
  std::uint64_t init = 0;
  bool refin = false;
  bool refout = false;
  std::uint64_t xorout = 0;
};

/// A checksum algorithm: CRCs and the simple sums used by device protocols.
///
/// Presets (check value of the ASCII string "123456789" in brackets):
///   xor8 / nmea [0x31], sum8 [0xDD], sum16 [0x01DD], lrc8 [0x23],
///   fletcher8 (u-blox UBX: CK_A in the high byte, CK_B low) [0xDD15],
///   crc8 [0xF4], crc8_maxim [0xA1], crc16_ccitt_false [0x29B1], crc16_xmodem [0x31C3],
///   crc16_kermit [0x2189], crc16_modbus [0x4B37], crc16_arc [0xBB3D], crc16_buypass [0xFEE8],
///   crc32 [0xCBF43926], crc32c [0xE3069283], crc64_xz [0x995DC9BBDF1939FA] (ECMA-182).
class Checksum
{
public:
  /// Named preset; throws std::invalid_argument listing the available names.
  static Checksum preset(std::string_view name);

  /// Custom CRC; throws std::invalid_argument for unsupported widths.
  static Checksum crc(const CrcParams & params);

  /// Parse a checksum spec from configuration:
  /// - a preset name: `crc16_modbus`
  /// - a mapping with a preset: `{ algo: crc16_modbus, ... }`
  /// - a custom CRC: `{ algo: crc, width: 16, poly: 0x1021, init: 0xFFFF, refin: false,
  ///   refout: false, xorout: 0 }`
  /// `extra_keys` are additional keys the caller handles (e.g. `from`, `to`).
  static Checksum parse(
    const Config & node, std::initializer_list<std::string_view> extra_keys = {});

  /// Compute over `data`. The result occupies the low width_bytes() bytes.
  std::uint64_t compute(ByteView data) const;

  /// Natural size of the result in bytes.
  std::size_t width_bytes() const {return width_bytes_;}

  const std::string & name() const {return name_;}

  static std::vector<std::string_view> preset_names();

private:
  enum class Kind : std::uint8_t {kCrc, kXor8, kSum8, kSum16, kLrc8, kFletcher8};

  Checksum(Kind kind, std::string name, std::size_t width_bytes);

  Kind kind_;
  std::string name_;
  std::size_t width_bytes_;
  CrcParams crc_{};
  std::uint64_t mask_ = 0;
  std::array<std::uint64_t, 256> table_{};
};

}  // namespace simport

#endif  // SIMPORT_CORE__CHECKSUM_HPP_
