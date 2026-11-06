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

#ifndef SIMPORT_PLUGINS__HTTP_CODEC_HPP_
#define SIMPORT_PLUGINS__HTTP_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "simport_core/codec.hpp"

namespace simport::plugins
{

/// HTTP/1.1 for a device's web API: decodes requests from the driver and encodes responses.
///
/// ```yaml
/// receive:                            # a request: method, path (without the query), query, body
///   - name: request
///     codec: { plugin: simport/HttpCodec }
///     to: { service: /my_sensor/http }
///     reply: response
/// send:
///   - name: response                  # the service's answer: status and body
///     codec:
///       plugin: simport/HttpCodec
///       status: 200                   # default when the answer gives none
///       content_type: application/json
///       headers: { Cache-Control: no-cache }   # optional extra headers
/// ```
///
/// Fields: `method`, `path`, `query` and `body` (strings) are filled when decoding a request;
/// `status` and `body` of a response come from the record (or `status`/`body`/`body_file`).
class HttpCodec : public Codec
{
public:
  void configure(const Config & config, const Config & envelope, Context & context) override;
  SchemaPtr schema() const override {return schema_;}
  bool can_decode() const override {return true;}
  void encode(const Record & record, std::vector<Frame> & out) override;
  DecodeResult decode(const Frame & frame, Record & out) override;

private:
  std::uint64_t status_ = 200;
  std::string content_type_;
  std::string body_;
  std::vector<std::pair<std::string, std::string>> headers_;
  SchemaPtr schema_;
  std::size_t method_ = 0, path_ = 0, query_ = 0, body_field_ = 0, status_field_ = 0;
};

/// Standard reason phrase for an HTTP status code ("OK", "Not Found", ...).
std::string http_reason(std::uint64_t status);

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__HTTP_CODEC_HPP_
