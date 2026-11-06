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

#include "simport_plugins/http_codec.hpp"

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace simport::plugins
{

std::string http_reason(std::uint64_t status)
{
  switch (status) {
    case 100: return "Continue";
    case 200: return "OK";
    case 201: return "Created";
    case 202: return "Accepted";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 409: return "Conflict";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default: return "Status";
  }
}

void HttpCodec::configure(const Config & config, const Config & envelope, Context &)
{
  if (envelope.defined() && !envelope.is_null()) {
    config.error("HTTP messages cannot use the port's binary packet envelope");
  }
  config.expect_keys({"plugin", "status", "content_type", "body", "body_file", "headers"});
  status_ = config.get<std::uint16_t>("status", 200);
  if (status_ < 100 || status_ > 599) {
    config["status"].error("expected an HTTP status code (100-599)");
  }
  content_type_ = config.get<std::string>("content_type", "application/json");
  if (config.has("body") && config.has("body_file")) {
    config.error("give either 'body' or 'body_file'");
  }
  body_ = config.get<std::string>("body", "");
  if (config.has("body_file")) {
    const auto path = config["body_file"].resolve_path(config.get<std::string>("body_file"));
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      config["body_file"].error("cannot read '" + path.string() + "'");
    }
    std::ostringstream text;
    text << in.rdbuf();
    body_ = text.str();
  }
  headers_.clear();
  for (const auto & [name, value] : config["headers"].entries()) {
    headers_.emplace_back(name, value.as<std::string>());
  }

  auto schema = std::make_shared<Schema>();
  method_ = schema->add({"method", ValueKind::kString, FieldRole::kComputed});
  path_ = schema->add({"path", ValueKind::kString, FieldRole::kComputed});
  query_ = schema->add({"query", ValueKind::kString, FieldRole::kComputed});
  body_field_ = schema->add({"body", ValueKind::kString, FieldRole::kInput});
  status_field_ = schema->add({"status", ValueKind::kUint, FieldRole::kInput});
  schema_ = std::move(schema);
}

void HttpCodec::encode(const Record & record, std::vector<Frame> & out)
{
  // Mapped values override the configured response; unmapped fields are empty / 0.
  const auto & mapped_body = std::get<std::string>(record[body_field_]);
  const std::string & body = mapped_body.empty() ? body_ : mapped_body;
  const auto mapped_status = saturate_cast<std::uint64_t>(record[status_field_]);
  const std::uint64_t status = mapped_status != 0 ? mapped_status : status_;

  std::string text = "HTTP/1.1 " + std::to_string(status) + " " + http_reason(status) + "\r\n";
  if (status >= 200 && status != 204 && status != 304) {
    text += "Content-Type: " + content_type_ + "\r\n";
    text += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  }
  for (const auto & [name, value] : headers_) {
    text += name + ": " + value + "\r\n";
  }
  text += "\r\n";
  if (status >= 200 && status != 204 && status != 304) {
    text += body;
  }
  out.push_back(Frame{Bytes(text.begin(), text.end()), {}, {}});
}

DecodeResult HttpCodec::decode(const Frame & frame, Record & out)
{
  const std::string_view text(reinterpret_cast<const char *>(frame.data.data()), frame.data.size());
  const auto line_end = text.find("\r\n");
  const auto header_end = text.find("\r\n\r\n");
  if (line_end == std::string_view::npos || header_end == std::string_view::npos) {
    return DecodeResult::failure(DecodeResult::Code::kBadLength, "incomplete HTTP message");
  }
  // Request line: METHOD target HTTP/1.x
  const std::string_view line = text.substr(0, line_end);
  const auto sp1 = line.find(' ');
  const auto sp2 = line.rfind(' ');
  if (sp1 == std::string_view::npos || sp2 == sp1 || !line.substr(sp2 + 1).starts_with("HTTP/")) {
    return DecodeResult::failure(DecodeResult::Code::kNoMatch, "not an HTTP request");
  }
  const std::string_view target = line.substr(sp1 + 1, sp2 - sp1 - 1);
  const auto q = target.find('?');
  out[method_] = std::string(line.substr(0, sp1));
  out[path_] = std::string(target.substr(0, q));
  out[query_] = q == std::string_view::npos ? std::string() : std::string(target.substr(q + 1));
  out[body_field_] = std::string(text.substr(header_end + 4));
  out.stamp = frame.stamp;
  return DecodeResult::success();
}

}  // namespace simport::plugins

PLUGINLIB_EXPORT_CLASS(simport::plugins::HttpCodec, simport::Codec)
