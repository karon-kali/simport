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

#ifndef SIMPORT_PLUGINS__NET_HPP_
#define SIMPORT_PLUGINS__NET_HPP_

#include <netinet/in.h>

#include <string>

#include "simport_core/config.hpp"
#include "simport_core/frame.hpp"

namespace simport::plugins
{

/// Parse `{ host: 127.0.0.1, port: 2368 }` or `"127.0.0.1:2368"`. Hosts are IPv4 literals or
/// names resolvable to IPv4 (e.g. localhost).
Endpoint parse_endpoint(const Config & node);

/// Resolve an IPv4 address. Throws std::invalid_argument.
sockaddr_in to_sockaddr(const Endpoint & endpoint);

Endpoint from_sockaddr(const sockaddr_in & addr);

/// `host:port`
std::string to_string(const Endpoint & endpoint);

/// Text for an errno value.
std::string errno_text(int err);

}  // namespace simport::plugins

#endif  // SIMPORT_PLUGINS__NET_HPP_
