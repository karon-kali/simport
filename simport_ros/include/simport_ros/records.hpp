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

#ifndef SIMPORT_ROS__RECORDS_HPP_
#define SIMPORT_ROS__RECORDS_HPP_

#include <string>
#include <vector>

#include "simport_core/output_channel.hpp"
#include "simport_core/schema.hpp"
#include "simport_msgs/msg/record.hpp"

namespace simport
{

/// A decoded packet as a ROS message; `name` is the receive message it matched.
simport_msgs::msg::Record to_msg(const Record & record, const std::string & name);

/// Field values of `msg` for a message with `schema`, matched by name and converted to the
/// field's kind. Names the schema does not have are returned in `unknown`.
FieldOverrides from_msg(
  const simport_msgs::msg::Record & msg, const Schema & schema,
  std::vector<std::string> * unknown = nullptr);

}  // namespace simport

#endif  // SIMPORT_ROS__RECORDS_HPP_
