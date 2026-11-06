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

#ifndef SIMPORT_ROS__DYNAMIC_MESSAGE_HPP_
#define SIMPORT_ROS__DYNAMIC_MESSAGE_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "simport_core/mapping.hpp"
#include "simport_core/value.hpp"

namespace simport
{

using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;
using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;

/// A ROS message of a type known only at runtime, e.g. "sensor_msgs/msg/Imu". Owns an
/// initialised C++ message instance that serialized messages are deserialized into.
class DynamicMessage
{
public:
  /// Load the type support for `type` (`pkg/msg/Name`; `pkg/Name` is accepted). Throws
  /// std::invalid_argument if the type cannot be found.
  explicit DynamicMessage(const std::string & type);
  ~DynamicMessage();

  DynamicMessage(const DynamicMessage &) = delete;
  DynamicMessage & operator=(const DynamicMessage &) = delete;

  /// Normalised type name, `pkg/msg/Name`.
  const std::string & type() const {return type_;}
  const MessageMembers & members() const {return *members_;}

  /// Throws std::runtime_error on malformed data.
  void deserialize(const rclcpp::SerializedMessage & message);

  const void * data() const {return buffer_.get();}

private:
  std::string type_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library_;
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library_;
  const MessageMembers * members_ = nullptr;
  std::unique_ptr<rclcpp::SerializationBase> serialization_;
  std::unique_ptr<std::max_align_t[]> buffer_;
};

/// `pkg::msg::Name` → `pkg/msg/Name`.
std::string type_name(const MessageMembers & members);

/// Reads one field, identified by a path, out of messages of a given type. Binding resolves
/// names and checks types once; read() then only follows offsets and array accessors.
class FieldAccessor
{
public:
  /// Throws std::invalid_argument with a human-readable reason (unknown field, message where
  /// a value is needed, unsupported type, ...).
  static FieldAccessor bind(const MessageMembers & root, const std::vector<PathElement> & path);

  /// Value of the field in `message` (an instance of the bound type). nullopt if an array
  /// index is out of range in this particular message.
  std::optional<Value> read(const void * message) const;

  /// kFloatArray when the path ends at a numeric array; otherwise the element kind.
  ValueKind kind() const {return kind_;}

private:
  struct Step
  {
    const MessageMember * member;
    std::optional<std::size_t> index;
  };

  std::vector<Step> steps_;
  ValueKind kind_ = ValueKind::kFloat;
};

}  // namespace simport

#endif  // SIMPORT_ROS__DYNAMIC_MESSAGE_HPP_
