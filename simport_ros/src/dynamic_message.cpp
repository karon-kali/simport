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

#include "simport_ros/dynamic_message.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace simport
{

namespace ft = rosidl_typesupport_introspection_cpp;

namespace
{

std::string normalise_type(const std::string & type)
{
  const auto first = type.find('/');
  if (first == std::string::npos || first == 0 || first + 1 == type.size()) {
    throw std::invalid_argument(
            "invalid message type '" + type + "': expected <package>/msg/<Name>");
  }
  const auto second = type.find('/', first + 1);
  if (second == std::string::npos) {
    return type.substr(0, first) + "/msg/" + type.substr(first + 1);
  }
  return type;
}

const MessageMembers & nested(const MessageMember & m)
{
  return *static_cast<const MessageMembers *>(m.members_->data);
}

std::string field_list(const MessageMembers & members)
{
  std::string out;
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    out += (i ? ", " : "") + std::string(members.members_[i].name_);
  }
  return out;
}

std::optional<ValueKind> kind_of_type(std::uint8_t type_id)
{
  switch (type_id) {
    case ft::ROS_TYPE_FLOAT:
    case ft::ROS_TYPE_DOUBLE:
    case ft::ROS_TYPE_LONG_DOUBLE:
      return ValueKind::kFloat;
    case ft::ROS_TYPE_CHAR:
    case ft::ROS_TYPE_OCTET:
    case ft::ROS_TYPE_UINT8:
    case ft::ROS_TYPE_UINT16:
    case ft::ROS_TYPE_UINT32:
    case ft::ROS_TYPE_UINT64:
      return ValueKind::kUint;
    case ft::ROS_TYPE_INT8:
    case ft::ROS_TYPE_INT16:
    case ft::ROS_TYPE_INT32:
    case ft::ROS_TYPE_INT64:
      return ValueKind::kInt;
    case ft::ROS_TYPE_BOOLEAN:
      return ValueKind::kBool;
    case ft::ROS_TYPE_STRING:
      return ValueKind::kString;
    default:
      return std::nullopt;  // wchar, wstring, message
  }
}

/// Read a primitive stored at `p` (plain field or std::array element).
Value read_scalar(std::uint8_t type_id, const void * p)
{
  switch (type_id) {
    case ft::ROS_TYPE_FLOAT: return static_cast<double>(*static_cast<const float *>(p));
    case ft::ROS_TYPE_DOUBLE: return *static_cast<const double *>(p);
    case ft::ROS_TYPE_LONG_DOUBLE:
      return static_cast<double>(*static_cast<const long double *>(p));
    case ft::ROS_TYPE_CHAR:
    case ft::ROS_TYPE_OCTET:
    case ft::ROS_TYPE_UINT8: return std::uint64_t{*static_cast<const std::uint8_t *>(p)};
    case ft::ROS_TYPE_UINT16: return std::uint64_t{*static_cast<const std::uint16_t *>(p)};
    case ft::ROS_TYPE_UINT32: return std::uint64_t{*static_cast<const std::uint32_t *>(p)};
    case ft::ROS_TYPE_UINT64: return *static_cast<const std::uint64_t *>(p);
    case ft::ROS_TYPE_INT8: return std::int64_t{*static_cast<const std::int8_t *>(p)};
    case ft::ROS_TYPE_INT16: return std::int64_t{*static_cast<const std::int16_t *>(p)};
    case ft::ROS_TYPE_INT32: return std::int64_t{*static_cast<const std::int32_t *>(p)};
    case ft::ROS_TYPE_INT64: return *static_cast<const std::int64_t *>(p);
    case ft::ROS_TYPE_BOOLEAN: return *static_cast<const bool *>(p);
    case ft::ROS_TYPE_STRING: return *static_cast<const std::string *>(p);
    default: return 0.0;
  }
}

/// Read element `i` of an array field via the introspection fetch function, which also works
/// for std::vector<bool>.
template<typename T>
T fetch(const MessageMember & m, const void * field, std::size_t i)
{
  T value{};
  m.fetch_function(field, i, &value);
  return value;
}

Value read_element(const MessageMember & m, const void * field, std::size_t i)
{
  switch (m.type_id_) {
    case ft::ROS_TYPE_FLOAT: return static_cast<double>(fetch<float>(m, field, i));
    case ft::ROS_TYPE_DOUBLE: return fetch<double>(m, field, i);
    case ft::ROS_TYPE_LONG_DOUBLE: return static_cast<double>(fetch<long double>(m, field, i));
    case ft::ROS_TYPE_CHAR:
    case ft::ROS_TYPE_OCTET:
    case ft::ROS_TYPE_UINT8: return std::uint64_t{fetch<std::uint8_t>(m, field, i)};
    case ft::ROS_TYPE_UINT16: return std::uint64_t{fetch<std::uint16_t>(m, field, i)};
    case ft::ROS_TYPE_UINT32: return std::uint64_t{fetch<std::uint32_t>(m, field, i)};
    case ft::ROS_TYPE_UINT64: return fetch<std::uint64_t>(m, field, i);
    case ft::ROS_TYPE_INT8: return std::int64_t{fetch<std::int8_t>(m, field, i)};
    case ft::ROS_TYPE_INT16: return std::int64_t{fetch<std::int16_t>(m, field, i)};
    case ft::ROS_TYPE_INT32: return std::int64_t{fetch<std::int32_t>(m, field, i)};
    case ft::ROS_TYPE_INT64: return fetch<std::int64_t>(m, field, i);
    case ft::ROS_TYPE_BOOLEAN: return fetch<bool>(m, field, i);
    case ft::ROS_TYPE_STRING: return fetch<std::string>(m, field, i);
    default: return 0.0;
  }
}

}  // namespace

std::string type_name(const MessageMembers & members)
{
  std::string ns = members.message_namespace_;
  for (std::size_t pos; (pos = ns.find("::")) != std::string::npos; ) {
    ns.replace(pos, 2, "/");
  }
  return ns + "/" + members.message_name_;
}

// -- DynamicMessage ----------------------------------------------------------------------------

DynamicMessage::DynamicMessage(const std::string & type)
: type_(normalise_type(type))
{
  try {
    introspection_library_ =
      rclcpp::get_typesupport_library(type_, "rosidl_typesupport_introspection_cpp");
    const auto * introspection = rclcpp::get_message_typesupport_handle(
      type_, "rosidl_typesupport_introspection_cpp", *introspection_library_);
    members_ = static_cast<const MessageMembers *>(introspection->data);

    cpp_library_ = rclcpp::get_typesupport_library(type_, "rosidl_typesupport_cpp");
    const auto * cpp = rclcpp::get_message_typesupport_handle(
      type_, "rosidl_typesupport_cpp", *cpp_library_);
    serialization_ = std::make_unique<rclcpp::SerializationBase>(cpp);
  } catch (const std::exception & e) {
    throw std::invalid_argument(
            "cannot load message type '" + type_ + "' (is its package built and sourced?): " +
            e.what());
  }
  const std::size_t n = (members_->size_of_ + sizeof(std::max_align_t) - 1) /
    sizeof(std::max_align_t);
  buffer_ = std::make_unique<std::max_align_t[]>(n);
  members_->init_function(buffer_.get(), rosidl_runtime_cpp::MessageInitialization::ALL);
}

DynamicMessage::~DynamicMessage()
{
  if (members_ != nullptr && buffer_) {
    members_->fini_function(buffer_.get());
  }
}

void DynamicMessage::deserialize(const rclcpp::SerializedMessage & message)
{
  serialization_->deserialize_message(&message, buffer_.get());
}

// -- FieldAccessor -----------------------------------------------------------------------------

FieldAccessor FieldAccessor::bind(
  const MessageMembers & root, const std::vector<PathElement> & path)
{
  if (path.empty()) {
    throw std::invalid_argument("empty field path");
  }
  FieldAccessor acc;
  const MessageMembers * members = &root;
  std::string so_far;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const PathElement & e = path[i];
    const bool last = i + 1 == path.size();
    const MessageMember * member = nullptr;
    for (std::uint32_t k = 0; k < members->member_count_; ++k) {
      if (e.name == members->members_[k].name_) {
        member = &members->members_[k];
        break;
      }
    }
    const std::string where = so_far.empty() ? type_name(*members) :
      so_far + " (" + type_name(*members) + ")";
    if (member == nullptr) {
      throw std::invalid_argument(
              "no field '" + e.name + "' in " + where + "; fields: " + field_list(*members));
    }
    so_far += (so_far.empty() ? "" : ".") + e.name;
    if (e.index && !member->is_array_) {
      throw std::invalid_argument("'" + so_far + "' is not an array and cannot be indexed");
    }
    if (e.index && member->is_array_ && !member->is_upper_bound_ && member->array_size_ > 0 &&
      *e.index >= member->array_size_)
    {
      throw std::invalid_argument(
              "index " + std::to_string(*e.index) + " is out of range for '" + so_far +
              "' (fixed size " + std::to_string(member->array_size_) + ")");
    }
    acc.steps_.push_back(Step{member, e.index});

    const bool is_message = member->type_id_ == ft::ROS_TYPE_MESSAGE;
    const bool whole_array = member->is_array_ && !e.index;
    if (!last) {
      if (!is_message) {
        throw std::invalid_argument("'" + so_far + "' is not a message, so it has no field '" +
                path[i + 1].name + "'");
      }
      if (whole_array) {
        throw std::invalid_argument(
                "'" + so_far + "' is an array; select an element, e.g. " + so_far + "[0]." +
                path[i + 1].name);
      }
      members = &nested(*member);
      continue;
    }
    if (is_message) {
      throw std::invalid_argument(
              "'" + so_far + "' is a message (" + type_name(nested(*member)) +
              "); map one of its fields: " + field_list(nested(*member)));
    }
    const auto kind = kind_of_type(member->type_id_);
    if (!kind) {
      throw std::invalid_argument("'" + so_far + "' has an unsupported type (wide strings)");
    }
    if (whole_array) {
      if (!is_numeric(*kind)) {
        throw std::invalid_argument(
                "'" + so_far + "' is a non-numeric array; select an element, e.g. " + so_far +
                "[0]");
      }
      acc.kind_ = ValueKind::kFloatArray;
    } else {
      acc.kind_ = *kind;
    }
  }
  return acc;
}

std::optional<Value> FieldAccessor::read(const void * message) const
{
  const auto * base = static_cast<const std::uint8_t *>(message);
  for (std::size_t i = 0; i < steps_.size(); ++i) {
    const MessageMember & m = *steps_[i].member;
    const void * field = base + m.offset_;
    const bool last = i + 1 == steps_.size();
    if (steps_[i].index) {
      const std::size_t index = *steps_[i].index;
      if (index >= m.size_function(field)) {
        return std::nullopt;
      }
      if (!last) {
        base = static_cast<const std::uint8_t *>(m.get_const_function(field, index));
        continue;
      }
      return read_element(m, field, index);
    }
    if (!last) {
      base = static_cast<const std::uint8_t *>(field);
      continue;
    }
    if (m.is_array_) {
      const std::size_t n = m.size_function(field);
      std::vector<double> out(n);
      for (std::size_t k = 0; k < n; ++k) {
        out[k] = to_double(read_element(m, field, k));
      }
      return out;
    }
    return read_scalar(m.type_id_, field);
  }
  return std::nullopt;
}

}  // namespace simport
