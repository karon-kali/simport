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

#include "simport_ros/point_cloud2_source.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace simport
{

namespace
{

using sensor_msgs::msg::PointField;

template<typename T>
double load_as(const std::uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return static_cast<double>(v);
}

/// Reader for one point field of a cloud; nullopt for unsupported types.
std::optional<double (*)(const std::uint8_t *)> reader_for(std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8: return &load_as<std::int8_t>;
    case PointField::UINT8: return &load_as<std::uint8_t>;
    case PointField::INT16: return &load_as<std::int16_t>;
    case PointField::UINT16: return &load_as<std::uint16_t>;
    case PointField::INT32: return &load_as<std::int32_t>;
    case PointField::UINT32: return &load_as<std::uint32_t>;
    case PointField::FLOAT32: return &load_as<float>;
    case PointField::FLOAT64: return &load_as<double>;
    default: return std::nullopt;
  }
}

const PointField * find_field(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const PointField & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

}  // namespace

PointCloud2Source::~PointCloud2Source()
{
  stop();
}

void PointCloud2Source::configure(
  const Config & config, Context & context, rclcpp_lifecycle::LifecycleNode & node,
  const MappingSpec & mapping)
{
  config.expect_keys({"plugin", "topic", "type", "reliability", "depth", "stamp"});
  context_.emplace(context);
  node_ = &node;
  topic_ = config.get<std::string>("topic");
  if (topic_.empty()) {
    config["topic"].error("topic must not be empty");
  }
  qos_ = rclcpp::QoS(config.get<std::uint32_t>("depth", 2));
  const std::string reliability = config.get<std::string>("reliability", "best_effort");
  if (reliability == "best_effort") {
    qos_.best_effort();
  } else if (reliability == "reliable") {
    qos_.reliable();
  } else {
    config["reliability"].error("expected 'best_effort' or 'reliable'");
  }
  const std::string stamp = config.get<std::string>("stamp", "auto");
  if (stamp != "auto" && stamp != "header" && stamp != "receive") {
    config["stamp"].error("expected 'auto', 'header' or 'receive'");
  }
  header_stamp_ = stamp != "receive";

  const Schema & schema = *mapping.schema();
  bindings_.clear();
  for (const FieldMapping & m : mapping.paths()) {
    const Config where = m.config.is_map() ? m.config["from"] : m.config;
    if (m.path_elements.size() != 1 || m.path_elements[0].index) {
      where.error("expected a point field name (x, intensity, ...), range, azimuth, elevation, "
        "width or height; got '" + m.path + "'");
    }
    Binding b{m.target, Kind::kField, m.path, m.transform};
    if (m.path == "range") {
      b.kind = Kind::kRange;
    } else if (m.path == "azimuth") {
      b.kind = Kind::kAzimuth;
    } else if (m.path == "elevation") {
      b.kind = Kind::kElevation;
    } else if (m.path == "width") {
      b.kind = Kind::kWidth;
    } else if (m.path == "height") {
      b.kind = Kind::kHeight;
    }
    const bool scalar = b.kind == Kind::kWidth || b.kind == Kind::kHeight;
    const ValueKind target = schema.field(m.target).kind;
    if (!scalar && target != ValueKind::kFloatArray) {
      where.error("'" + m.path + "' is an array (one value per point), but codec field '" +
        m.target_name + "' is " + std::string(to_string(target)) + "; use a field in a group");
    }
    if (scalar && m.transform.array_ops()) {
      where.error("'" + m.path + "' is a number, so it cannot be rearranged as an array");
    }
    bindings_.push_back(std::move(b));
  }
  prototype_ = mapping.make_record();
}

void PointCloud2Source::start(RecordCallback callback)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
  }
  subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, qos_,
    [this](const sensor_msgs::msg::PointCloud2 & cloud) {on_cloud(cloud);});
}

void PointCloud2Source::stop()
{
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = nullptr;
}

void PointCloud2Source::on_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!callback_) {
    return;
  }
  const std::size_t n = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (cloud.is_bigendian || cloud.data.size() < n * cloud.point_step) {
    if (!reported_missing_.exchange(true)) {
      context_->logger().error("cannot read clouds on {}: big-endian or truncated data", topic_);
    }
    return;
  }
  const auto point = [&](std::size_t i) {
      const std::size_t row = i / cloud.width;
      const std::size_t col = i % cloud.width;
      return cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
    };
  // Readers for a field by name; x/y/z for the computed values.
  struct Reader
  {
    std::size_t offset;
    double (*read)(const std::uint8_t *);
  };
  const auto reader = [&](const std::string & name) -> std::optional<Reader> {
      const PointField * f = find_field(cloud, name);
      if (f == nullptr) {
        return std::nullopt;
      }
      const auto read = reader_for(f->datatype);
      if (!read) {
        return std::nullopt;
      }
      return Reader{f->offset, *read};
    };

  auto record = std::make_shared<Record>(prototype_);
  for (const Binding & b : bindings_) {
    Value value;
    if (b.kind == Kind::kWidth || b.kind == Kind::kHeight) {
      value = std::uint64_t{b.kind == Kind::kWidth ? cloud.width : cloud.height};
    } else {
      std::vector<double> out(n);
      if (b.kind == Kind::kField) {
        const auto r = reader(b.field);
        if (!r) {
          if (!reported_missing_.exchange(true)) {
            std::string names;
            for (const PointField & f : cloud.fields) {
              names += " " + f.name;
            }
            context_->logger().error(
              "clouds on {} have no readable field '{}' (fields:{}); it stays 0", topic_,
              b.field, names);
          }
          continue;
        }
        for (std::size_t i = 0; i < n; ++i) {
          out[i] = r->read(point(i) + r->offset);
        }
      } else {
        const auto x = reader("x");
        const auto y = reader("y");
        const auto z = reader("z");
        if (!x || !y || !z) {
          if (!reported_missing_.exchange(true)) {
            context_->logger().error("clouds on {} have no x, y and z fields", topic_);
          }
          continue;
        }
        for (std::size_t i = 0; i < n; ++i) {
          const std::uint8_t * p = point(i);
          const double px = x->read(p + x->offset);
          const double py = y->read(p + y->offset);
          const double pz = z->read(p + z->offset);
          switch (b.kind) {
            case Kind::kRange: out[i] = std::sqrt(px * px + py * py + pz * pz); break;
            case Kind::kAzimuth: out[i] = std::atan2(py, px); break;
            default: out[i] = std::atan2(pz, std::hypot(px, py)); break;
          }
        }
      }
      value = std::move(out);
    }
    try {
      (*record)[b.target] = b.transform.apply(value);
    } catch (const ValueError & e) {
      if (!reported_missing_.exchange(true)) {
        context_->logger().error("clouds on {}: {}", topic_, e.what());
      }
    }
  }
  record->stamp = header_stamp_ ?
    Timestamp(static_cast<std::int64_t>(cloud.header.stamp.sec) * 1'000'000'000 +
    cloud.header.stamp.nanosec) :
    context_->clock().now();
  callback_(std::move(record));
}

std::string PointCloud2Source::describe() const
{
  return topic_ + " [sensor_msgs/msg/PointCloud2]";
}

}  // namespace simport

PLUGINLIB_EXPORT_CLASS(simport::PointCloud2Source, simport::Source)
