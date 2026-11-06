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

// simport_check: validate a device YAML and show what the device will put on the wire,
// without opening the port or needing a running simulation.
//
//   ros2 run simport_ros simport_check <device.yaml>

#include <chrono>
#include <exception>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "simport_core/device_config.hpp"
#include "simport_ros/device.hpp"

namespace
{

using simport::Bytes;

std::string hexdump(const Bytes & data)
{
  std::string out;
  for (std::size_t row = 0; row < data.size(); row += 16) {
    std::string hex;
    std::string ascii;
    for (std::size_t i = row; i < row + 16; ++i) {
      if (i < data.size()) {
        hex += std::format("{:02x} ", data[i]);
        ascii += (data[i] >= 0x20 && data[i] < 0x7f) ? static_cast<char>(data[i]) : '.';
      } else {
        hex += "   ";
      }
    }
    out += std::format("      {:04x}  {} |{}|\n", row, hex, ascii);
  }
  return out;
}

std::string source_of(const simport::MappingSpec & m, std::size_t field)
{
  const auto describe = [](const simport::FieldMapping & f) {
      std::string s;
      switch (f.from) {
        case simport::FieldMapping::From::kPath: s = f.path; break;
        case simport::FieldMapping::From::kBuiltin: s = std::string(simport::to_string(f.builtin));
          break;
      }
      const simport::ValueTransform & t = f.transform;
      if (t.transpose_rows) {s += std::format(" transposed ({} rows)", *t.transpose_rows);}
      if (t.reverse) {s += " reversed";}
      if (t.step != 1) {s += std::format(" every {}", t.step);}
      if (t.resize) {s += std::format(" resized to {}", *t.resize);}
      if (t.scale) {s += std::format(" * {}", *t.scale);}
      if (t.offset) {s += std::format(" + {}", *t.offset);}
      if (t.nonfinite) {s += std::format(" (inf/nan -> {})", *t.nonfinite);}
      if (t.ramp) {s += std::format(" ramp {} x {}", t.ramp->count, t.ramp->step);}
      return s;
    };
  for (const auto * list : {&m.paths(), &m.builtins()}) {
    for (const auto & f : *list) {
      if (f.target == field) {
        return describe(f);
      }
    }
  }
  return "";
}

void print_fields(const simport::Schema & schema, const simport::MappingSpec * mapping)
{
  for (std::size_t i = 0; i < schema.size(); ++i) {
    const auto & f = schema.field(i);
    std::string from;
    if (f.role == simport::FieldRole::kComputed) {
      from = "(computed)";
    } else if (mapping != nullptr) {
      from = source_of(*mapping, i);
      if (from.empty()) {
        from = mapping->paths().empty() && mapping->builtins().empty() ?
          "(from the service response)" : "(no 'from': always 0)";
      }
    }
    std::cout << std::format("    {:<16} {:<7} {}\n", f.name, simport::to_string(f.kind), from);
  }
}

void print_device(const simport::Device & device)
{
  std::cout << "device   " << device.name() << "\n";
  std::cout << "port     " << device.transport()->describe() << "\n";
  const auto caps = device.transport()->caps();
  if (caps.line_rate_bps) {
    std::cout << std::format(
      "line     {} bit/s, {} bits per byte -> {:.0f} bytes/s\n", *caps.line_rate_bps,
      caps.bits_per_byte, *caps.line_rate_bps / caps.bits_per_byte);
  }
  if (device.framer()) {
    std::cout << "framer   splits the received byte stream into packets\n";
  }

  for (const auto & o : device.sends()) {
    std::cout << "\nsend " << o.spec.name << "\n";
    std::cout << "  from     " << (o.source ? o.source->describe() : "(replies only)");
    if (o.source && !o.spec.stream) {
      std::cout << ", latest sent as replies only";
    } else if (o.spec.rate) {
      std::cout << std::format(", at {} Hz", *o.spec.rate);
    }
    std::cout << "\n  fields\n";
    const auto & mapping = o.channel->mapping();
    const auto & schema = *mapping.schema();
    print_fields(schema, &mapping);
    // Sample packet from constants and builtins (message fields read as 0; message arrays as
    // two zeros).
    simport::Record r = mapping.make_record();
    for (const auto & m : mapping.paths()) {
      if (schema.field(m.target).kind == simport::ValueKind::kFloatArray) {
        r[m.target] = std::vector<double>(2, 0.0);
      }
    }
    const auto now = std::chrono::duration_cast<simport::Timestamp>(
      std::chrono::system_clock::now().time_since_epoch());
    mapping.apply_builtins(r, simport::BuiltinInputs{now, now, now});
    std::vector<simport::Frame> frames;
    try {
      o.channel->codec()->encode(r, frames);
      constexpr std::size_t kMaxShown = 3;
      for (std::size_t n = 0; n < frames.size() && n < kMaxShown; ++n) {
        std::cout << std::format(
          "  sample packet ({} bytes; values as 0)\n", frames[n].data.size());
        std::cout << hexdump(frames[n].data);
      }
      if (frames.size() > kMaxShown) {
        std::cout << std::format("  ... {} packets for this record\n", frames.size());
      } else if (frames.empty()) {
        std::cout << "  sample: no packets (the record's arrays are empty)\n";
      }
    } catch (const std::exception & e) {
      std::cout << "  sample packet: encode failed: " << e.what() << "\n";
    }
  }

  for (const auto & r : device.receives()) {
    std::cout << "\nreceive " << r->spec.name << "\n";
    for (const auto & [field, value] : r->spec.match.entries()) {
      std::string values;
      for (const auto & v :
        value.is_sequence() ? value.items() : std::vector<simport::Config>{value})
      {
        values += (values.empty() ? "" : " or ") + v.scalar();
      }
      std::cout << "  match    " << field << " == " << values << "\n";
    }
    if (r->spec.target == simport::ReceiveSpec::Target::kBehavior) {
      std::cout << "  to       the behavior (" << device.behavior()->describe() << ")\n";
    } else if (r->spec.target == simport::ReceiveSpec::Target::kService) {
      std::cout << "  to       service " << r->spec.to;
      for (std::size_t i = 0; i < r->spec.replies.size(); ++i) {
        std::cout << (i == 0 ? ", replies sent as " : " or ") << r->spec.replies[i];
      }
      std::cout << "\n";
    } else {
      std::cout << "  to       topic " << r->spec.to << " (simport_msgs/Record)\n";
    }
    std::cout << "  fields\n";
    print_fields(*r->channel->codec()->schema(), nullptr);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    std::cerr << "usage: simport_check <device.yaml>\n\n"
      "Validates a simport device file (including every plugin's parameters and every\n"
      "message field mapping) and prints the resulting wire layout, without opening the\n"
      "port or needing a simulator.\n";
    return argc == 2 ? 0 : 2;
  }
  rclcpp::init(1, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  int status = 0;
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("simport_check");
    auto logger = std::make_shared<simport::StderrLogger>(simport::LogLevel::kWarn);
    try {
      const auto cfg = simport::DeviceConfig::load(argv[1]);
      simport::Device device(*node, logger);
      device.build(cfg, /*open_transport=*/ false);
      print_device(device);
      std::cout << "\nOK: " << argv[1] << " is valid\n";
    } catch (const std::exception & e) {
      std::cerr << "error: " << e.what() << "\n";
      status = 1;
    }
  }
  rclcpp::shutdown();
  return status;
}
