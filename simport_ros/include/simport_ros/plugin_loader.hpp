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

#ifndef SIMPORT_ROS__PLUGIN_LOADER_HPP_
#define SIMPORT_ROS__PLUGIN_LOADER_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "simport_core/device_config.hpp"

namespace simport
{

/// pluginlib loader that reports failures as ConfigErrors located at the plugin reference.
/// Must outlive every instance it creates.
template<typename Base>
class PluginLoader
{
public:
  PluginLoader(const std::string & package, const std::string & base_class)
  : loader_(package, base_class) {}

  std::shared_ptr<Base> create(const PluginSpec & spec)
  {
    const Config where = spec.config.has("plugin") ? spec.config["plugin"] : spec.config;
    if (!loader_.isClassAvailable(spec.plugin)) {
      std::string available;
      for (const auto & name : loader_.getDeclaredClasses()) {
        available += " " + name;
      }
      where.error(
        "unknown " + loader_.getBaseClassType() + " plugin '" + spec.plugin + "'; available:" +
        (available.empty() ? " (none)" : available));
    }
    try {
      return loader_.createSharedInstance(spec.plugin);
    } catch (const pluginlib::PluginlibException & e) {
      where.error("cannot load plugin '" + spec.plugin + "': " + e.what());
    }
  }

private:
  pluginlib::ClassLoader<Base> loader_;
};

}  // namespace simport

#endif  // SIMPORT_ROS__PLUGIN_LOADER_HPP_
