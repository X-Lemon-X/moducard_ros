#pragma once

#include "ctype.h"
#include "mc_plugin_base/mc_firmware/can_base.hpp"
#include "mc_plugin_base/mc_firmware/status.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace mcan {

struct ModuleParams
{
  std::string module_ros_name;
  std::string module_name;
  std::string frame_id;
  uint32_t unique_id;
  uint8_t node_id;
  float states_update_default_rate;

  std::vector<std::pair<std::string, float>> states_custom_rates;
  std::vector<std::pair<std::string, std::string>> parameters_default_values;

  std::string to_string() const
  {
    std::string result = "ModuleParams:\n";
    result += "  module_ros_name: " + module_ros_name + "\n";
    result += "  module_name: " + module_name + "\n";
    result += "  frame_id: " + frame_id + "\n";
    result += "  unicie_id: " + std::to_string(unique_id) + "\n";
    result += "  node_id: " + std::to_string(node_id) + "\n";
    result +=
      "  states_update_default_rate: " + std::to_string(states_update_default_rate) +
      "\n";

    result += "  states_custom_rates:\n";
    for (const auto& entry : states_custom_rates) {
      result += "    - state_name: " + entry.first +
                ", rate: " + std::to_string(entry.second) + "\n";
    }

    result += "  parameters_default_values:\n";
    for (const auto& entry : parameters_default_values) {
      result +=
        "    - param_name: " + entry.first + ", default_value: " + entry.second + "\n";
    }

    return result;
  }
};

struct DriverParams
{
  std::string name;
  std::string ros_package;
};

} // namespace mcan