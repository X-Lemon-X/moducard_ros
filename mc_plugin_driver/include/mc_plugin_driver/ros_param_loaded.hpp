#pragma once

#include <string>
#include "mc_plugin_base/status.hpp"
#include "mc_plugin_base/can_base.hpp"
#include "mc_plugin_driver/mc_param.hpp"
#include "ctype.h"
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

namespace mcan {

class McDriverParamsLoader {
public:
  McDriverParamsLoader() = default;

  Status load_params(std::string param_file_path) {
    try {
      YAML::Node config = YAML::LoadFile(param_file_path);

      bool load_modules = true;
      bool load_drivers = true;
      if(!config["drivers"] && !config["modules"]) {
        return Status::KeyError("No 'drivers' or 'modules' section found in YAML file");
      }

      if(!config["drivers"]) {
        load_drivers = false;
      }

      if(!config["modules"]) {
        load_modules = false;
      }

      if(!config["states_update_default_rate"]) {
        return Status::KeyError("No 'states_update_default_rate' specified in global YAML file");
      }

      float global_default_rate = config["states_update_default_rate"].as<float>();
      if(global_default_rate < 1.0f) {
        return Status::Invalid("Global 'states_update_default_rate' must be at lest 1Hz");
      }
      _global_default_rate = global_default_rate;
      update_max_rate(global_default_rate);


      if(!config["can_primary_interface"]) {
        return Status::KeyError("No 'can_primary_interface' specified in YAML file");
      }
      _can_primary_interface = config["can_primary_interface"].as<std::string>();

      if(!config["can_secondary_interface"]) {
        return Status::KeyError("No 'can_secondary_interface' specified in YAML file");
      }
      _can_secondary_interface = config["can_secondary_interface"].as<std::string>();

      if(!config["frame_id"]) {
        return Status::KeyError("No global 'frame_id' specified in YAML file");
      }
      std::string global_frame_id = config["frame_id"].as<std::string>();

      if(load_modules) {
        // Iterate through the modules map (each key is the module_ros_name)
        for(const auto &module_entry : config["modules"]) {
          ModuleParams params;

          // The key is the module_ros_name (e.g., "motor_front_left")
          params.module_ros_name = module_entry.first.as<std::string>();

          const YAML::Node &module_data = module_entry.second;

          // Load driver (maps to module_name)
          if(module_data["driver"]) {
            params.module_name = module_data["driver"].as<std::string>();
          } else {
            return Status::KeyError("No 'driver' specified for module " + params.module_ros_name);
          }

          if(module_data["frame_id"]) {
            params.frame_id = module_data["frame_id"].as<std::string>();
          } else {
            // Fall back to global frame_id if not specified per module
            params.frame_id = global_frame_id;
          }

          // Load unique_id (note: hex format in YAML)
          if(module_data["unique_id"]) {
            params.unique_id = module_data["unique_id"].as<uint32_t>();
          } else {
            return Status::KeyError("No 'unique_id' specified for module " + params.module_ros_name);
          }

          // Load node_id
          if(module_data["node_id"]) {
            params.node_id = module_data["node_id"].as<uint8_t>();
          } else {
            ARI_ASIGN_OR_RETURN(node_id, get_new_node_id());
            params.node_id = node_id;
          }

          // Load states_update_default_rate
          if(module_data["states_update_default_rate"]) {
            float rate = module_data["states_update_default_rate"].as<float>();
            if(rate < 1.0f) {
              return Status::Invalid("Module '" + params.module_ros_name + "' has 'states_update_default_rate' less than 1Hz, which is not allowed");
            }
            params.states_update_default_rate = rate;
          } else if(config["states_update_default_rate"]) {
            // Fall back to global default if not specified per module
            params.states_update_default_rate = global_default_rate;
          }
          update_max_rate(params.states_update_default_rate);

          // Load states_custom_rates (stored as map in YAML)
          if(module_data["states_custom_rates"]) {
            for(const auto &rate_entry : module_data["states_custom_rates"]) {
              std::string state_name = rate_entry.first.as<std::string>();
              float rate             = rate_entry.second.as<float>();
              if(rate <= 0.0f) {
                return Status::Invalid("Module '" + params.module_ros_name + "' has custom rate for state '" +
                                       state_name + "' that is not positive number");
              }
              update_max_rate(rate);
              params.states_custom_rates.push_back({ state_name, rate });
            }
          }

          // Load parameters (stored as map in YAML)
          if(module_data["parameters"]) {
            for(const auto &param_entry : module_data["parameters"]) {
              std::string param_name  = param_entry.first.as<std::string>();
              std::string param_value = param_entry.second.as<std::string>();
              params.parameters_default_values.push_back({ param_name, param_value });
            }
          }

          // Store the module params using module_ros_name as key
          _module_params[params.module_ros_name] = params;
        }
      }

      if(load_drivers) {
        for(const auto &driver_entry : config["drivers"]) {
          DriverParams params;
          params.name = driver_entry.first.as<std::string>();

          const YAML::Node &driver_data = driver_entry.second;

          if(driver_data["ros_package"]) {
            params.ros_package = driver_data["ros_package"].as<std::string>();
          } else {
            return Status::KeyError("No 'ros_package' specified for driver " + params.name);
          }

          // Store the driver params using driver name as key
          _driver_params[params.name] = params;
        }
      }

      return Status::OK();

    } catch(const YAML::Exception &e) {
      return Status::ExpressionValidationError("YAML parsing error: " + std::string(e.what()));
    } catch(const std::exception &e) {
      return Status::ExpressionValidationError("Error loading params: " + std::string(e.what()));
    }
  }

  const std::unordered_map<std::string, ModuleParams> &get_module_params() {
    return _module_params;
  }

  const std::unordered_map<std::string, DriverParams> &get_driver_params() {
    return _driver_params;
  }

  std::string get_can_primary_interface_name() const {
    return _can_primary_interface;
  }

  std::string get_can_secondary_interface_name() const {
    return _can_secondary_interface;
  }

  float get_global_default_rate() const {
    return _global_default_rate;
  }

  float get_max_update_rate() const {
    return _max_update_rate;
  }

private:
  void update_max_rate(float new_rate) {
    if(new_rate > _max_update_rate) {
      _max_update_rate = new_rate;
    }
  }

  Result<uint8_t> get_new_node_id() {
    std::unordered_set<uint8_t> used_node_ids;

    // Collect all already assigned node_ids from modules
    for(const auto &entry : _module_params) {
      if(entry.second.node_id != 0) { // Assuming 0 means unassigned
        used_node_ids.insert(entry.second.node_id);
      }
    }

    for(uint8_t candidate_id = 1; candidate_id < 255; ++candidate_id) {
      if(used_node_ids.find(candidate_id) == used_node_ids.end()) {
        return Result<uint8_t>::OK(std::move(candidate_id));
      }
    }

    // If all IDs are taken (very unlikely), return 0 to indicate error
    return Status::CapacityError("All node IDs are already assigned");
  }

  std::unordered_map<std::string, ModuleParams> _module_params;
  std::unordered_map<std::string, DriverParams> _driver_params;
  std::string _can_primary_interface;
  std::string _can_secondary_interface;
  float _global_default_rate;
  float _max_update_rate;
};

} // namespace mcan