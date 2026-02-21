#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mc_plugin_driver/mc_plugin_driver.hpp"
#include "mc_can_driver/can_linux_driver.hpp"
#include <pluginlib/class_loader.hpp>

using namespace std::chrono_literals;
using namespace mcan;


McPLuginDriver::McPLuginDriver()
: Node("McPLuginDriver"), _plugin_loader("mc_plugin_driver", "mcan::McPluginExporterBase") {
  this->declare_parameter("config_file", "");
  auto config_file = this->get_parameter("config_file").as_string();


  auto load_plugins_res = _plugin_loader.getDeclaredClasses();
  if(load_plugins_res.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No plugins found in pluginlib loader");
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Found %zu plugins for boards:", load_plugins_res.size());
    for(const auto &plugin : load_plugins_res) {
      RCLCPP_INFO(this->get_logger(), "  - %s", plugin.c_str());
    }
  }


  if(config_file.empty()) {
    RCLCPP_WARN(this->get_logger(), "No config file specified");
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());
  }

  auto load_res = load_config(config_file);
  if(!load_res.ok()) {
    RCLCPP_ERROR(this->get_logger(), "%s", load_res.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Config loaded successfully");
  }

  load_plugins();

  auto maybe_can_primary = CanDriver::Make(_param_loader.get_can_primary_interface_name(), 100000, 32, 512);
  if(!maybe_can_primary.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize primary CAN interface: %s",
                 maybe_can_primary.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  }
  _can_interface_primary = maybe_can_primary.valueOrDie();
  auto maybe_can_secondary = CanDriver::Make(_param_loader.get_can_secondary_interface_name(), 100000, 32, 512);
  if(!maybe_can_secondary.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize secondary CAN interface: %s",
                 maybe_can_secondary.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  }
  _can_interface_secondary = maybe_can_secondary.valueOrDie();
  RCLCPP_INFO(this->get_logger(), "CAN interfaces initialized successfully");

  _run_main_loop.store(true);
  _main_loop_thread = std::thread(&McPLuginDriver::main_loop, this);

  RCLCPP_INFO(this->get_logger(), "McPLuginDriver Initialized");
}

McPLuginDriver::~McPLuginDriver() {
  // Clean up all plugin handles
  _run_main_loop.store(false);
  if(_main_loop_thread.joinable()) {
    _main_loop_thread.join();
  }
  RCLCPP_INFO(this->get_logger(), "McPLuginDriver shutdown complete");
}


void McPLuginDriver::main_loop() {
  while(_run_main_loop.load()) {
    if(_func_main_loop) {
      std::lock_guard<std::mutex> lock(state_mutex);
      _func_main_loop();
    }
  }
}

Status McPLuginDriver::switch_state(McSlavePluginDriverState next_state) {
  std::lock_guard<std::mutex> lock(state_mutex);

  if(state == next_state) {
    RCLCPP_WARN(this->get_logger(), "Already in state %d, ignoring switch request", static_cast<int>(next_state));
    return Status::OK();
  }


  // Call exit function for current state
  if(_func_exit_prev_state) {
    auto exit_status = _func_exit_prev_state();
    if(!exit_status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to exit state %d: %s", static_cast<int>(state),
                   exit_status.to_string().c_str());
      return Status::Invalid("Failed to exit state " + std::to_string(static_cast<int>(state)) + ": " +
                             exit_status.to_string());
    }
  }


  switch(next_state) {
  case McSlavePluginDriverState::UNINITIALIZED:
    _func_main_loop        = nullptr;
    _func_exit_prev_state  = nullptr;
    _func_enter_next_state = nullptr;
    break;
  case McSlavePluginDriverState::INITIALIZED: break;
  default: return Status::Invalid("Unknown state " + std::to_string(static_cast<int>(next_state)));
  }


  // Call enter function for next state
  if(_func_enter_next_state) {
    auto enter_status = _func_enter_next_state();
    if(!enter_status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enter state %d: %s", static_cast<int>(next_state),
                   enter_status.to_string().c_str());
      return Status::Invalid("Failed to enter state " + std::to_string(static_cast<int>(next_state)) + ": " +
                             enter_status.to_string());
    }
  }
  return Status::OK();
}


Result<std::shared_ptr<McPluginExporterBase>> McPLuginDriver::load_plugin_creator(const DriverParams &driver_params) {
  auto plugin_res = _plugin_loader.createSharedInstance(driver_params.ros_package);

  if(!plugin_res) {
    return Status::Invalid("Failed to load plugin creator for driver " + driver_params.name);
  }
  return Result<std::shared_ptr<McPluginExporterBase>>::OK(std::move(plugin_res));
}

Status McPLuginDriver::load_config(const std::string &config_file) {
  auto module_params_res = _param_loader.load_params(config_file);
  if(!module_params_res.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load module parameters: %s",
                 module_params_res.status().to_string().c_str());
    return Status::Invalid("Failed to load module parameters: " + module_params_res.status().to_string());
  }

  auto module_params = _param_loader.get_module_params();
  for(const auto &[module_name, params] : module_params) {
    RCLCPP_INFO(this->get_logger(), "Loaded config for module: %s\n%s", module_name.c_str(),
                params.to_string().c_str());
  }
  return Status::OK();
}

Status McPLuginDriver::load_plugins() {
  for(auto &[plugin_name, driver_params] : _param_loader.get_driver_params()) {
    ARI_ASIGN_OR_RETURN(plugin_creator, load_plugin_creator(driver_params));
    _loaded_plugins[driver_params.name] = std::move(plugin_creator);
  }
  return Status::OK();
}


Result<std::shared_ptr<McSlavePluginDriverBase>> McPLuginDriver::load_create_slave_driver(const ModuleParams &module_params) {
  auto plugin_it = _loaded_plugins.find(module_params.module_name);
  if(plugin_it == _loaded_plugins.end()) {
    RCLCPP_ERROR(this->get_logger(), "Plugin for module %s could't be found", module_params.module_ros_name.c_str());
    return Status::Invalid("Plugin for module " + module_params.module_ros_name + " could't be found");
  }
  auto slave_driver_ptr =
  plugin_it->second->create_new_instance(*this, _can_interface_primary, _can_interface_secondary, module_params);
  if(!slave_driver_ptr.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create slave driver for module %s using plugin %s, error: %s",
                 module_params.module_ros_name.c_str(), module_params.module_name.c_str(),
                 slave_driver_ptr.status().to_string().c_str());
    return Status::Invalid("Failed to create slave driver for module " + module_params.module_ros_name + " using plugin " +
                           module_params.module_name + ", error: " + slave_driver_ptr.status().to_string());
  }
  return Result<std::shared_ptr<McSlavePluginDriverBase>>::OK(
  std::shared_ptr<McSlavePluginDriverBase>(slave_driver_ptr.valueOrDie()));
}