#pragma once
#include "rclcpp/rclcpp.hpp"
#include "mc_plugin_base/can_base.hpp"
#include "mc_plugin_base/status.hpp"
#include "mc_plugin_base/mc_common.hpp"
#include <dlfcn.h>
#include <unordered_map>
#include <functional>

// #include "mc_plugin_driver.hpp"
#include "mc_plugin_driver/timing.hpp"
#include "mc_plugin_driver/mc_slave_driver.hpp"
#include "mc_plugin_driver/mc_plugin_exporter.hpp"
#include "mc_plugin_driver/ros_param_loaded.hpp"
#include <pluginlib/class_loader.hpp>

namespace mcan {

struct PluginInterface {
  using CreatePluginFunc = McSlavePluginDriverBase *(*)(rclcpp::Node &,
                                                        std::shared_ptr<CanBase>,
                                                        std::shared_ptr<CanBase>,
                                                        const ModuleParams &);
  using GetPluginIdFunc  = uint64_t (*)();
  CreatePluginFunc create_fn;
  void *handle;
  uint64_t unique_id;
};

enum class McPLuginDriverStates : uint8_t {
  NONE,
  UNINITIALIZED,
  ENTER_CONFIG_MODE, // all deivecv enter config mode
  DISCOVER_DEVICES, // dicover all devices on network
  CHECK_COMPATIBILITY, // check if each deviec on a network have a drive if not display that  
  LOAD_PLUGINS, // load all plugins according to user settings and compatibility check, if errors occur during loading, display that and skip loading that plugin
  INIT_MODULES, // set node id for each module according to config and call driver_on_init for each driver, check the with ping
  CONTROL_LOOP, // call driver_control_loop for each driver in a loop and monitor their status, if any driver enters error state, display that and try to recover from it
  ERROR_MODE, // if any error occurs during any of the above steps, switch to this mode and display appropriate error message and try to recover from it
};


class McPLuginDriver : public rclcpp::Node {
public:
  McPLuginDriver();
  ~McPLuginDriver();

private:
  void main_loop();

  void stop_main_loop();
  void start_main_loop();

  Status switch_state(McPLuginDriverStates next_state);


  Status load_config(const std::string &config_file);

  Result<std::shared_ptr<McPluginExporterBase>> load_plugin_creator(const DriverParams &driver_params);

  Status load_plugins();
  Result<std::shared_ptr<PluginInterface>> load_plugin(const DriverParams &driver_params);


  Result<std::shared_ptr<McSlavePluginDriverBase>> load_create_slave_driver(const ModuleParams &mo_duleparams);

  Status enter_configuration_mode();
  Status discover_devices();
  Status check_compatibility();
  Status load_plugins_state();
  Status init_modules();
  Status control_loop();
  Status error_mode();

  using func_ptr_t        = std::function<void()>;
  using func_ptr_status_t = std::function<Status()>;


  std::shared_ptr<CanBase> _can_interface_primary;
  std::shared_ptr<CanBase> _can_interface_secondary;
  McDriverParamsLoader _param_loader;
  std::vector<std::shared_ptr<McSlavePluginDriverBase>> _slave_drivers;
  std::vector<std::shared_ptr<McSlavePluginDriverBase>> _active_slave_drivers;
  std::unordered_map<std::string, std::shared_ptr<mcan::McPluginExporterBase>> _loaded_plugins; // plugin path -> dlopen handle
  pluginlib::ClassLoader<mcan::McPluginExporterBase> _plugin_loader;


  func_ptr_t _func_main_loop               = nullptr;
  func_ptr_status_t _func_exit_prev_state  = nullptr;
  func_ptr_status_t _func_enter_next_state = nullptr;
  McPLuginDriverStates state           = McPLuginDriverStates::NONE;
  std::thread _main_loop_thread;
  std::atomic<bool> _run_main_loop{ false };
  std::mutex state_mutex;
};


} // namespace mcan