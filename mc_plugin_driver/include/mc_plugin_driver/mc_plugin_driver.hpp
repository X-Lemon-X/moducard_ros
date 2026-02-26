#pragma once
#include "mc_plugin_base/mc_firmware/can_base.hpp"
#include "mc_plugin_base/mc_firmware/mc_common.hpp"
#include "mc_plugin_base/mc_firmware/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include <dlfcn.h>
#include <functional>
#include <unordered_map>

// #include "mc_plugin_driver.hpp"
#include "mc_plugin_driver/mc_plugin_exporter.hpp"
#include "mc_plugin_driver/mc_slave_driver.hpp"
#include "mc_plugin_driver/ros_param_loaded.hpp"
#include "mc_plugin_driver/timing.hpp"
#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy.hpp"
#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp"
#include <pluginlib/class_loader.hpp>

namespace mcan {

struct PluginInterface
{
  using CreatePluginFunc = McSlavePluginDriverBase* (*)(rclcpp::Node&,
                                                        std::shared_ptr<CanBase>,
                                                        std::shared_ptr<CanBase>,
                                                        const ModuleParams&);
  using GetPluginIdFunc = uint64_t (*)();
  CreatePluginFunc create_fn;
  void* handle;
  uint64_t unique_id;
};

static const uint8_t MASTER_DRIVER_NODE_ID = 0x01;

enum class McPLuginDriverStates : uint8_t
{
  UNINITIALIZED,
  INITIALIZED,
  ENTER_CONFIG_MODE,   // all deivecv enter config mode
  DISCOVER_DEVICES,    // dicover all devices on network
  CHECK_COMPATIBILITY, // check if each deviec on a network have a drive if not display
                       // that
  LOAD_PLUGINS, // load all plugins according to user settings and compatibility check, if
                // errors occur during loading, display that and skip loading that plugin
  INIT_MODULES, // set node id for each module according to config and call driver_on_init
                // for each driver, check the with ping
  CONTROL_LOOP, // call driver_control_loop for each driver in a loop and monitor their
                // status, if any driver enters error state, display that and try to
                // recover from it
  ERROR_MODE,   // if any error occurs during any of the above steps, switch to this mode
                // and display appropriate error message and try to recover from it
};

class McBasicModuleDriver
{
 public:
  McBasicModuleDriver(rclcpp::Node& node,
                      std::shared_ptr<CanBase> can_primary,
                      std::shared_ptr<CanBase> can_secondary,
                      uint8_t node_id,
                      uint32_t unique_id,
                      const DeviceIdentifier_t& device_identifier);

  ~McBasicModuleDriver();

  Status start_driver();

  Status request_new_node_id(uint8_t new_node_id);

  Result<mcan_base_module_dummy::configs::GetHardwareType::Type>
  request_get_hardware_type();

  Result<mcan_base_module_dummy::configs::PingModule::Type> request_ping_module();

  Status enable_led_flash(bool enable);

  Status set_plugin_driver(std::shared_ptr<McPluginExporterBase> plugin_driver);

  Status load_plugin(const ModuleParams& module_params);

  McSlavePluginDriverBase* get_driver() { return _loaded_plugin_driver.get(); }

  HardwareType_t get_hardware_type() { return _hardware_type; }

  uint64_t get_plugin_identifier() { return _plugin_unique_id; }

  uint32_t get_unique_id() { return _unique_id; }

  Status control_loop()
  {
    if (_loaded_plugin_driver) {
      return _loaded_plugin_driver->driver_control_call();
    }
    return Status::OK();
  }

  bool is_device_supported();

 private:
  void callback_get_hardware_type(CanBase& can, const CanFrame& frame, void* args);
  void callback_ping_module(CanBase& can, const CanFrame& frame, void* args);

  Status stop_driver();
  bool started = false;
  rclcpp::Node& _node;
  std::shared_ptr<CanBase> _can_interface;
  std::shared_ptr<CanBase> _can_secondary;
  uint8_t _node_id;
  uint32_t _unique_id;
  ModuleParams _module_params;
  std::shared_ptr<McSlavePluginDriverBase> _loaded_plugin_driver = nullptr;
  HardwareType_t _hardware_type;
  uint64_t _plugin_unique_id;
  std::shared_ptr<mcan::McPluginExporterBase> _plugin_exporter;
};

class McPLuginDriver : public rclcpp::Node
{
 public:
  McPLuginDriver();
  ~McPLuginDriver();

 private:
  void main_loop();

  bool execute_state();
  void switch_to_state(McPLuginDriverStates next_state);

  Status load_config(const std::string& config_file);

  Status load_available_plugins();
  Result<std::shared_ptr<PluginInterface>> load_plugin(const DriverParams& driver_params);

  Status enter_configuration_mode();
  Status discover_devices();
  Status check_compatibility();
  Status load_plugins_state();
  Status init_modules();
  Status control_loop();
  Status exit_control_loop();

  Status error_mode();

  Status enter_uninitialized_state();
  Status enter_initialized_state();

  void callback_discover_response(CanBase& can, const CanFrame& frame, void* args);
  std::mutex _mutex_device_updates;

  using func_ptr_t = std::function<void()>;
  using func_ptr_status_t = std::function<Status()>;

  std::shared_ptr<CanBase> _can_interface_primary;
  std::shared_ptr<CanBase> _can_interface_secondary;
  McDriverParamsLoader _param_loader;
  std::vector<std::shared_ptr<McBasicModuleDriver>> _connected_devices;
  std::vector<std::shared_ptr<McBasicModuleDriver>> _active_slave_drivers;
  std::unordered_map<uint64_t, std::shared_ptr<mcan::McPluginExporterBase>>
    _loaded_plugins;
  pluginlib::ClassLoader<mcan::McPluginExporterBase> _plugin_loader;

  func_ptr_t _func_main_loop = nullptr;
  func_ptr_status_t _func_exit_prev_state = nullptr;
  func_ptr_status_t _func_enter_next_state = nullptr;
  McPLuginDriverStates _state = McPLuginDriverStates::UNINITIALIZED;
  McPLuginDriverStates _next_state = McPLuginDriverStates::UNINITIALIZED;
  std::thread _main_loop_thread;
  std::atomic<bool> _run_main_loop{ false };
  std::atomic<bool> _switch_state{ false };
};

} // namespace mcan