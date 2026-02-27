#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mc_can_driver/can_linux_driver.hpp"
#include "mc_plugin_driver/mc_plugin_driver.hpp"
#include <pluginlib/class_loader.hpp>

#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy.hpp"
#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp"

using namespace std::chrono_literals;
using namespace mcan;

McBasicModuleDriver::McBasicModuleDriver(rclcpp::Node& node,
                                         std::shared_ptr<CanBase> can_primary,
                                         std::shared_ptr<CanBase> can_secondary,
                                         uint8_t node_id,
                                         uint32_t unique_id,
                                         const DeviceIdentifier_t& device_identifier)
  : _node(node)
  , _can_interface(std::move(can_primary))
  , _can_secondary(std::move(can_secondary))
  , _node_id(node_id)
  , _unique_id(unique_id)
  , _plugin_unique_id(device_identifier.unique_id)
{
}

Status
McBasicModuleDriver::start_driver()
{
  if (started) {
    return Status::OK("Driver already started");
  }

  ARI_RETURN_ON_ERROR(_can_interface->add_callback(
    mcan_connect_msg_id_with_node_id(
      mcan_base_module_dummy::configs::GetHardwareType::k_base_address, _node_id),
    std::bind(&McBasicModuleDriver::callback_get_hardware_type,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3),
    nullptr));
  ARI_RETURN_ON_ERROR(_can_interface->add_callback(
    mcan_connect_msg_id_with_node_id(
      mcan_base_module_dummy::configs::PingModule::k_base_address, _node_id),
    std::bind(&McBasicModuleDriver::callback_ping_module,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3),
    nullptr));

  started = true;
  return Status::OK();
}

Status
McBasicModuleDriver::stop_driver()
{
  if (!started) {
    return Status::OK("Driver already stopped");
  }
  started = false;
  ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
    mcan_base_module_dummy::configs::GetHardwareType::k_base_address, _node_id)));
  ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
    mcan_base_module_dummy::configs::PingModule::k_base_address, _node_id)));
  return Status::OK();
}

McBasicModuleDriver::~McBasicModuleDriver()
{
  (void)stop_driver();
}

Status
McBasicModuleDriver::request_new_node_id(uint8_t new_node_id)
{
  configs::SetDeviceNodeId msg;
  msg.value = new_node_id;
  CanFrame frame;
  frame.id = mcan_connect_msg_id_with_node_id(_unique_id, MASTER_DRIVER_NODE_ID);
  frame.size = sizeof(msg.value);
  frame.is_extended = true;
  frame.is_remote_request = false;
  std::memcpy(frame.data, &msg.value, sizeof(msg.value));
  ARI_RETURN_ON_ERROR(_can_interface->send(frame));
  _node_id = new_node_id;
  if (_loaded_plugin_driver) {
    _loaded_plugin_driver->driver_set_node_id(new_node_id);
  }
  return Status::OK();
}

Result<mcan_base_module_dummy::configs::GetHardwareType::Type>
McBasicModuleDriver::request_get_hardware_type()
{
  configs::GetHardwareType request;
  ARI_RETURN_ON_ERROR(mcan_request_and_await_msg(*_can_interface, request, _node_id));
  _hardware_type = request.value;
  return Result<mcan_base_module_dummy::configs::GetHardwareType::Type>::OK(
    std::move(request.value));
}

Result<mcan_base_module_dummy::configs::PingModule::Type>
McBasicModuleDriver::request_ping_module()
{
  configs::PingModule request;
  ARI_RETURN_ON_ERROR(mcan_request_and_await_msg(*_can_interface, request, _node_id));
  return Result<mcan_base_module_dummy::configs::PingModule::Type>::OK(
    std::move(request.value));
}

Status
McBasicModuleDriver::enable_led_flash(bool enable)
{
  mcan_base_module_dummy::configs::FlashIndicatorLed msg;
  msg.value = enable;
  return mcan_pack_send_msg(*_can_interface, msg, _node_id);
}

void
McBasicModuleDriver::callback_get_hardware_type(CanBase& can,
                                                const CanFrame& frame,
                                                void* args)
{
  (void)can;
  (void)args;
  CanMultiPackageFrame<configs::GetHardwareType> multi_frame;
  if (mcan_unpack_msg(frame, multi_frame).ok()) {
    _hardware_type = multi_frame.value;
    return;
  }
  RCLCPP_ERROR(_node.get_logger(), "Failed to unpack GetHardwareType response");
}

void
McBasicModuleDriver::callback_ping_module(CanBase& can, const CanFrame& frame, void* args)
{
  (void)can;
  (void)args;
  CanMultiPackageFrame<configs::PingModule> multi_frame;
  if (mcan_unpack_msg(frame, multi_frame).ok()) {
    RCLCPP_INFO(_node.get_logger(), "Received ping response: %d", multi_frame.value);
    return;
  }

  RCLCPP_ERROR(_node.get_logger(), "Failed to unpack PingModule response");
}

Status
McBasicModuleDriver::load_plugin(const ModuleParams& module_params)
{
  if (_plugin_exporter->get_plugin_unique_id() != _plugin_unique_id) {
    return Status::Invalid("Plugin unique ID does not match module's plugin unique ID");
  }
  if (_loaded_plugin_driver) {
    return Status::Invalid("Plugin already loaded for this module");
  }
  ARI_ASIGN_TO_OR_RETURN(_loaded_plugin_driver,
                         _plugin_exporter->create_new_instance(
                           _node, _can_interface, _can_secondary, module_params));
  _node_id = module_params.node_id;
  return Status::OK();
}

Status
McBasicModuleDriver::set_plugin_driver(
  std::shared_ptr<McPluginExporterBase> plugin_driver)
{
  if (!plugin_driver) {
    return Status::Invalid("Plugin driver is null");
  }
  _plugin_exporter = std::move(plugin_driver);
  return Status::OK();
}

bool
McBasicModuleDriver::is_device_supported()
{
  return _plugin_exporter != nullptr;
}

// McPLuginDriver implementation

McPLuginDriver::McPLuginDriver()
  : Node("McPLuginDriver")
  , _plugin_loader("mc_plugin_driver", "mcan::McPluginExporterBase")
{
  this->declare_parameter("config_file", "");
  auto config_file = this->get_parameter("config_file").as_string();

  auto load_plugins_res = _plugin_loader.getDeclaredClasses();
  if (load_plugins_res.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No plugins found in pluginlib loader");
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Found %zu plugins for boards:", load_plugins_res.size());
    for (const auto& plugin : load_plugins_res) {
      RCLCPP_INFO(this->get_logger(), "  - %s", plugin.c_str());
    }
  }

  if (config_file.empty()) {
    RCLCPP_WARN(this->get_logger(), "No config file specified");
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());
  }

  auto load_res = load_config(config_file);
  if (!load_res.ok()) {
    RCLCPP_ERROR(this->get_logger(), "%s", load_res.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Config loaded successfully");
  }

  load_available_plugins();

  auto maybe_can_primary =
    CanDriver::Make(_param_loader.get_can_primary_interface_name(), 100000, 32, 512);
  if (!maybe_can_primary.ok()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize primary CAN interface: %s",
                 maybe_can_primary.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  }
  _can_interface_primary = maybe_can_primary.valueOrDie();
  auto maybe_can_secondary =
    CanDriver::Make(_param_loader.get_can_secondary_interface_name(), 100000, 32, 512);
  if (!maybe_can_secondary.ok()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize secondary CAN interface: %s",
                 maybe_can_secondary.status().to_string().c_str());
    rclcpp::shutdown();
    return;
  }
  _can_interface_secondary = maybe_can_secondary.valueOrDie();
  RCLCPP_INFO(this->get_logger(), "CAN interfaces initialized successfully");

  RCLCPP_INFO(this->get_logger(), "McPLuginDriver Initialized");
  switch_to_state(McPLuginDriverStates::INITIALIZED);
  _run_main_loop.store(true);
  _main_loop_thread = std::thread(&McPLuginDriver::main_loop, this);
}

McPLuginDriver::~McPLuginDriver()
{
  send_request_to_enter_config_mode();
  _run_main_loop.store(false);
  if (_main_loop_thread.joinable()) {
    _main_loop_thread.join();
  }

  _active_slave_drivers.clear();
  _connected_devices.clear();
  _loaded_plugins.clear();

  RCLCPP_INFO(this->get_logger(), "McPLuginDriver shutdown complete");
}

void
McPLuginDriver::main_loop()
{
  double update_rate = _param_loader.get_max_update_rate() * 5.0f;
  std::chrono::nanoseconds update_period_ns(static_cast<int64_t>(1e9 / update_rate));
  RCLCPP_INFO(this->get_logger(),
              "Entering main loop with update rate: %f Hz, %i [ns]",
              update_rate,
              update_period_ns.count());

  while (_run_main_loop.load()) {
    if (execute_state() && _func_main_loop) {
      _func_main_loop();
    }
    std::this_thread::sleep_for(update_period_ns);
  }
}

void
McPLuginDriver::switch_to_state(McPLuginDriverStates next_state)
{
  if (_state == next_state) {
    RCLCPP_WARN(this->get_logger(),
                "Already in state %d, ignoring switch request",
                static_cast<int>(_next_state));
    return;
  }
  _next_state = next_state;
  _switch_state.store(true);
}

bool
McPLuginDriver::execute_state()
{
  if (!_switch_state.load()) {
    return true;
  }

  if (_func_exit_prev_state) {
    auto exit_status = _func_exit_prev_state();
    if (!exit_status.ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to exit state %d: %s",
                   static_cast<int>(_state),
                   exit_status.to_string().c_str());
      switch_to_state(McPLuginDriverStates::ERROR_MODE);
      return false;
    }
  }
  McPLuginDriverStates next_state = _next_state;
  _switch_state.store(false);

  switch (next_state) {
    case McPLuginDriverStates::UNINITIALIZED:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state =
        std::bind(&McPLuginDriver::enter_uninitialized_state, this);
      break;
    case McPLuginDriverStates::INITIALIZED:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::enter_initialized_state, this);
      break;
    case McPLuginDriverStates::ENTER_CONFIG_MODE:
      _func_main_loop =
        nullptr; // std::bind(&McPLuginDriver::enter_configuration_mode, this);
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::enter_configuration_mode, this);
      break;
    case McPLuginDriverStates::DISCOVER_DEVICES:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::discover_devices, this);
      break;
    case McPLuginDriverStates::CHECK_COMPATIBILITY:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::check_compatibility, this);
      break;
    case McPLuginDriverStates::LOAD_PLUGINS:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::load_plugins_state, this);
      break;
    case McPLuginDriverStates::INIT_MODULES:
      _func_main_loop = nullptr;
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = std::bind(&McPLuginDriver::init_modules, this);
      break;
    case McPLuginDriverStates::CONTROL_LOOP:
      _func_main_loop = std::bind(&McPLuginDriver::control_loop, this);
      _func_exit_prev_state = std::bind(&McPLuginDriver::exit_control_loop, this);
      _func_enter_next_state = nullptr;
      break;
    case McPLuginDriverStates::ERROR_MODE:
      _func_main_loop = std::bind(&McPLuginDriver::error_mode, this);
      _func_exit_prev_state = nullptr;
      _func_enter_next_state = nullptr;
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown state %d", static_cast<int>(next_state));
      return false;
  }

  // Call enter function for next state
  if (_func_enter_next_state) {
    auto enter_status = _func_enter_next_state();
    if (!enter_status.ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to enter state %d: %s",
                   static_cast<int>(next_state),
                   enter_status.to_string().c_str());
      switch_to_state(McPLuginDriverStates::ERROR_MODE);
      return false;
    }
    // When we enter a state and switch to another state during entering the state we
    // should not call the exit function of the previous state because we are not really
    // in that state yet, so we will just set the exit function to null if we detect that
    // we switched to another state during entering of the new state.
    if (next_state != _next_state) {
      _func_exit_prev_state = nullptr;
    }
  }
  return next_state == _next_state;
}

Status
McPLuginDriver::load_config(const std::string& config_file)
{
  auto module_params_res = _param_loader.load_params(config_file);
  if (!module_params_res.ok()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to load module parameters: %s",
                 module_params_res.status().to_string().c_str());
    return Status::Invalid("Failed to load module parameters: " +
                           module_params_res.status().to_string());
  }

  auto module_params = _param_loader.get_module_params();
  for (const auto& [module_name, params] : module_params) {
    RCLCPP_INFO(this->get_logger(),
                "Loaded config for module: %s\n%s",
                module_name.c_str(),
                params.to_string().c_str());
  }
  return Status::OK();
}

Status
McPLuginDriver::load_available_plugins()
{
  for (auto& [plugin_name, driver_params] : _param_loader.get_driver_params()) {
    auto plugin_res = _plugin_loader.createSharedInstance(driver_params.ros_package);
    if (!plugin_res) {
      return Status::Invalid("Failed to load plugin creator for driver " +
                             driver_params.name);
    }
    _loaded_plugins[plugin_res->get_plugin_unique_id()] = std::move(plugin_res);
  }
  return Status::OK();
}

void
McPLuginDriver::callback_discover_response(CanBase& can,
                                           const CanFrame& frame,
                                           void* args)
{
  (void)can;
  (void)args;
  CanMultiPackageFrame<mcan::mcan_base_module_dummy::configs::DiscoverDevices>
    response_buf;

  if (!mcan_unpack_msg(frame, response_buf).ok()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack can DiscoverDevices response");
    return;
  }
  std::lock_guard<std::mutex> lock(_mutex_device_updates);
  uint32_t unique_id = frame.id >> 8;

  auto it =
    std::find_if(_connected_devices.begin(),
                 _connected_devices.end(),
                 [&response_buf](const std::shared_ptr<McBasicModuleDriver>& driver) {
                   return driver->get_unique_id() == response_buf.value;
                 });

  if (it != _connected_devices.end()) {
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "\033[36mNew device discovered ID: 0x%08x PID: 0x%08lx\033[0m",
              unique_id,
              response_buf.value);

  DeviceIdentifier_t device_id;
  device_id.unique_id = response_buf.value;

  auto new_driver = std::make_shared<McBasicModuleDriver>(
    *this, _can_interface_primary, _can_interface_secondary, 0, unique_id, device_id);
  _connected_devices.emplace_back(std::move(new_driver));
}

Status
McPLuginDriver::enter_initialized_state()
{
  RCLCPP_INFO(this->get_logger(), "Entering Initialized state");

  // first we need to add callbacks for the messages that we will use in the configuration
  // process, like discover devices response, and enter config mode response, and also we
  // can add callback for ping response to check if devices are responding, but since we
  // are not sure if all devices will support ping we can make it optional by just adding
  // the callback and if some device does not support it we will just not receive any
  // response and we can handle that in the code by just checking if we received a
  // response or not after sending ping message.

  std::lock_guard<std::mutex> lock(_mutex_device_updates);
  // ndoe id is 8 bits the deviec wil repson on node id 0
  // and then wil have 21 btis frame id so wi have to receive them.
  uint32_t unique_id = 0;
  uint32_t mask = 0xFF;
  _connected_devices.clear();
  ARI_RETURN_ON_ERROR(_can_interface_primary->add_callback_masked(
    unique_id,
    mask,
    std::bind(&McPLuginDriver::callback_discover_response,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3),
    nullptr));

  // _can_interface_primary->add_callback_masked()
  ARI_RETURN_ON_ERROR(_can_interface_primary->open_can());
  ARI_RETURN_ON_ERROR(_can_interface_secondary->open_can());

  switch_to_state(McPLuginDriverStates::ENTER_CONFIG_MODE);
  return Status::OK();
}

Status
McPLuginDriver::enter_uninitialized_state()
{
  RCLCPP_INFO(this->get_logger(), "Entering uninitialized state");
  std::lock_guard<std::mutex> lock(_mutex_device_updates);
  _connected_devices.clear();
  ARI_RETURN_ON_ERROR(_can_interface_primary->remove_callback_masked(0, 0xFF));
  ARI_RETURN_ON_ERROR(_can_interface_primary->close_can());
  ARI_RETURN_ON_ERROR(_can_interface_secondary->close_can());

  return Status::OK();
}

Status
McPLuginDriver::send_request_to_enter_config_mode()
{
  mcan::mcan_base_module_dummy::configs::EnterConfigurationMode config_msg;
  config_msg.value = 1;
  return mcan_request_msg(*_can_interface_primary, config_msg, MASTER_DRIVER_NODE_ID);
}

Status
McPLuginDriver::enter_configuration_mode()
{
  RCLCPP_INFO(this->get_logger(), "Entering configuration mode");
  send_request_to_enter_config_mode();
  std::this_thread::sleep_for(750ms);
  // wait for devices to enter config mode
  switch_to_state(McPLuginDriverStates::DISCOVER_DEVICES);
  return Status::OK();
}

Status
McPLuginDriver::discover_devices()
{
  RCLCPP_INFO(this->get_logger(), "Discovering devices...");
  mcan::mcan_base_module_dummy::configs::DiscoverDevices discover_msg;
  // this will trigger multiple nodes to respond with their device info.
  mcan_request_msg(*_can_interface_primary, discover_msg, MASTER_DRIVER_NODE_ID);
  // so we will go to sleep for a moment to allow responses to be processed in the
  // callback and update the discovered devices list before we move to the next step which
  auto start_time = std::chrono::steady_clock::now();
  auto last_device_time = start_time;
  size_t last_device_count = 0;

  while (true) {
    std::this_thread::sleep_for(100ms);

    size_t current_count;
    {
      std::lock_guard<std::mutex> lock(_mutex_device_updates);
      current_count = _connected_devices.size();
    }

    if (current_count > last_device_count) {
      last_device_count = current_count;
      last_device_time = std::chrono::steady_clock::now();
    }

    auto elapsed = std::chrono::steady_clock::now() - last_device_time;
    if (elapsed > 1500ms) {
      RCLCPP_INFO(this->get_logger(),
                  "\033[32mDiscovery complete: %zu devices found\033[0m",
                  current_count);
      break;
    }
  }

  std::lock_guard<std::mutex> lock(_mutex_device_updates);
  for (const auto& device : _connected_devices) {
    RCLCPP_INFO(this->get_logger(),
                "Discovered device - Unique ID: 0x%08x, Plugin Unique ID: 0x%08lx",
                device->get_unique_id(),
                device->get_plugin_identifier());
  }

  if (last_device_count == 0) {
    RCLCPP_WARN(this->get_logger(), "No devices discovered");
    switch_to_state(McPLuginDriverStates::ENTER_CONFIG_MODE);
    return Status::OK("No devices discovered, but continuing anyway");
  }
  switch_to_state(McPLuginDriverStates::CHECK_COMPATIBILITY);
  return Status::OK();
}

Status
McPLuginDriver::check_compatibility()
{
  RCLCPP_INFO(this->get_logger(), "Checking compatibility");
  std::lock_guard<std::mutex> lock(_mutex_device_updates);
  bool all_compatible = true;
  for (auto& device : _connected_devices) {
    auto maybe_plugin = _loaded_plugins.find(device->get_plugin_identifier());
    if (maybe_plugin == _loaded_plugins.end()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "No plugin found for device ID: %x. This device requires plugin with PID: %x",
        device->get_unique_id(),
        device->get_plugin_identifier());
      all_compatible = false;
      continue;
    }
    device->set_plugin_driver(maybe_plugin->second);
  }
  switch_to_state(McPLuginDriverStates::LOAD_PLUGINS);
  return Status::OK();
}

Status
McPLuginDriver::load_plugins_state()
{
  RCLCPP_INFO(this->get_logger(), "Loading plugins and creating slave drivers");
  for (auto& modules : _param_loader.get_module_params()) {
    auto maybe_device =
      std::find_if(_connected_devices.begin(),
                   _connected_devices.end(),
                   [&modules](const std::shared_ptr<McBasicModuleDriver>& driver) {
                     return driver->get_unique_id() == modules.second.unique_id;
                   });
    if (maybe_device == _connected_devices.end()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No discovered device matches module config with unique ID: %x",
                   modules.second.unique_id);
      return Status::Invalid(
        "No discovered device matches module config with unique ID: " +
        std::to_string(modules.second.unique_id));
    }
    if (!maybe_device->get()->is_device_supported()) {
      return Status::Invalid(
        "Device with unique ID: " + std::to_string(modules.second.unique_id) +
        " is not supported by any loaded plugin. The Device requires plugin with PID:" +
        std::to_string(maybe_device->get()->get_plugin_identifier()));
    }

    auto& device = *maybe_device;
    ARI_RETURN_ON_ERROR(device->load_plugin(modules.second));
    _active_slave_drivers.emplace_back(device);
  }

  for (const auto& driver : _active_slave_drivers) {
    RCLCPP_INFO(this->get_logger(),
                "Loaded plugin for device Name: %s ID: %x, Plugin Unique ID: %x",
                driver->get_driver()->driver_get_module_params().module_ros_name.c_str(),
                driver->get_unique_id(),
                driver->get_plugin_identifier());
    ARI_RETURN_ON_ERROR(driver->request_new_node_id(
      driver->get_driver()->driver_get_module_params().node_id));
  }
  // we will give more less all nodes a bit of time to change to Normal operational mode
  // WHY because with linux can socket to handle registering multiple callbacks and to add
  // filter to CAN at driver level we have to disable the can socket temprarly and then
  // re-enable it after we register th ecallback unfortunately this takes few milisecodn
  // adn when we register 40 callbacks fro each module this adds up and can cause some
  // messages to be lost if they are sent during the time we are registering callbacks, so
  // we will just wait for a moment after registering all callbacks to give time for all
  // devices to switch to normal mode and start sending messages before we start the
  // control loop and expect to receive messages from the devices.
  std::this_thread::sleep_for(2000ms);
  switch_to_state(McPLuginDriverStates::INIT_MODULES);
  return Status::OK();
}

Status
McPLuginDriver::init_modules()
{
  RCLCPP_INFO(this->get_logger(), "Initializing modules");
  for (const auto& driver : _active_slave_drivers) {
    driver->start_driver();
    ARI_ASIGN_OR_RETURN(response, driver->request_get_hardware_type());
    ARI_RETURN_ON_ERROR(driver->get_driver()->driver_on_init());
    ARI_RETURN_ON_ERROR(driver->get_driver()->driver_on_activate());

    RCLCPP_INFO(this->get_logger(),
                "Initialized module and ROS interfaces for Module: %s Name: %s ID: %x, "
                "PID: %x, Hardware "
                "Firmware rev: %u, Hardware rev: %u, Hardware timestamp: %u",
                driver->get_driver()->driver_get_module_params().module_ros_name.c_str(),
                driver->get_driver()->driver_get_module_params().module_name.c_str(),
                driver->get_unique_id(),
                driver->get_plugin_identifier(),
                response.fw_revision,
                response.hw_revision,
                response.hw_time_stamp);
  }
  switch_to_state(McPLuginDriverStates::CONTROL_LOOP);
  return Status::OK();
}

Status
McPLuginDriver::control_loop()
{
  for (const auto& driver : _active_slave_drivers) {
    if (!driver->control_loop().ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Control loop for module %s returned error: %s",
        driver->get_driver()->driver_get_module_params().module_ros_name.c_str(),
        driver->control_loop().to_string().c_str());
      switch_to_state(McPLuginDriverStates::ERROR_MODE);
    }
  }

  return Status::OK();
}

Status
McPLuginDriver::exit_control_loop()
{
  RCLCPP_INFO(this->get_logger(), "Exiting control loop");
  for (const auto& driver : _active_slave_drivers) {
    ARI_RETURN_ON_ERROR(driver->get_driver()->driver_on_deactivate());
  }
  return Status::OK();
}

Status
McPLuginDriver::error_mode()
{
  RCLCPP_ERROR(this->get_logger(), "Entering unrecoverable ERROR.");

  rclcpp::shutdown();

  return Status::OK();
}