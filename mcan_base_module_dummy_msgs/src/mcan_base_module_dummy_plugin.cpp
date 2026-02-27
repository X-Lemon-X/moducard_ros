// Auto-generated plugin main file for ROS 2
//
// This will be converted to a plugin implementation
//

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>

// Module type headers
#include <mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp>
#include <mcan_base_module_dummy_msgs/mcan_base_module_dummy.hpp>

// Conversion utilities
#include <mcan_base_module_dummy_msgs/conversions.hpp>

// Generated ROS messages
#include <mcan_base_module_dummy_msgs/msg/discover_devices.hpp>
#include <mcan_base_module_dummy_msgs/msg/set_device_node_id.hpp>
#include <mcan_base_module_dummy_msgs/msg/enter_configuration_mode.hpp>
#include <mcan_base_module_dummy_msgs/msg/get_hardware_type.hpp>
#include <mcan_base_module_dummy_msgs/msg/ping_module.hpp>
#include <mcan_base_module_dummy_msgs/msg/flash_indicator_led.hpp>
#include <mcan_base_module_dummy_msgs/msg/dummy_command.hpp>

// Dependency conversions
#include <mcan_basic_module_msgs/conversions.hpp>

#include <mc_plugin_driver/mc_plugin_driver.hpp>
#include <mc_plugin_driver/mc_slave_driver.hpp>
#include <mc_plugin_driver/mc_param.hpp>
#include <mc_plugin_driver/mc_plugin_exporter.hpp>
#include <mc_can_driver/can_linux_driver.hpp>


namespace mcan {
class mcan_base_module_dummy_plugin : public McPluginExporterBase {
public:
  mcan_base_module_dummy_plugin() : McPluginExporterBase() {
  }

  virtual Result<std::shared_ptr<McSlavePluginDriverBase>> create_new_instance(rclcpp::Node &node,
                                                                               std::shared_ptr<CanBase> can_primary,
                                                                               std::shared_ptr<CanBase> can_secondary,
                                                                               const ModuleParams &params) override {
    auto result =
    mcan::McSlavePluginDriver<mcan::mcan_base_module_dummy::McCanSlaveInterface_t, mcan::mcan_base_module_dummy::Hardware_t, mcan_base_module_dummy_msgs::Conversions>::Make(
    node, std::move(can_primary), std::move(can_secondary), params);

    if(!result.ok()) {
      return result.status();
    }
    return Result<std::shared_ptr<McSlavePluginDriverBase>>::OK(
    std::shared_ptr<McSlavePluginDriverBase>(result.valueOrDie()));
  }

  virtual uint64_t get_plugin_unique_id() const override {
    return mcan::mcan_base_module_dummy::Hardware_t::k_unique_id;
  };
};

}; // namespace mcan

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mcan::mcan_base_module_dummy_plugin, mcan::McPluginExporterBase)
