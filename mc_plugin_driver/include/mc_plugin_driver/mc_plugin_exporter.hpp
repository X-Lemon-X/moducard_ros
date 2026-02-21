#pragma once

#include "mc_plugin_base/can_base.hpp"
#include "mc_plugin_base/status.hpp"

#include "mc_plugin_driver/mc_slave_driver.hpp"

namespace mcan {


class McPluginExporterBase {
public:
  McPluginExporterBase()          = default;
  virtual ~McPluginExporterBase() = default;

  virtual Result<std::shared_ptr<McSlavePluginDriverBase>> create_new_instance(rclcpp::Node &node,
                                                                               std::shared_ptr<CanBase> can_primary,
                                                                               std::shared_ptr<CanBase> can_secondary,
                                                                               const ModuleParams &params) = 0;

  virtual uint64_t get_plugin_unique_id() const = 0;
};

} // namespace mcan