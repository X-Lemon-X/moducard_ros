#pragma once

#include "mc_plugin_base/mc_firmware/can_base.hpp"
#include "mc_plugin_base/mc_firmware/mc_common.hpp"
#include "mc_plugin_base/mc_firmware/status.hpp"
#include "mc_plugin_driver/ros_param_loaded.hpp"
#include "mc_plugin_driver/timing.hpp"
#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy.hpp"
#include "mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <bitset>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

#define HARDWARE_PUBLIC __attribute__((visibility("default")))

namespace mcan {

using namespace mcan_basic_module;
using namespace mcan_base_module_dummy;

enum class McSlavePluginDriverState : uint8_t
{
  UNINITIALIZED,
  INITIALIZED,
  ACTIVE,
  ERROR
};

class McSlavePluginDriverBase : public rclcpp::Node
{
 public:
  McSlavePluginDriverBase(rclcpp::Node& node,
                          std::string node_name,
                          std::shared_ptr<CanBase> can_interface_primary,
                          std::shared_ptr<CanBase> can_interface_secondary,
                          const ModuleParams& module_params,
                          uint64_t unique_driver_id)

    : Node(node, node_name)
    , _can_interface_primary(std::move(can_interface_primary))
    , _can_interface_secondary(std::move(can_interface_secondary))
    , _unique_driver_id(unique_driver_id)
    , _module_params(module_params)
  {
  }

  virtual ~McSlavePluginDriverBase() = default;

  /// @brief Start the driver, this will be called after the driver is created.
  ///
  /// This function will creat all the ros interfaces: publishers, subscribers
  HARDWARE_PUBLIC virtual Status driver_on_init() = 0;

  /// @brief Start the drive work.
  ///
  /// This should add appropriate callbacks to the CAN interface.
  HARDWARE_PUBLIC virtual Status driver_on_activate() = 0;

  /// @brief Stop the driver , this will be called when the driver is stopped or before
  /// cleanup.
  ///
  /// This should remove all the callbacks from the CAN interface and stop any ongoing
  /// work.
  HARDWARE_PUBLIC virtual Status driver_on_deactivate() = 0;

  /// @brief This will be called in a loop after the driver is activated, this is where
  /// the main control logic should be implemented.
  ///
  /// So this call should basically request appropriate hardware states with appropriate
  /// frequencies updates according to module config and send out commands to control
  /// using some queues from services
  HARDWARE_PUBLIC virtual Status driver_control_call() = 0;

  HARDWARE_PUBLIC virtual Status driver_read_commands() = 0;

  HARDWARE_PUBLIC virtual Status driver_read_states() = 0;

  HARDWARE_PUBLIC virtual Status driver_read_configs() = 0;

  HARDWARE_PUBLIC virtual std::string driver_get_name() = 0;

  HARDWARE_PUBLIC void driver_set_node_id(uint8_t node_id)
  {
    _module_params.node_id = node_id;
  }

  HARDWARE_PUBLIC const ModuleParams& driver_get_module_params() const
  {
    return _module_params;
  }

  HARDWARE_PUBLIC McSlavePluginDriverState driver_get_state() const
  {
    return _driver_state;
  }

  HARDWARE_PUBLIC uint64_t driver_get_driver_unique_id() const
  {
    return _unique_driver_id;
  }

 protected:
  const std::shared_ptr<CanBase> _can_interface_primary;
  const std::shared_ptr<CanBase> _can_interface_secondary;
  const uint64_t _unique_driver_id;
  ModuleParams _module_params;

  DeviceStatus_t _device_status = DeviceStatus_t::UNKNOWN;
  DeviceIdentifier_t _device_identifier = {};
  McSlavePluginDriverState _driver_state = McSlavePluginDriverState::UNINITIALIZED;
};

template<typename McCanSlaveInterface, typename Hardware, typename McRosMsgTranslator>
class McSlavePluginDriver : public McSlavePluginDriverBase
{

 public:
  Result<McSlavePluginDriver<
    McCanSlaveInterface,
    Hardware,
    McRosMsgTranslator>*> static Make(rclcpp::Node& node,
                                      std::shared_ptr<CanBase> can_interface_primary,
                                      std::shared_ptr<CanBase> can_interface_secondary,
                                      const ModuleParams& module_params)
  {
    if (!can_interface_primary) {
      return Status::Invalid("Can primary interface is null");
    }

    if (!can_interface_secondary) {
      return Status::Invalid("Can secondary interface is null");
    }

    // if (module_params.module_name.empty()) {
    //   return Status::Invalid("Module name is empty");
    // }

    if (module_params.module_ros_name.empty()) {
      return Status::Invalid("Module ROS name is empty");
    }

    if (module_params.unique_id > 0x1FFFFF) {
      return Status::Invalid("UID must be a 21-bit value");
    }

    if (module_params.node_id < 2 || module_params.node_id > 255) {
      return Status::Invalid(
        "Node ID must be an 8-bit value 2-255 (0 and 1 are reserved)");
    }

    if (module_params.frame_id.empty()) {
      return Status::Invalid("Frame ID is not specified");
    }

    return Result<McSlavePluginDriver*>::OK(
      new McSlavePluginDriver(node,
                              std::move(can_interface_primary),
                              std::move(can_interface_secondary),
                              std::move(module_params)));
  }

  HARDWARE_PUBLIC virtual Status driver_on_init() override
  {
    if (_driver_state != McSlavePluginDriverState::UNINITIALIZED) {
      return Status::OK("Driver is already initialized");
    }

    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            using MsgT = std::decay_t<decltype(this->_interface.*(args))>;
            using RosMsgT =
              std::decay_t<decltype(McRosMsgTranslator::mcan_decode_message_to_ros(
                this->_interface.*(args)))>;
            auto msg_buffer = std::make_shared<CanMultiPackageFrame<MsgT>>();
            auto publisher = this->create_publisher<RosMsgT>(
              _module_params.module_ros_name + "/get/" + std::string(MsgT::k_group) +
                "/" + std::string(MsgT::k_name),
              10);
            uint32_t can_id = mcan_connect_msg_id_with_node_id(
              (this->_interface.*(args)).k_base_address, _module_params.node_id);

            auto can_callback = [this, args, msg_buffer, publisher](
                                  CanBase& can, const CanFrame& frame, void* arg) {
              (void)can;
              (void)arg;
              Status status = mcan_unpack_msg(frame, *msg_buffer);
              if (status.status_code() == StatusCode::Cancelled) {
                return; // wait for more frames
              } else if (!status.ok()) {
                return; // error unpacking
              }
              auto& state_ver = this->_interface.*(args);
              state_ver.value = msg_buffer->value;
              (this->_interface.*(args)).value = state_ver.value;
              auto ros_msg = McRosMsgTranslator::mcan_decode_message_to_ros(state_ver);
              ros_msg.header.stamp = this->get_clock()->now();
              ros_msg.header.frame_id = this->_module_params.frame_id;
              publisher->publish(ros_msg);
            };
            _can_callbacks.emplace(can_id, std::move(can_callback));
          }(),
          ...);
      },
      _interface.get_read_variables());

    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            using MsgT = std::decay_t<decltype(this->_interface.*(args))>;
            using RosMsgT =
              std::decay_t<decltype(McRosMsgTranslator::mcan_decode_message_to_ros(
                this->_interface.*(args)))>;
            auto subscription = this->create_subscription<RosMsgT>(
              _module_params.module_ros_name + "/set/" + std::string(MsgT::k_group) +
                "/" + std::string(MsgT::k_name),
              10,
              [this, args](const RosMsgT& ros_msg) {
                if (!this->_is_active.load()) {
                  RCLCPP_WARN(this->get_logger(),
                              "Received command while driver is not active, ignoring");
                  return;
                }
                auto cmd = McRosMsgTranslator::mcan_encode_ros_to_message(ros_msg);
                (this->_interface.*(args)).value = cmd.value;
                mcan_pack_send_msg(*_can_interface_primary,
                                   this->_interface.*(args),
                                   _module_params.node_id);
              });
            _subscriptions.push_back(subscription);
          }(),
          ...);
      },
      _interface.get_write_variables());

    // load timeres
    std::apply(
      [&](auto&&... args) {
        size_t idx = 0;
        (
          [&] {
            using MsgT = std::decay_t<decltype(this->_interface.*(args))>;
            auto maybe_override_rate = std::find_if(
              _module_params.states_custom_rates.begin(),
              _module_params.states_custom_rates.end(),
              [&](const auto& entry) { return entry.first == MsgT::k_name; });

            float update_rate = _module_params.states_update_default_rate;
            if (maybe_override_rate != _module_params.states_custom_rates.end()) {
              RCLCPP_DEBUG(this->get_logger(),
                           "Using custom update rate for state %s: %f Hz",
                           MsgT::k_name,
                           maybe_override_rate->second);
              update_rate = maybe_override_rate->second;
            }
            _timers.emplace(idx, TimingObject(update_rate));
            ++idx;
          }(),
          ...);
      },
      _interface.get_state_variables());

    _driver_state = McSlavePluginDriverState::INITIALIZED;
    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_on_activate() override
  {
    if (_is_active.load()) {
      return Status::OK("Driver is already active");
    }

    _is_active.store(true);
    _can_interface_primary->close_can();
    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            uint32_t can_id = mcan_connect_msg_id_with_node_id(
              (this->_interface.*(args)).k_base_address, _module_params.node_id);
            auto callback = _can_callbacks.find(can_id);
            if (callback == _can_callbacks.end()) {
              return Status::KeyError("No callback found for CAN ID: " +
                                      std::to_string(can_id));
            }
            return _can_interface_primary->add_callback(can_id, callback->second);
          }(),
          ...);
      },
      _interface.get_read_variables());

    _can_interface_primary->open_can();
    driver_read_commands();
    driver_read_states();
    driver_read_configs();

    for (auto& timer : _timers) {
      timer.second.reset();
    }

    _driver_state = McSlavePluginDriverState::ACTIVE;
    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_on_deactivate() override
  {
    if (!_is_active.load()) {
      return Status::OK("Driver is already inactive");
    }
    _is_active.store(false);
    _can_interface_primary->close_can();
    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            uint32_t can_id = mcan_connect_msg_id_with_node_id(
              (this->_interface.*(args)).k_base_address, _module_params.node_id);
            return _can_interface_primary->remove_callback(can_id);
          }(),
          ...);
      },
      _interface.get_read_variables());
    _can_interface_primary->open_can();
    _driver_state = McSlavePluginDriverState::INITIALIZED;
    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_control_call() override
  {
    if (!_is_active.load()) {
      return Status::OK("Driver is not active");
    }

    std::apply(
      [&](auto&&... args) {
        size_t idx = 0;
        (
          [&] {
            if constexpr ((this->_interface.*(args)).k_allow_read) {
              if (FrequencyTimer::should_trigger_and_reset(this->_timers[idx])) {
                mcan_request_msg(*_can_interface_primary,
                                 this->_interface.*(args),
                                 _module_params.node_id);
              }
            }
            ++idx;
          }(),
          ...);
      },
      _interface.get_state_variables());

    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_read_commands() override
  {
    ARI_RETURN_ON_ERROR(return_when_not_active());
    // make request to hardware to read commands if needed
    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            if constexpr ((this->_interface.*(args)).k_allow_read) {
              mcan_request_msg(*_can_interface_primary,
                               this->_interface.*(args),
                               _module_params.node_id);
            }
          }(),
          ...);
      },
      _interface.get_command_variables());

    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_read_states() override
  {
    ARI_RETURN_ON_ERROR(return_when_not_active());
    // make request to hardware to read states if needed
    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            if constexpr ((this->_interface.*(args)).k_allow_read) {
              mcan_request_msg(*_can_interface_primary,
                               this->_interface.*(args),
                               _module_params.node_id);
            }
          }(),
          ...);
      },
      _interface.get_state_variables());

    return Status::OK();
  };

  HARDWARE_PUBLIC virtual Status driver_read_configs() override
  {
    ARI_RETURN_ON_ERROR(return_when_not_active());
    // make request to hardware to read configs if needed
    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            if constexpr ((this->_interface.*(args)).k_allow_read) {
              mcan_request_msg(*_can_interface_primary,
                               this->_interface.*(args),
                               _module_params.node_id);
            }
          }(),
          ...);
      },
      _interface.get_config_variables());

    return Status::OK();
  };

  HARDWARE_PUBLIC virtual std::string driver_get_name() override
  {
    return std::string(_hardware.k_name);
  }

 private:
  McSlavePluginDriver(rclcpp::Node& node,
                      std::shared_ptr<CanBase> can_interface_primary,
                      std::shared_ptr<CanBase> can_interface_secondary,
                      const ModuleParams& module_params)
    : McSlavePluginDriverBase(node,
                              module_params.module_ros_name + "_node",
                              std::move(can_interface_primary),
                              std::move(can_interface_secondary),
                              std::move(module_params),
                              Hardware::k_unique_id)
  {
  }

  Status return_when_not_active()
  {
    if (!_is_active.load()) {
      return Status::Invalid("Driver is not active");
    }
    return Status::OK();
  }

  std::unordered_map<uint32_t, mcan::CanBase::can_callback_type> _can_callbacks;
  std::unordered_map<size_t, TimingObject> _timers;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> _subscriptions;

  std::atomic<bool> _is_active{ false };
  Hardware _hardware;
  McCanSlaveInterface _interface;
  DeviceMode _mode{ DeviceMode::UNDEFINED };
};

} // namespace mcan