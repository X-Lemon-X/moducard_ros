// Auto-generated CAN frame encoder/decoder for ROS messages
//
// DO NOT EDIT MANUALLY
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

// ROS message includes
#include <mcan_basic_module_msgs/msg/device_status.hpp>
#include <mcan_basic_module_msgs/msg/device_node_identifier.hpp>
#include <mcan_basic_module_msgs/msg/hardware_type.hpp>
#include <mcan_basic_module_msgs/msg/device_identifier.hpp>
#include <mcan_basic_module_msgs/msg/device_info.hpp>
#include <mcan_basic_module_msgs/msg/discover_devices.hpp>
#include <mcan_basic_module_msgs/msg/set_device_node_id.hpp>
#include <mcan_basic_module_msgs/msg/enter_configuration_mode.hpp>
#include <mcan_basic_module_msgs/msg/get_hardware_type.hpp>
#include <mcan_basic_module_msgs/msg/ping_module.hpp>
#include <mcan_basic_module_msgs/msg/flash_indicator_led.hpp>

// C++ type includes
#include <mcan_basic_module_msgs/mcan_basic_module_types.hpp>

namespace mcan_basic_module_msgs {

/**
 * Message conversion utilities for CAN frames and ROS messages.
 * All methods are static inline for use as template parameters.
 *
 * Four conversion types:
 * 1. mcan_encode_val_to_ros() - Primitive values to std_msgs wrappers
 * 2. mcan_decode_type_to_ros() - C++ custom types to ROS type messages
 * 3. mcan_decode_message_to_ros() - Module messages to ROS messages (with header)
 * 4. mcan_encode_ros_to_message() - ROS messages to module messages
 */
struct Conversions {

  // ========== Primitive Value to ROS Wrapper ==========

  static inline std_msgs::msg::UInt8 mcan_encode_val_to_ros(const std::uint8_t& val) {
    std_msgs::msg::UInt8 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::UInt16 mcan_encode_val_to_ros(const std::uint16_t& val) {
    std_msgs::msg::UInt16 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::UInt32 mcan_encode_val_to_ros(const std::uint32_t& val) {
    std_msgs::msg::UInt32 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::UInt64 mcan_encode_val_to_ros(const std::uint64_t& val) {
    std_msgs::msg::UInt64 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Int8 mcan_encode_val_to_ros(const std::int8_t& val) {
    std_msgs::msg::Int8 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Int16 mcan_encode_val_to_ros(const std::int16_t& val) {
    std_msgs::msg::Int16 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Int32 mcan_encode_val_to_ros(const std::int32_t& val) {
    std_msgs::msg::Int32 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Int64 mcan_encode_val_to_ros(const std::int64_t& val) {
    std_msgs::msg::Int64 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Float32 mcan_encode_val_to_ros(const float& val) {
    std_msgs::msg::Float32 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Float64 mcan_encode_val_to_ros(const double& val) {
    std_msgs::msg::Float64 msg;
    msg.data = val;
    return msg;
  }

  static inline std_msgs::msg::Bool mcan_encode_val_to_ros(const bool& val) {
    std_msgs::msg::Bool msg;
    msg.data = val;
    return msg;
  }


  template<float Scale>
  static inline double
  mcan_decode_type_to_ros(const FloatInt16_t<Scale>& val)
  {
    return (double)val;
  }

  static inline double
  mcan_encode_ros_to_type(const double& val)
  {
    return (double)val;
  }
                         
  // ========== Custom Type Conversions ==========

  /**
   * Convert C++ custom type to ROS type message
   * @param val C++ type instance
   * @return ROS type message
   */
  static inline msg::DeviceStatus mcan_decode_type_to_ros(const mcan::mcan_basic_module::DeviceStatus_t& val) {
    msg::DeviceStatus msg;
    msg.value = static_cast<uint8_t>(val);
    return msg;
  }

  /**
   * Convert ROS type message to C++ custom type
   * @param msg ROS type message
   * @return C++ type instance
   */
  static inline mcan::mcan_basic_module::DeviceStatus_t mcan_encode_ros_to_type(const msg::DeviceStatus& msg) {
    return static_cast<mcan::mcan_basic_module::DeviceStatus_t>(msg.value);
  }

  /**
   * Convert C++ custom type to ROS type message
   * @param val C++ type instance
   * @return ROS type message
   */
  static inline msg::DeviceNodeIdentifier mcan_decode_type_to_ros(const mcan::mcan_basic_module::DeviceNodeIdentifier_t& val) {
    msg::DeviceNodeIdentifier msg;
    msg.manufacturer_id = val.manufacturer_id;
    return msg;
  }

  /**
   * Convert ROS type message to C++ custom type
   * @param msg ROS type message
   * @return C++ type instance
   */
  static inline mcan::mcan_basic_module::DeviceNodeIdentifier_t mcan_encode_ros_to_type(const msg::DeviceNodeIdentifier& msg) {
    mcan::mcan_basic_module::DeviceNodeIdentifier_t val;
    val.manufacturer_id = msg.manufacturer_id;
    return val;
  }

  /**
   * Convert C++ custom type to ROS type message
   * @param val C++ type instance
   * @return ROS type message
   */
  static inline msg::HardwareType mcan_decode_type_to_ros(const mcan::mcan_basic_module::HardwareType_t& val) {
    msg::HardwareType msg;
    msg.hw_time_stamp = val.hw_time_stamp;
    msg.hw_revision = val.hw_revision;
    msg.fw_revision = val.fw_revision;
    return msg;
  }

  /**
   * Convert ROS type message to C++ custom type
   * @param msg ROS type message
   * @return C++ type instance
   */
  static inline mcan::mcan_basic_module::HardwareType_t mcan_encode_ros_to_type(const msg::HardwareType& msg) {
    mcan::mcan_basic_module::HardwareType_t val;
    val.hw_time_stamp = msg.hw_time_stamp;
    val.hw_revision = msg.hw_revision;
    val.fw_revision = msg.fw_revision;
    return val;
  }

  /**
   * Convert C++ custom type to ROS type message
   * @param val C++ type instance
   * @return ROS type message
   */
  static inline msg::DeviceIdentifier mcan_decode_type_to_ros(const mcan::mcan_basic_module::DeviceIdentifier_t& val) {
    msg::DeviceIdentifier msg;
    msg.unique_id = val.unique_id;
    return msg;
  }

  /**
   * Convert ROS type message to C++ custom type
   * @param msg ROS type message
   * @return C++ type instance
   */
  static inline mcan::mcan_basic_module::DeviceIdentifier_t mcan_encode_ros_to_type(const msg::DeviceIdentifier& msg) {
    mcan::mcan_basic_module::DeviceIdentifier_t val;
    val.unique_id = msg.unique_id;
    return val;
  }

  /**
   * Convert C++ custom type to ROS type message
   * @param val C++ type instance
   * @return ROS type message
   */
  static inline msg::DeviceInfo mcan_decode_type_to_ros(const mcan::mcan_basic_module::DeviceInfo_t& val) {
    msg::DeviceInfo msg;
    msg.device_identifier = mcan_basic_module_msgs::Conversions::mcan_decode_type_to_ros(val.device_identifier);
    msg.hardware_type = mcan_basic_module_msgs::Conversions::mcan_decode_type_to_ros(val.hardware_type);
    return msg;
  }

  /**
   * Convert ROS type message to C++ custom type
   * @param msg ROS type message
   * @return C++ type instance
   */
  static inline mcan::mcan_basic_module::DeviceInfo_t mcan_encode_ros_to_type(const msg::DeviceInfo& msg) {
    mcan::mcan_basic_module::DeviceInfo_t val;
    val.device_identifier = mcan_basic_module_msgs::Conversions::mcan_encode_ros_to_type(msg.device_identifier);
    val.hardware_type = mcan_basic_module_msgs::Conversions::mcan_encode_ros_to_type(msg.hardware_type);
    return val;
  }

  // ========== Module Message Conversions ==========

};  // struct Conversions

}  // namespace mcan_basic_module_msgs
