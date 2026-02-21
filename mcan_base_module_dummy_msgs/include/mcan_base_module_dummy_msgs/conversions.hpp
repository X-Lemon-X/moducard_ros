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
#include <mcan_base_module_dummy_msgs/msg/discover_devices.hpp>
#include <mcan_base_module_dummy_msgs/msg/set_device_node_id.hpp>
#include <mcan_base_module_dummy_msgs/msg/enter_configuration_mode.hpp>
#include <mcan_base_module_dummy_msgs/msg/get_hardware_type.hpp>
#include <mcan_base_module_dummy_msgs/msg/ping_module.hpp>
#include <mcan_base_module_dummy_msgs/msg/flash_indicator_led.hpp>
#include <mcan_base_module_dummy_msgs/msg/dummy_command.hpp>

// C++ type includes
#include <mcan_base_module_dummy_msgs/mcan_base_module_dummy_types.hpp>
#include <mcan_base_module_dummy_msgs/mcan_base_module_dummy.hpp>

// Dependency conversions
#include <mcan_basic_module_msgs/conversions.hpp>

namespace mcan_base_module_dummy_msgs {

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

  // ========== Custom Type Conversions ==========

  // ========== Module Message Conversions ==========

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::DiscoverDevices mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::DiscoverDevices& msg_struct) {
    msg::DiscoverDevices msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::DiscoverDevices mcan_encode_ros_to_message(const msg::DiscoverDevices& msg) {
    mcan::mcan_base_module_dummy::configs::DiscoverDevices msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::SetDeviceNodeId mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::SetDeviceNodeId& msg_struct) {
    msg::SetDeviceNodeId msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::SetDeviceNodeId mcan_encode_ros_to_message(const msg::SetDeviceNodeId& msg) {
    mcan::mcan_base_module_dummy::configs::SetDeviceNodeId msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::EnterConfigurationMode mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::EnterConfigurationMode& msg_struct) {
    msg::EnterConfigurationMode msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::EnterConfigurationMode mcan_encode_ros_to_message(const msg::EnterConfigurationMode& msg) {
    mcan::mcan_base_module_dummy::configs::EnterConfigurationMode msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::GetHardwareType mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::GetHardwareType& msg_struct) {
    msg::GetHardwareType msg;
    msg.data = mcan_basic_module_msgs::Conversions::mcan_decode_type_to_ros(msg_struct.value);
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::GetHardwareType mcan_encode_ros_to_message(const msg::GetHardwareType& msg) {
    mcan::mcan_base_module_dummy::configs::GetHardwareType msg_struct;
    msg_struct.value = mcan_basic_module_msgs::Conversions::mcan_encode_ros_to_type(msg.data);
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::PingModule mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::PingModule& msg_struct) {
    msg::PingModule msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::PingModule mcan_encode_ros_to_message(const msg::PingModule& msg) {
    mcan::mcan_base_module_dummy::configs::PingModule msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::FlashIndicatorLed mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::FlashIndicatorLed& msg_struct) {
    msg::FlashIndicatorLed msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::FlashIndicatorLed mcan_encode_ros_to_message(const msg::FlashIndicatorLed& msg) {
    mcan::mcan_base_module_dummy::configs::FlashIndicatorLed msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

  /**
   * Convert C++ module message struct to ROS message
   * @param msg_struct C++ module message struct (from main header)
   * @return ROS message with data (header not initialized)
   */
  static inline msg::DummyCommand mcan_decode_message_to_ros(const mcan::mcan_base_module_dummy::configs::DummyCommand& msg_struct) {
    msg::DummyCommand msg;
    msg.data = msg_struct.value;
    return msg;
  }

  /**
   * Convert ROS message to C++ module message struct (extracts data, ignores header)
   * @param msg ROS message instance
   * @return C++ module message struct (from main header)
   */
  static inline mcan::mcan_base_module_dummy::configs::DummyCommand mcan_encode_ros_to_message(const msg::DummyCommand& msg) {
    mcan::mcan_base_module_dummy::configs::DummyCommand msg_struct;
    msg_struct.value = msg.data;
    return msg_struct;
  }

};  // struct Conversions

}  // namespace mcan_base_module_dummy_msgs
