// Auto-generated header for base_module_dummy module
// Do not edit manually

#pragma once

#include <cstdint>
#include <string>
#include <cstddef>
#include <tuple>
#include "super_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"


#include "base_module_dummy_types.hpp"
#include "base_module_dummy.hpp"

namespace mcan {

namespace base_module_dummy {

struct McRosMsgTranslator {
  static std_msgs::msg::UInt8 mcan_encode_msg_val_to_ros_msg_val(const uint8_t &val) {
    std_msgs::msg::UInt8 ros_msg;
    ros_msg.data = val;
    return ros_msg;
  }

  // template <typename T> static float mcan_encode_msg_val_to_ros_msg_val(const FloatInt16_t<T> &val) {
  //   return val();
  // }

  static HardwareTypeMsg mcan_decode_msg_to_ros_msg(const HardwareType_t &msg) {
    HardwareTypeMsg ros_msg;
    ros_msg.hw_revision = msg.hw_revision;
    ros_msg.fw_revision = msg.fw_revision;
    ros_msg.time_stamp  = msg.hw_time_stamp;
    return ros_msg;
  }


  static DummyCommandMsg mcan_decode_msg_to_ros_msg(const configs::DummyCommand &cmd) {
    DummyCommandMsg ros_msg;
    ros_msg.data = mcan_encode_msg_val_to_ros_msg_val(cmd.value);
    return ros_msg;
  }

  static configs::GetHardwareType mcan_encode_ros_msg_to_msg(const HardwareTypeMsg &ros_msg) {
    configs::GetHardwareType cmd;
    cmd.value = mcan_encode_ros_msg_val_msg_val(ros_msg.data);
    return cmd;
  }
};

} // namespace base_module_dummy

} // namespace mcan