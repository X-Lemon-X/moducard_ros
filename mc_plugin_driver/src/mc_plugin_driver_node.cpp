#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mc_plugin_base/mc_firmware/can_base.hpp"
#include "mc_plugin_driver/mc_plugin_driver.hpp"

using namespace mcan;

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McPLuginDriver>());
  rclcpp::shutdown();
  return 0;
}
