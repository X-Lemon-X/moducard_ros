cmake_minimum_required(VERSION 3.8)
project(mcan_base_module_dummy_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mcan_basic_module_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DummyCommand.msg"
  DEPENDENCIES std_msgs mcan_basic_module_msgs
)


# Create interface library for C++ type headers
add_library(mcan_base_module_dummy_cpp_headers INTERFACE)
target_include_directories(mcan_base_module_dummy_cpp_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mcan_base_module_dummy_cpp_headers INTERFACE mcan_basic_module_msgs
)
install(TARGETS mcan_base_module_dummy_cpp_headers
  EXPORT mcan_base_module_dummy_cpp_headersTargets)
install(EXPORT mcan_base_module_dummy_cpp_headersTargets
  DESTINATION share/${PROJECT_NAME}/cmake)

find_package(rclcpp REQUIRED)

install(
DIRECTORY include/
DESTINATION include
)
# Export include directories for downstream packages
ament_export_include_directories(include)
ament_export_targets(mcan_base_module_dummy_cpp_headersTargets)
ament_export_dependencies(mcan_basic_module_msgs std_msgs)

ament_package()