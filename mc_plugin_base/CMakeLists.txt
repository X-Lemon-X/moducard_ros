cmake_minimum_required(VERSION 3.8)
project(mc_plugin_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_library(${PROJECT_NAME} SHARED
 src/mc_plugin_base.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mc_drivers_common/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  PUBLIC
  rclcpp
)

ament_export_dependencies(
  rclcpp
)

# # Create interface library for C++ type headers
# add_library(${PROJECT_NAME}_cpp_headers INTERFACE)
# target_include_directories(${PROJECT_NAME}_cpp_headers INTERFACE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(${PROJECT_NAME}_cpp_headers INTERFACE rclcpp)
# install(TARGETS ${PROJECT_NAME}_cpp_headers
#   EXPORT ${PROJECT_NAME}_cpp_headersTargets)
# install(EXPORT ${PROJECT_NAME}_cpp_headersTargets
#   DESTINATION share/${PROJECT_NAME}/cmake)


# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(
  DIRECTORY mc_drivers_common/include/
  DESTINATION include/mc_plugin_base
)

install(TARGETS mc_plugin_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
