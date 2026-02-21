cmake_minimum_required(VERSION 3.8)
project(mc_plugin_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  mc_plugin_base
  mcan_base_module_dummy_msgs
  mc_can_driver
  pluginlib
  yaml-cpp
)


set(CMAKE_CXX_STANDARD 23)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(mc_plugin_driver_node
  src/mc_plugin_driver.cpp
  src/mc_plugin_driver_node.cpp
)

# Create interface library for C++ type headers
# add_library(${PROJECT_NAME} INTERFACE)
# target_include_directories(${PROJECT_NAME} INTERFACE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}_cpp_headersTargets)
# install(EXPORT ${PROJECT_NAME}_cpp_headersTargets
#   DESTINATION share/${PROJECT_NAME}/cmake)

target_include_directories(mc_plugin_driver_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)





add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE c_std_99 cxx_std_23)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${pluginlib_TARGETS})



ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp yaml-cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})







# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library and export targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# PLUGIN base












ament_target_dependencies(
  mc_plugin_driver_node PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  mc_plugin_driver_node PUBLIC
    ${PARAMETERS_LIB_NAME}
    yaml-cpp
)

# install(TARGETS 
#   ${PROJECT_NAME} 
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

# install(TARGETS mc_plugin_driver_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )


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

install(TARGETS mc_plugin_driver_node
  DESTINATION lib/${PROJECT_NAME}
)


# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )


ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
