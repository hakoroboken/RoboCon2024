cmake_minimum_required(VERSION 3.5)
project(ombridge_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_ombridge_driver SHARED
  DIRECTORY src
)

target_include_directories(target_ombridge_driver
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_ombridge_driver
  PLUGIN "ombridge_driver::OmBridgeDriver"
  EXECUTABLE ombridge_driver_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)