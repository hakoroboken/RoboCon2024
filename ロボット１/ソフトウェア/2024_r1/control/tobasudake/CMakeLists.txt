cmake_minimum_required(VERSION 3.5)
project(tobasudake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_tobasudake SHARED
  DIRECTORY src
)

target_include_directories(target_tobasudake
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_tobasudake
  PLUGIN "tobasudake::TobasuDake"
  EXECUTABLE tobasudake_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)