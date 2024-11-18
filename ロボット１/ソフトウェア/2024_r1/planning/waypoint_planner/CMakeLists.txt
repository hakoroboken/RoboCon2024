cmake_minimum_required(VERSION 3.5)
project(waypoint_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_waypoint_planner SHARED
  DIRECTORY src
)

target_include_directories(target_waypoint_planner
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_waypoint_planner
  PLUGIN "waypoint_planner::WayPointPlanner"
  EXECUTABLE waypoint_planner_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)