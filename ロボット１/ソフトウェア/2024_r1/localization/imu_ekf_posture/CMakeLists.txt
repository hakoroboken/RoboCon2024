cmake_minimum_required(VERSION 3.5)
project(imu_ekf_posture)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_imu_ekf_posture SHARED
  DIRECTORY src
)

target_include_directories(target_imu_ekf_posture
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_imu_ekf_posture
  PLUGIN "imu_ekf_posture::ImuEkfPosture"
  EXECUTABLE imu_ekf_posture_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)