cmake_minimum_required(VERSION 3.10)
project(ar_interface)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubicTrajParams.msg"
  "msg/CubicTrajCoeffs.msg"
  "srv/ComputeCubicTraj.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()