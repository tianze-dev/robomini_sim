cmake_minimum_required(VERSION 3.8)
project(robomini_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionToGimbal.msg"
  "msg/GimbalToVision.msg"
  "msg/RuneState.msg"
  "msg/RuneHit.msg"
  "msg/ScoreBoard.msg"
  "srv/SetRuneMode.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
