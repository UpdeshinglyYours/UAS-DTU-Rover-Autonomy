cmake_minimum_required(VERSION 3.8)
project(fcb_orin_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "srv/SetZoom.srv"
  "srv/SetImagingMode.srv"
  "srv/StopRecording.srv"
  "srv/RenameRecording.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
