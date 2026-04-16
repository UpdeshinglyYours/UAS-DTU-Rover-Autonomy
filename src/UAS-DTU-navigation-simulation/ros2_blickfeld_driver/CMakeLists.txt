cmake_minimum_required(VERSION 3.13)
project(blickfeld_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(diagnostic_updater REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(blickfeld-scanner 2.20.6 REQUIRED full)

set(dependencies
  blickfeld-scanner
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
set(srv_files
  "srv/ImuStaticTransformation.srv"
  "srv/SetScanPattern.srv"
)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

set(${PROJECT_NAME}_INCLUDE_DIRS
  "include/;modules/ros_${PROJECT_NAME}_core/include/"
)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)

include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(/opt/ros/humble/include/tf2)
include_directories(/opt/ros/humble/include/tf2_ros)
include_directories(/opt/ros/humble/include/cv_bridge)
include_directories(/opt/ros/humble/include/rclcpp)
include_directories(/opt/ros/humble/include/rcl)
include_directories(/opt/ros/humble/include/rcl_yaml_param_parser)
include_directories(/opt/ros/humble/include/rcpputils)
include_directories(/opt/ros/humble/include/tracetools)
include_directories(/opt/ros/humble/include/rcl_interfaces)
include_directories(/opt/ros/humble/include/libstatistics_collector)
include_directories(/opt/ros/humble/include/statistics_msgs)
include_directories(/opt/ros/humble/include/diagnostic_msgs)
include_directories(/opt/ros/humble/include/rclcpp_components)
include_directories(/opt/ros/humble/include/class_loader)

################################################
## Submodules ##
################################################
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/modules/ros_${PROJECT_NAME}_core")

# driver lib
add_library(blickfeld_driver_lib
  src/blickfeld_driver.cpp
)
target_link_libraries(blickfeld_driver_lib
  blickfeld_driver_core
  blickfeld-scanner
)

ament_target_dependencies(blickfeld_driver_lib
  ${dependencies}
)

# link to just generated srv msg
#see Also https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071
rosidl_target_interfaces(
  blickfeld_driver_lib
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# driver as component
add_library(blickfeld_driver_component
  src/blickfeld_driver_component.cpp
)
target_compile_definitions(blickfeld_driver_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(blickfeld_driver_component
  blickfeld_driver_lib
)
rclcpp_components_register_nodes(blickfeld_driver_component "blickfeld::ros_interop::BlickfeldDriverComponent")
set(node_plugins "${node_plugins}blickfeld_driver_component;$<TARGET_FILE:node_component>\n")

ament_target_dependencies(blickfeld_driver_component
  ${dependencies}
)
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

# driver as node
add_executable(blickfeld_driver_node
  src/blickfeld_driver_node.cpp
)
target_link_libraries(blickfeld_driver_node
  blickfeld_driver_lib
)
ament_target_dependencies(blickfeld_driver_node
  ${dependencies}
)

install(TARGETS blickfeld_driver_lib blickfeld_driver_component blickfeld_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRS}
  DESTINATION include/
)

# Install launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
# Install config files
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(${${PROJECT_NAME}_INCLUDE_DIRS})
ament_export_libraries(blickfeld_driver_core)
ament_export_dependencies(${dependencies})
ament_package()
