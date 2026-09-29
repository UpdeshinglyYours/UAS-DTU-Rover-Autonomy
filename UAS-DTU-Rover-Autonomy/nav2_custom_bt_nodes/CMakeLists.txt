cmake_minimum_required(VERSION 3.5)
project(nav2_custom_bt_nodes)

#Default to C++14/17

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
rclcpp
behaviortree_cpp_v3
nav2_behavior_tree
geometry_msgs
nav_msgs
tf2
tf2_geometry_msgs
)

#1. Build the shared library for our 3 custom nodes

add_library(nav2_custom_bt_nodes SHARED plugins/casualty_bt_nodes.cpp)

#2. Link dependencies

ament_target_dependencies(nav2_custom_bt_nodes ${dependencies})

#3. Export macro required by BehaviorTree.CPP factory

target_compile_definitions(nav2_custom_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

#4. Install rules for C++ binaries

install(TARGETS nav2_custom_bt_nodes
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)

# ==============================================================================

# 5. NEW: Install rules for Behavior Tree XML files!

# (This assumes you put your XML files in a folder called 'behavior_trees' or 'config'

# inside this package. Change "config" below to whatever folder your XML is actually in.)

# ==============================================================================

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(nav2_custom_bt_nodes)
ament_export_dependencies(${dependencies})

ament_package()