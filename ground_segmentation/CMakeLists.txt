cmake_minimum_required(VERSION 3.8)
project(ground_segmentation)

# Use modern C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Core Ground Segmentation Library & Component Node
ament_auto_add_library(ground_segmentation SHARED
  src/ground_segmentation_node.cpp
)
target_link_libraries(ground_segmentation Eigen3::Eigen)

rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::GroundSegmentationNode"
  EXECUTABLE ground_segmentation_node
)

# Install launch directory and headers cleanly
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
)

