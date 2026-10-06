cmake_minimum_required(VERSION 3.8)
project(lightwarelidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  include/linux
)

add_library(lwNx src/lwNx.cpp)
add_library(platformLinux src/linux/platformLinux.cpp)
add_library(serialPortLinux src/linux/lwSerialPortLinux.cpp)

# SF45/B executable
add_executable(sf45b src/sf45b.cpp)
target_link_libraries(sf45b
  lwNx
  platformLinux
  serialPortLinux
)
ament_target_dependencies(sf45b
  rclcpp
  sensor_msgs
)

install(TARGETS
  lwNx
  platformLinux
  serialPortLinux
  sf45b
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
