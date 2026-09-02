cmake_minimum_required(VERSION 3.8)
project(dtu_prior_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dtu_reference_route_publisher.py
  scripts/dtu_path_metrics.py
  scripts/generate_centerline_navigation.py
  scripts/generate_nav2_maps.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
