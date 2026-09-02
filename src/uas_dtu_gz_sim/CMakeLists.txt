cmake_minimum_required(VERSION 3.8)
project(uas_dtu_gz_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models rviz validation worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES BARRICADE_COURSE.md BCR_DTU_NAV2.md CENTERLINE_NAVIGATION.md LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
