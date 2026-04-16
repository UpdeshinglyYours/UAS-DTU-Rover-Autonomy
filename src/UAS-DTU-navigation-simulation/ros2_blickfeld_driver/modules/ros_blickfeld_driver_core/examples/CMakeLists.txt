find_package(Boost 1.65 COMPONENTS program_options REQUIRED)

add_executable(example
  example.cpp
)

target_link_libraries(example
  PRIVATE
  ${PROJECT_NAME}_core
  Boost::program_options
  ${catkin_LIBRARIES}
)
