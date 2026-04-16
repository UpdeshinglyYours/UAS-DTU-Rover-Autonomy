set(${PROJECT_NAME}_CORE_INCLUDE_DIRS
  "include;${PROJECT_SOURCE_DIR}/include"
)

include_directories(
  ${${PROJECT_NAME}_CORE_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${cv_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/blickfeld_driver_core.cpp
  src/blickfeld_driver_imu_parser.cpp
  src/blickfeld_driver_point_cloud_parser.cpp
  src/blickfeld_driver_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_core 
  blickfeld-scanner
  ${cv_bridge_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
