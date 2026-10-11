cmake_minimum_required(VERSION 3.8)
project(patchwork_plusplus)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jsk_recognition_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io visualization filters)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        include
        SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(patchwork_plusplus_src
        src/patchwork_plusplus_node.cpp
        )

ament_auto_add_library(patchwork_plusplus SHARED
        ${patchwork_plusplus_src}
        )

if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
    find_package(Qhull REQUIRED)
    target_link_libraries(patchwork_plusplus
            QHULL::QHULL
            )
endif()

target_link_libraries(patchwork_plusplus
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

rclcpp_components_register_node(patchwork_plusplus
        PLUGIN "patchwork_plusplus::PatchworkPlusPlus"
        EXECUTABLE patchwork_plusplus_exe
        )

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_auto_package()
