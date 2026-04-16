# Blickfeld ROS Core package

Independent from ROS1 and ROS2, this package provides the core functionality of creating PointCloud2, Image , Imu message from Blickfeld LiDAR devices. This package should be bundled with [ros_blickfeld_driver](https://gitlab.muc.blickfeld.com/enduser-software/ros_blickfeld_driver/) or [ros2_blickfeld_driver](https://gitlab.muc.blickfeld.com/enduser-software/ros2_blickfeld_driver/).

## Dependencies and Build

This package depends on a header file as a wrapper for ros message headers, to handle ROS1 and ROS2 different message header files. The header file should be available under "blickfeld_driver/blickfeld_driver_ros_types.h". This is not a stand-alone package and it does not have any other dependencies. In order to build this package, include cmake of this package in the top level package(ros1_blickfeld_driver or ros2_blickfeld_driver).

## License

This package is released under a [BSD 3-Clause License](LICENSE) (see also [https://opensource.org/licenses/BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause))
