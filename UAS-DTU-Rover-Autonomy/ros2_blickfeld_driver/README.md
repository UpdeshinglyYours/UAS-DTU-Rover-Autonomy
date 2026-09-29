# Blickfeld ROS2 package

This package provides a ROS2 node for publishing PointCloud2 messages from Blickfeld LiDAR devices.
The driver is available for download under https://www.blickfeld.com/resources.

## Supported devices

The Blickfeld ROS2 driver supports all available Blickfeld LiDARs such as Cube 1 and Cube Range 1.

## Supported ROS2 Distributions

The Blickfeld ROS2 driver supports the following ROS2 distribution:

- Humble

## Dependencies

Please install the following dependencies on your system in advance.

- [blickfeld-scanner-library (BSL) ](https://docs.blickfeld.com/cube/latest/external/blickfeld-scanner-lib/install.html) with system-wide protobuf installation. This package has been tested using **BSL Version 2.18.2** (minimum version). Ubuntu 22.04 only has protobuf23 in apt repositories. 

- [ROS Humble Installation](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Install-Debians/) with Ubuntu 22.04
- [diagnostic_updater](https://index.ros.org/p/diagnostic_updater/) can be acquired via your distribution's package manager, ${ROS_DISTRO} should be your ROS2 version (which is Foxy Fitzroy or shorten foxy).

      $ sudo apt install ros-${ROS_DISTRO}-diagnostic-updater
      $ sudo apt install ros-${ROS_DISTRO}-diagnostic-msgs

  or via

      $ rosdep update
      $ rosdep install --from-paths src --ignore-src -r -y --skip-keys "blickfeld-scanner"

  The -skip-keys will instruct rosdep not to check blickfeld-scanner, which is not a ROS2 package.

- [colcon](https://colcon.readthedocs.io/en/released/user/installation.html): required for building the workspace

The default DDS of ROS2 Humble FastRTPS is not compatible with BSL.
Hence, for ROS2 Humble, you need to install DDS from other vendors. (e.g., Eclipse Cyclone or RTI Connext)

- [Eclipse Cyclone DDS](https://index.ros.org/doc/ros2/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS/): to bypass the default DDS of ROS2 Humble(FastRTPS)

## Build

Before building, please source your ROS2 distribution. You will need to run this command on every new shell you open to have access to the ROS2 commands.

    $ source /opt/ros/${ROS_DISTRO}/setup.bash

Create your colcon workspace:

    $ mkdir -p ${colcon_workspace}/src

where ${colcon_workspace} is your custom directory. Move the ros2_blickfeld_driver package into the `{colcon_workspace}/src` directory and build your workspace:

    $ cd ${colcon_workspace}
    $ colcon build --symlink-install --cmake-clean-first

## Running the Blickfeld ROS2 node

### Switching to desired DDS

:bangbang: **To run Blickfeld Driver as ROS2 node, you need to switch from the default DDS (FastDDS) to a compatible one**. :bangbang:

This can be done using **either of the two** approaches.

1. Exporting the RMW_IMPLEMENTATION for the present terminal session

```
   $ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

2. [Switching to the desired RMW_IMPLEMENTATION](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/) while running by adding a prefix before the ROS2 run

```
   $ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node ARGUMENTS
```

where `ARGUMENTS` is `--ros-args` and list of parameters to be used

**NOTE**

Before starting any communication with blickfeld driver ROS2 node, the default DDS should be changed to cyclone DDS or other vendors. Therefore, it is suggested to set a system global variable.

### Using ROS2 run

Before running, ensure that your colcon workspace is sourced. This is achieved by sourcing the setup script in your colcon workspace.

    $ source ${colcon_workspace}/install/setup.bash

You can launch the ROS2 driver in a DHCP controlled network by providing the hostname of the Cube sensor (you can check and set the hostname in the WebGUI of the device) e.g, cube-XXXXXXXXX, where XXXXXXXXX must be replaced by the 9-digit serial number of the Cube:

    $ ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=cube-XXXXXXXXX --remap __node:=bf_lidar

Alternatively, you can enter the IP address of the Cube instead of the hostname.

You can also start the driver with more parameters, e.g.:

    $ ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=cube-XXXXXXXXX -p publish_ambient_light:=true -p publish_intensities:=false

To visualize the published data, run rviz2:

    $ rviz2

### Using Python API

If you prefer to put the parameters into a yaml file, run the python launch file. It reads the config files in the blickfeld_driver package ros2_blickfeld_driver/config. There are two yaml files: 1) driver_config.yaml: configures host id, output point cloud topic etc. 2) blickfeld_scanner.rviz:
configures the parameters for Rviz visualization, e.g. topic of point cloud, configuration of opened viewers etc. To run it with the python API and yaml file execute the following command:

    $ ros2 launch blickfeld_driver live_scanner_node.launch.py

### Using XML File

In case you prefer a similar way to ROS1 launch, you can start the driver as follows:

    $ ros2 launch -a blickfeld_driver live_scanner.launch.xml host:=cube-XXXXXXXXX

## Running the Blickfeld ROS2 component

Before running, ensure that your colcon workspace is sourced. This is achieved by sourcing the setup script in your colcon workspace.

    $ source ${colcon_workspace}/install/setup.bash

### Using ROS2 run

You can launch the Blickfeld ROS2 component in a DHCP controlled network by providing the hostname of the LiDAR device (you can check and set the hostname in the WebGUI of the device) e.g. cube-XXXXXXXXX, where XXXXXXXXX is the 9-digit serial number:

    $ ros2 component standalone blickfeld_driver blickfeld::ros_interop::BlickfeldDriverComponent -p host:=cube-XXXXXXXXX -p publish_ambient_light:=true -p publish_intensities:=false

To visualize the published data, run rviz2:

    $ rviz2

### Using Python API

If you prefer to put the parameters into a yaml file, run the python launch file. It reads the config files in the blickfeld_driver package ros2_blickfeld_driver/config. There are two yaml files: 1) driver_config.yaml: configures host id, output point cloud topic etc. 2) blickfeld_scanner.rviz:
configures the parameters for Rviz visualization, e.g. topic of point cloud, configuration of opened viewers etc. To run it with the python API and yaml file execute the following command:

    $ ros2 launch blickfeld_driver live_scanner_component.launch.py

## Parameters

All the following parameters are available in launch _and_ run:

| Argument                                         | Default                                | Note                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ------------------------------------------------ | -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **host** (required)                              |                                        | The hostname or the IP address of the Cube sensor you want to publish the point clouds from, e.g., `cube-XXXXXXXXX`.                                                                                                                                                                                                                                                                                                                                                                                                        |
| lidar_frame_id                                   | `lidar`                                | The ROS TF where the point cloud should be in.                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| remap                                            | `true`                                 | Remap this node’s input/output topics to commonly used ones. <br/>If `false`, canonical names in the form of `$(arg node_name)/foo*(in/out)`are used, depending on whether a topic is an input or an output topic. This option only works when launching the driver using the xml file.                                                                                                                                                                                                                                                                                                          |
| publish_ambient_light                            | `false`                                | Set to `true` if the PointCloud2 message should contain the ambient light level.                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| publish_explicit_range                           | `false`                                | Set to `true` if the PointCloud2 message should contain the`range` field.                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| publish_intensities                              | `false`                                | Set to `true` if the PointCloud2 message should contain intensities.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| publish_no_return_points                         | `false`                                | Set to `true` if the PointCloud2 message should contain points with a fixed range for pulses without a return. The fixed range can be configured using the `no_return_point_range` parameter.                                                                                                                                                                                                                                                                                                                               |
| returns_publishing_options                       | `strongest`                            | Different options to publish multiple returns, possible values are: `strongest`, `closest`, `farthest`, and `all`, in case of `all`, the field `return_id` (ID for each returned point) is added to the point cloud. The `return_id` field is absent from the point cloud when the `returns_publishing_options` value is set to `strongest`, `closest` or `farthest`.                                                                                                                                                       |
| publish_point_id                                 | `true`                                 | Outputs a frame-global id per laser pulse. If the device is configured to return multiple returns (multiple reflections) per pulse, point_id will be identical for all returns that belonged to the same laser pulse; In this case the `return_id` will differentiate the multiple returns from a laser pulse. |
| publish_point_time_offset                        | `false`                                | If set to `true`, the PointCloud2 message will contain the timestamp field per point, which represents the time offset from the start of the frame.                                                                                                                                                                                                                                                                                                                                                                         |
| no_return_point_range                            | `1.0`                                  | Set the fixed range for points that are output despite the laser pulse did not cause a return to be detected.                                                                                                                                                                                                                                                                                                                                                                                                               |
| projection_type                                  | `angle_preserving`                     | Different options to project the data onto a 2D image, possible values are: `angle_preserving` or `scanline_preserving`. These indicate if a correct spherical projection is desired, or each row of the image should correspond to one scanline. Supported scan patterns for `angle_preserving` are `ONLY_UP`, `ONLY_DOWN`, and `COMBINE_UP_DOWN`. Supported scan patterns for `scanline_preserving` are `ONLY_UP`, and `ONLY_DOWN`.                                                                                                                                                                                                                                                                  |
| publish_ambient_image                            | `false`                                | Set to `true` to publish an ambient light image. Depending on projection_type specific scan patterns are supported. Please check projection_type for more detail.                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| publish_intensity_image                          | `false`                                | Set to `true` to publish an intensity image. In case multiple return is activated in cube, intensity image is created based on strongest return. Depending on projection_type specific scan patterns are supported. Please check projection_type for more detail.                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| publish_range_image                              | `false`                                | Set to `true` to publish a range image. In case multiple return is activated in cube, range image is created based on strongest return. Depending on projection_type specific scan patterns are supported. Please check projection_type for more detail.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| publish_point_id_image                           | `true`                                 | Set to `true` to publish an image with point ids. Depending on projection_type specific scan patterns are supported. Please check projection_type for more detail.                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| imu_acceleration_unit                            | `g`                              | Different imu acceleration units: `g`, `meter_per_second_squared`.                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| publish_imu                                      | `false`                                | Set to `true` to publish IMU data in burst mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| publish_imu_static_tf_at_start                   | `false`                                | Call "Publish IMU tf transform" service once at the beginning. This service publishes the IMU data from device as static TF transformation.                                                                                                                                                                                                                                                                                                                                                                                 |
| use_lidar_timestamp                              | `true`                                 | Set to `true` if the timestamp in the ROS2 point cloud message should be generated from the device timestamp; otherwise the timestamp will represent the ROS2 time when the frame was received on ROS2.                                                                                                                                                                                                                                                                                                                     |
| logging_throttled_interval                        | `10000`                                 | The duration of the throttle interval of logging as an integral value in milliseconds. By default, the driver publishes logging messages every 10 seconds. |
| use_background_subtraction                       | `false`                                | Enables on-device background subtraction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| use_neighbor_filter                              | `false`                                | Enables on-device neighbor filter.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| background_subtraction_exponential_decay_rate    | `0.005`                                | Exponential decay factor. Controls how fast objects switch between foreground and background when background subtraction is enabled.                                                                                                                                                                                                                                                                                                                                                                                        |
| background_subtraction_num_initialization_frames | `10`                                   | Number of frames to initialize the background subtraction method with.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ambient_image_out                                | `$(arg node_name)/ambient_image_out`   | Topic to publish the ambient image on.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| diagnostic_out                                   | `$(arg node_name)/diagnostic`          | Topic to publish the diagnostic status.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| imu_out                                          | `$(arg node_name)/imu`                 | Topic to publish the IMU burst data.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| intensity_image_out                              | `$(arg node_name)/intensity_image_out` | Topic to publish the intensity image on.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| point_cloud_out                                  | `$(arg node_name)/points_raw`          | Topic to publish the PointCloud2 message to.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| range_image_out                                  | `$(arg node_name)/range_image_out`     | Topic to publish the range image on.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| point_id_image_out                               | `$(arg node_name)/point_id_image_out`  | Topic to publish the point id image on.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

## Services

The Blickfeld ROS2 driver advertises different services. If blickfeld_driver is running with ROS2 node, for service call and communication change the [default DDS](#switching-to-desired-dds) from FastRTPS.

### Set scan pattern

Before running the command, ensure that your colcon workspace is sourced. This service call will set the scan pattern on a Cube sensor.

e.g. `ros2 service call /$(arg node_name)/set_scan_pattern blickfeld_driver/srv/SetScanPattern "{fov_horizontal: 72.0, fov_vertical: 30.0, angle_spacing: 0.4, scanlines_up: 28, scanlines_down: 28, frame_mode: 'COMBINE_UP_DOWN', pulse_type: 'INTERLEAVE'}" `

### Publish IMU tf transform

This service call will receive the static IMU data from a Cube sensor and create a frame from the data with the frame_id `$(arg lidar_frame_id)_imu`.

`ros2 service call /$(arg node_name)/publish_imu_static_tf_service blickfeld_driver/srv/ImuStaticTransformation`

## License

This package is released under a [BSD 3-Clause License](LICENSE) (see also [https://opensource.org/licenses/BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause))
