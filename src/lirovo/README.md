# 🛠️ Setup Instructions

### Create a workspace and add an src folder to it

``` mkdir nav2_ws/src && cd src/```

### Clone the repo

```git clone https://github.com/AkshatKaushal25/UAS-DTU-navigation-simulation.git . ```

### head back to root directory

```cd ..```

### Install Dependencies 

```rosdep install -y --from-paths ./src --ignore-src```

### Build the directory 

```colcon build --symlink-install```


# 🛠️ Lidar Setup Instructions

### Installing Dependencies

1. **Blickfeld scanner library**  
   ```sudo apt update
      sudo apt update
      bash sudo apt install -y wget libprotobuf-dev libprotobuf23 
      wget https://github.com/Blickfeld/blickfeld-scanner-lib/releases/latest/download/blickfeld-scanner-lib-dev-testing-Linux.deb  
      sudo dpkg -i blickfeld-scanner-lib-dev-testing-Linux.deb``` 

2. **Diagnostic updater**  
    ```sudo apt install ros-humble-diagnostic-updater 
       sudo apt install ros-humble-diagnostic-updater 
       sudo apt install ros-humble-diagnostic-msgs```

### Build and Source the package
### Run The Node

```RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=192.168.26.26 --remap __node:=bf_lidar -p publish_imu:=true -p publish_imu_static_tf_at_start:=true```

# 🛠️ Lirovo Setup Instructions

### Main launch command

The live perception and navigation stack is started in this order after the
Blickfeld driver and MAVROS are available:

```bash
ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py use_sim_time:=false
ros2 launch patchwork_plusplus patchwork_plusplus.launch.py use_sim_time:=false
ros2 launch lirovo lirovo.launch.py use_sim_time:=false
```

Run each command in a separate sourced terminal. The GenZ pipeline publishes
`/bf_lidar/point_cloud_deskewed` and `/genz/odometry`. Patchwork++ splits the
deskewed cloud into `/ground` and `/nonground`. The `lirovo` launch converts
`/nonground` to `/scan` for slam_toolbox and sends `/nonground` directly to
the local and global STVL costmap layers. Nav2 does not use `/scan` as an
obstacle source.

The pointcloud-to-laserscan input and output can be overridden when needed:

```bash
ros2 launch lirovo lirovo.launch.py \
  input_cloud_topic:=/nonground \
  output_scan_topic:=/scan \
  use_sim_time:=false
```

The starting height filter is `-0.10` to `1.50` m in `base_footprint`. Flat
ground should already have been removed by Patchwork++ before this filter.

### Initialiase LiDAR

```RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=192.168.26.26 --remap __node:=bf_lidar -p publish_imu:=true -p publish_imu_static_tf_at_start:=true```

### Launch SITL

```sim_vehicle.py -v Rover --console --map```

### Launch mavros

```ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14550@ -p baud_rate:=57600```

## Bag validation

The supplied bag can exercise Patchwork++ directly from the raw Blickfeld
cloud. It must not be expected to produce a new deskewed cloud unless the
deskew pipeline and its required IMU input are also played and launched.

Terminal 1:

```bash
ros2 bag play ~/19july.bag --clock --loop \
  --topics /bf_lidar/point_cloud_out /tf /tf_static
```

Terminal 2:

```bash
ros2 launch patchwork_plusplus patchwork_plusplus.launch.py \
  input_cloud_topic:=/bf_lidar/point_cloud_out \
  use_sim_time:=true
```

Terminal 3 (pointcloud-to-laserscan, robot_localization, slam_toolbox, and
Nav2):

```bash
ros2 launch lirovo lirovo.launch.py \
  input_cloud_topic:=/nonground \
  output_scan_topic:=/scan \
  use_sim_time:=true
```

Terminal 4:

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Check the data flow and loaded Patchwork++ parameters with:

```bash
ros2 topic hz /ground
ros2 topic hz /nonground
ros2 topic hz /scan
ros2 topic echo /ground --once --field width
ros2 topic echo /nonground --once --field width
ros2 topic echo /nonground --once --field header.frame_id
ros2 param get /patchwork_plusplus input_cloud_topic
ros2 param get /patchwork_plusplus sensor_height
ros2 param get /patchwork_plusplus min_range
ros2 param get /patchwork_plusplus max_range
ros2 param get /patchwork_plusplus enable_RNR
```

The expected Patchwork++ values are `0.60`, `0.30`, `30.0`, and `false` for
sensor height, minimum range, maximum range, and RNR respectively.

Patchwork++ preserves the cloud timestamp and frame. Discover that frame and
validate every required TF path without adding substitute transforms:

```bash
ros2 topic echo /bf_lidar/point_cloud_deskewed --once --field header.frame_id
ros2 topic echo /nonground --once --field header.frame_id
ros2 run tf2_ros tf2_echo base_footprint <cloud_frame>
ros2 run tf2_ros tf2_echo odom <cloud_frame>
ros2 run tf2_ros tf2_echo map <cloud_frame>
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

STVL clearing remains disabled. Enable it only after physically validating
the Blickfeld horizontal and vertical FOV, LiDAR orientation, sensor origin,
and all of the TF paths above.

    
    
       
