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

After the Blickfeld driver and MAVROS are available, start the runtime in this
order in separate sourced terminals:

```bash
ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py use_sim_time:=false
ros2 launch patchwork_plusplus patchwork_plusplus.launch.py use_sim_time:=false
ros2 launch lirovo lirovo.launch.py use_sim_time:=false
python3 /home/aryaman/Desktop/new_omega_fixed5.py
```

GenZ publishes `/bf_lidar/point_cloud_deskewed` and `/genz/odometry`.
Patchwork++ supplies the `/nonground` input required by LiRovo. LiRovo starts
the EKF, converts `/nonground` to `/scan`, starts SLAM Toolbox and Nav2, and
uses `/scan` for obstacle marking and clearing in both costmaps. Nav2 publishes
the smoothed command on `/cmd_vel`. `new_omega_fixed5.py` consumes that command
and publishes MAVROS RC overrides on `/mavros/rc/override`.

Once all four stages are healthy, send the final goal as an external Nav2
action command. Goal submission is deliberately not part of any launch file:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 170.0, y: 202.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

Replace the position and orientation with the actual mission final pose. Nav2
uses its standard periodically replanning NavigateToPose behavior tree.

The pointcloud-to-laserscan input and output can be overridden when needed:

```bash
ros2 launch lirovo lirovo.launch.py \
  input_cloud_topic:=/nonground \
  output_scan_topic:=/scan \
  use_sim_time:=false
```

The pointcloud-to-laserscan range starts at `0.43` m and ends at `35.0` m.
The height filter is `0.0` to `2.0` m in `base_link`. Flat ground should
already have been removed by Patchwork++ before this filter.

### DTU prior-map localization with single-antenna RTK

The default `global_localization:=slam` mode above is unchanged. The opt-in
`dtu_rtk` mode keeps the existing local EKF for `odom -> base_link`, disables
SLAM Toolbox, and starts a second EKF for the global map-pose estimate. The
global EKF has `publish_tf: false`. `manual_global_alignment` is the sole owner
of `map -> odom`; the local EKF remains the sole owner of
`odom -> base_link`. The global EKF receives DTU-map X/Y from
`navsat_transform_node`, differential GenZ motion, and absolute MAVROS yaw. It
does not fuse `/mavros/local_position/odom`. `base_link` is already REP-103
(+X forward and +Y left), so the single static
`base_link -> base_footprint` transform has zero yaw.

Raw GPS never reaches `navsat_transform_node` directly. `rtk_fix_gate` accepts
only `GPSRAW.GPS_FIX_TYPE_RTK_FIXED` (`6`) after ten consecutive samples,
reported horizontal variance no greater than `0.25 m²`, fresh status/fix
timestamps, and (after an existing manual or RTK anchor) a re-entry error no
greater than `2 m`. RTK Float (`5`), 3D Fix, stale, and poor-accuracy data are
not republished. Loss of Fixed closes the gate without resetting either EKF.

Before changing parameters, record one stationary-and-straight-line dataset.
Do not run the RC override bridge or send a navigation goal during this test:

```bash
ros2 bag record -o dtu_rtk_validation \
  /mavros/global_position/raw/fix \
  /mavros/imu/data \
  /mavros/local_position/odom \
  /mavros/gpsstatus/gps1/rtk \
  /mavros/gps_rtk/rtk_baseline \
  /genz/odometry \
  /tf /tf_static
```

Start the DTU localization prototype with Nav2 inactive:

```bash
ros2 launch lirovo lirovo.launch.py \
  use_sim_time:=false \
  global_localization:=dtu_rtk \
  dtu_nav2_autostart:=false \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0
```

#### Manual map alignment without RTK Fixed

Keep Nav2 inactive, open RViz with Fixed Frame `map`, select **2D Pose
Estimate**, click the rover's actual map position, drag the arrow along the
rover's actual forward direction, and release. The
`manual_global_alignment` node looks up the current `odom -> base_link`, logs
the implied transform

```text
T_map_odom = T_map_base_clicked * inverse(T_odom_base_current)
```

and calls the supported `robot_localization/srv/SetPose` service at
`/ekf_filter_node_map/set_pose`. It stores the resulting `map -> odom` and
repeats that exact numerical transform at 30 Hz. The timer never integrates or
recomputes the transform, so GenZ, IMU, local-EKF, and global-EKF frequency
differences cannot move it. It is sent on `/tf`, rather than `/tf_static`, only
because a later accepted RTK-Fixed correction is allowed to replace its X/Y.
The yaw remains fixed until another deliberate manual alignment. The node does
not reset the local EKF. Verify the result before activating Nav2:

```bash
ros2 topic echo /dtu_global_localization/status --once --full-length
ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

The status topic is a transient-local `std_msgs/msg/String` containing JSON.
Its `state` is one of `UNINITIALIZED`, `MANUAL_INITIALIZED`, `RTK_PENDING`, or
`RTK_ACTIVE`; it also reports fix type, consecutive Fixed count, gate state,
quality, manual initialization, re-entry distance, and the current rejection
reason. A manual click is accepted only when its map-frame timestamp is fresh,
the local TF exists, and the global set-pose service is available. The clicked
yaw is passed intact to the EKF; continuing absolute IMU yaw fusion assumes the
IMU heading agrees with the physically observed forward direction.

When the RTK gate is closed, `/odometry/gps` receives no new data and the
stored `map -> odom` cannot change. Each accepted gated RTK update may correct
only its X/Y translation using the timestamp-matched `odom -> base_link` TF.
If RTK Fixed is lost, the last accepted transform is held exactly. No TF-owner
handoff occurs in any state.

The manual datum in `dtu_prior_map/config/navsat_datum.yaml` is:

```text
[28.7451419213, 77.1119513181, -0.0177330185]
```

The yaw is the negative UTM zone 43 grid convergence at the map origin. It
cancels the convergence added internally by `navsat_transform_node`, making
the transformed GPS position agree with the source EPSG:32643 map coordinates:

```text
map_x = UTM_easting  - 706231.750
map_y = UTM_northing - 3181578.500
```

The global EKF starts with `1e6 m²` X/Y variance, so the first gated RTK-Fixed
measurement can initialize its absolute map position when no manual anchor
exists. It can briefly publish near `(0,0)` before either anchor is accepted;
this is why `dtu_nav2_autostart` must remain `false`.

Replay the August 25 inputs at real-time speed with a 100 Hz bag clock. Select
only the live data sources shown below: do not replay the bag's recorded `/tf`
or `/odometry/filtered`, because the current pipeline owns those interfaces.

```bash
ros2 bag play ~/Desktop/aug25.bag/aug25.bag_0.db3 \
  --clock 100 --rate 1.0 --disable-keyboard-controls \
  --topics /genz/odometry \
           /mavros/global_position/raw/fix \
           /mavros/gpsstatus/gps1/raw \
           /mavros/imu/data
```

Before any live Nav2 activation, complete all of these readiness checks:

- The measured GPS antenna phase centre relative to `base_footprint` is
  `[0.10, 0.05, 0.40] m`: 10 cm forward, 5 cm left, and 40 cm up. The DTU RTK
  launch publishes this exactly once as `base_footprint -> gps_antenna`.
- Load `config/mavros_dtu_gps_frame.yaml` after the standard MAVROS APM config
  so `/mavros/global_position/raw/fix` has `frame_id: gps_antenna`. Do not apply
  the lever arm in another component.
- Observe ten consecutive RTK-fixed samples with X and Y covariance no greater
  than `0.25 m²`.
- Observe `/odometry/global` within `2 m` of `/odometry/gps` for five
  consecutive fixes.
- Confirm fresh GPS, IMU, GenZ, and TF data and exactly one publisher for each
  dynamic TF edge.
- Manually drive a straight segment of at least `20 m` above `1 m/s`. Retain
  MAVROS yaw fusion only when mean course-versus-yaw error is within `5°` and
  there are no jumps above `10°`.

The following commands provide the read-only pose, TF, and lifecycle evidence:

```bash
ros2 topic echo /mavros/global_position/raw/fix --once
ros2 topic hz /mavros/global_position/raw/fix
ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_footprint
ros2 run tf2_ros tf2_echo base_footprint gps_antenna
ros2 topic info -v /tf
ros2 service call /dtu_static_map_server/get_state \
  lifecycle_msgs/srv/GetState "{}"
ros2 service call /dtu_centerline_cost_server/get_state \
  lifecycle_msgs/srv/GetState "{}"
ros2 service call /planner_server/get_state lifecycle_msgs/srv/GetState "{}"
ros2 service call /controller_server/get_state lifecycle_msgs/srv/GetState "{}"
```

Start MAVROS with the GPS-frame override last so it wins parameter precedence;
replace `<CURRENT_FCU_URL>` with the connection string already used on the
rover:

```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=<CURRENT_FCU_URL> \
  --params-file /opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml \
  --params-file /opt/ros/humble/share/mavros/launch/apm_config.yaml \
  --params-file "$(ros2 pkg prefix lirovo)/share/lirovo/config/mavros_dtu_gps_frame.yaml"

ros2 param get /mavros/global_position frame_id
ros2 topic echo /mavros/global_position/raw/fix \
  --field header.frame_id --once
```

The map servers must report `active`; the planner and controller must remain
`unconfigured` or `inactive` until every gate above passes. Confirm MAVROS yaw
is ENU and earth-referenced (zero east, positive counter-clockwise). Correct the
heading source rather than hiding an IMU error with a geographic offset.

After those checks pass, activate the inactive Nav2 nodes explicitly:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Autonomous goals and physical actuation remain out of scope until the checks
pass and a bounded test area, operator, stop method, and speed limit are
explicitly authorized.

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

Terminal 3 (pointcloud-to-laserscan, robot_localization, SLAM Toolbox, and
Nav2):

```bash
ros2 launch lirovo lirovo.launch.py \
  input_cloud_topic:=/nonground \
  output_scan_topic:=/scan \
  use_sim_time:=true
```

Terminal 4:

```bash
python3 /home/aryaman/Desktop/new_omega_fixed5.py --ros-args \
  -p use_sim_time:=true
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

Both Nav2 costmaps mark and clear obstacles from `/scan`.

    
    
       
