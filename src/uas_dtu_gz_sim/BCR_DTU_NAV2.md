# BCR Bot + DTU prior map + Nav2 simulation

The centreline-guided extension is documented in
[CENTERLINE_NAVIGATION.md](CENTERLINE_NAVIGATION.md). The legacy `0/70/100`
road filter documented below remains available with
`enable_centerline_tether:=false enable_road_preference:=true`.

## Verified integration inputs

The BCR description is the locally modified `bcr_bot` package in
`~/q_ws/src/bcr_bot`. The autonomy checkout deliberately does not duplicate
that package.

| Item | Verified value |
| --- | --- |
| Simulator | modern Gazebo Sim Harmonic, `gz-sim8` 8.14.0 |
| Existing BCR world launch | `bcr_bot/launch/gz.launch.py` |
| Empty world | `bcr_bot/worlds/empty.sdf` (`empty_world`) |
| Warehouse world | `bcr_bot/worlds/small_warehouse.sdf` |
| Robot model | `bcr_bot/urdf/bcr_bot.xacro` including `gz.xacro` |
| Spawn mechanism | `ros_gz_sim create` from `/robot_description` |
| Gazebo drive input | `/cmd_vel`, `gz.msgs.Twist` |
| Existing ROS drive input | `/bcr_bot/cmd_vel`, `geometry_msgs/msg/Twist` |
| Integrated Nav2 drive path | ROS `/cmd_vel` directly bridged to Gazebo `/cmd_vel` |
| Existing ROS odometry | `/bcr_bot/odom`, `nav_msgs/msg/Odometry` |
| Existing odometry TF | Gazebo `OdometryPublisher`: `odom -> base_footprint` |
| Raw Gazebo LiDAR | `/points`, `gz.msgs.LaserScan` |
| Integrated ROS scan | `/bcr_bot/scan`, `sensor_msgs/msg/LaserScan` |
| Raw Gazebo point cloud | `/points/points`, `gz.msgs.PointCloudPacked` |
| Integrated ROS cloud | `/bcr_bot/points`, `sensor_msgs/msg/PointCloud2` |
| Sensor/body TF | `robot_state_publisher`: `base_footprint -> base_link -> sensors` |
| Nav2 source configuration | `bcr_bot/config/nav2_params.yaml` and `lirovo/config/nav2_params.yaml` |
| DTU map launch | `dtu_prior_map/launch/dtu_prior_map.launch.py` |

The older BCR bridge still subscribes to Gazebo `/scan`, but the current BCR
GPU LiDAR was changed to a 32-line sensor with `<topic>points</topic>`. A live
Gazebo probe confirmed that its LaserScan is `/points` and its point cloud is
`/points/points`. The new integration launch corrects that stale bridge mapping
without changing any existing BCR launch.

The repository was already substantially dirty before this integration. Its
tracked diff covered 85 files (854 insertions, 7590 deletions), including
changes under `lirovo`, `genz-icp`, and `patchwork-plusplus-ros2`, plus deleted
ground-segmentation and target-explorer trees. It also had several unrelated
untracked packages and data directories. Both `src/dtu_prior_map` and
`src/uas_dtu_gz_sim` are themselves untracked, so their changes do not appear
in normal `git diff` output. None of those unrelated changes were modified or
cleaned up for this task.

## Initial pose and TF ownership

The first fix in `dtu_gps_tf_sync_bag` is:

```text
latitude  = 28.7486135000
longitude = 77.1170079000
UTM 43N   = E 706718.805800, N 3181972.018522
DTU map   = x 487.055800, y 393.518522
```

This uses the authoritative map origin `E 706231.750, N 3181578.500` from
`dtu_map_origin.yaml`. For comparison, the rounded coordinate in the request
(`28.7486143, 77.1170085`) converts to `(487.112826, 393.608227)`. It does not
convert to `(446.8, 379.2)` with EPSG:32643 and the checked-in origin.

The bag's first raw IMU yaw is `-50.24647 deg`. Applying the synchronized-bag
heading offset already recorded by `dtu_live_map_localizer.py` (`159.2683 deg`)
gives `109.02183 deg`, or `1.902790 rad`. That is the launch default. It remains
configurable; `initial_yaw:=0.0` means map +X/east.

Gazebo spawns at world `(0, 0)` and world odometry starts at approximately
zero. The integration publishes the only `map -> odom` transform as:

```text
translation = (487.055800, 393.518522, 0)
rotation    = yaw 0
```

TF ownership is therefore unambiguous:

```text
static_transform_publisher  map -> odom
Gazebo OdometryPublisher    odom -> base_footprint
robot_state_publisher       base_footprint -> base_link -> sensors
```

No AMCL, SLAM Toolbox, live GPS localizer, or robot_localization node is started
by this launch.

## Build and launch

The BCR workspace uses a symlink install. Rebuild its one required package,
then source that package before building the two affected autonomy packages:

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/q_ws"
colcon build --packages-select bcr_bot --symlink-install
source "$HOME/q_ws/install/bcr_bot/share/bcr_bot/package.bash"

cd "$HOME/UAS-DTU-Rover-Autonomy"
colcon build --packages-select dtu_prior_map uas_dtu_gz_sim --symlink-install
source install/dtu_prior_map/share/dtu_prior_map/package.bash
source install/uas_dtu_gz_sim/share/uas_dtu_gz_sim/package.bash
```

Start the basic static-map test with road preference disabled:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  enable_centerline_tether:=false enable_road_preference:=false
```

RViz is included. To run it separately:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py use_rviz:=false
ros2 run rviz2 rviz2 -d \
  "$(ros2 pkg prefix uas_dtu_gz_sim)/share/uas_dtu_gz_sim/rviz/bcr_bot_dtu_sim.rviz"
```

For a display-less machine or CI run, disable both graphical clients:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  headless:=true use_rviz:=false
```

After the baseline works, restart with the soft road-cost filter:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  enable_centerline_tether:=false enable_road_preference:=true
```

The launch also accepts `initial_map_x`, `initial_map_y`, `initial_yaw`,
`headless`, `use_rviz`, `autostart`, and `use_sim_time`. Keep
`use_sim_time:=true` for this simulation.

## Map and Nav2 behavior

Both costmaps use `/dtu_static_map`. Buildings and the outside boundary are
lethal. The local rolling costmap includes the same static layer, so controller
collision checks still see nearby mapped buildings even though the empty
Gazebo world does not contain them. Both costmaps retain a live LaserScan
obstacle layer for future physical Gazebo obstacles.

The global-only road filter consumes `/dtu_road_mask` through
`/dtu_road_filter_info`. The checked mask contains exactly:

```text
road                0   low cost
off-road ground    70   elevated but traversable cost
building/outside  100   lethal
```

Nav2's Humble `KeepoutFilter` converts the raw occupancy percentages to
costmap values, so 70 remains below the lethal threshold. The static map is the
authoritative hard-obstacle layer; the road mask is only a planning bias.

## Test goals

The following cell-center coordinates were checked directly against both PGM
files. The start and first goal are in the same connected road component.

| Pose | Map `(x, y)` m | Distance from start | Static | Road mask |
| --- | ---: | ---: | ---: | ---: |
| Start | `(487.055800, 393.518522)` | `0 m` | `0` | `0` |
| Road goal | `(517.625, 359.625)` | `45.64 m` | `0` | `0` |
| Slightly off-road goal | `(506.625, 352.625)` | `45.33 m` | `0` | `70` |

Send the road goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 517.625, y: 359.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

Reset the simulation, enable road preference, and send the off-road goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 506.625, y: 352.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Runtime checks

Lifecycle nodes should report `active`:

```bash
for node in dtu_static_map_server dtu_road_mask_server \
  dtu_road_filter_info_server controller_server planner_server \
  smoother_server behavior_server bt_navigator waypoint_follower \
  velocity_smoother; do
  ros2 service call "/$node/get_state" lifecycle_msgs/srv/GetState "{}"
done
```

Inspect maps, TF, sensor data and motion commands:

```bash
ros2 topic echo /dtu_static_map --once
ros2 topic echo /dtu_road_mask --once
ros2 topic echo /dtu_road_filter_info --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /bcr_bot/odom
ros2 topic hz /bcr_bot/scan
ros2 topic hz /cmd_vel
```

Check TF publishers before introducing another localization source:

```bash
ros2 topic info -v /tf
ros2 topic info -v /tf_static
ros2 run tf2_tools view_frames
```

In RViz, verify the robot starts near `(487.06, 393.52)`, the global plan does
not cross lethal static-map cells, and the enabled road filter keeps the road
goal path in low-cost corridors where a road-connected alternative exists.

## Observed runtime results

A headless test of the installed launch produced the following results:

- All ten map/Nav2 lifecycle nodes reached `active`.
- `map -> base_link` resolved near `(487.055, 393.519)` at yaw `1.903 rad`.
- Odometry arrived at 10 Hz; LaserScan and PointCloud2 arrived at about 20 Hz.
- Every observed TF child had one parent. The two `/tf` publishers own disjoint
  edges: Gazebo owns `odom -> base_footprint`, while robot-state-publisher owns
  the robot joints. The two `/tf_static` publishers likewise own disjoint
  edges.
- With road preference disabled, the road-goal path had 133 samples: 20 on
  mask value 0 and 113 on mask value 70. It crossed zero lethal static cells.
- With road preference enabled, the road-goal path had 257 samples: 248 on
  mask value 0 and 9 on mask value 70. It crossed zero lethal static cells.
- The enabled-filter off-road goal reached its exact requested endpoint with
  230 samples: 206 on mask value 0 and 24 on mask value 70, and zero lethal
  static cells.
- During a 15-second navigation sample, `/cmd_vel` ran at about 20 Hz and the
  rover moved 1.28 m before the test goal was deliberately canceled.

In the empty world, LaserScan messages contain no finite returns; that is
expected because there is no physical geometry within sensor range.

## Empty-world limitation

The DTU occupancy grid is a Nav2 planning/collision-checking input; it does not
create Gazebo collision geometry. The rover will plan around mapped buildings,
but simulated LiDAR cannot see them and Gazebo cannot physically collide with
them. Dynamic obstacle avoidance requires adding physical Gazebo objects. No
3D DTU world is generated by this integration.
