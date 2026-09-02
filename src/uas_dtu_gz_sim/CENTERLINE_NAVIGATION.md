# Centreline-guided DTU navigation

This simulation keeps two deliberately separate paths:

- `/dtu_reference_route` is the obstacle-independent shortest route over the
  original `roads` LineStrings. It is reliable and transient-local.
- `/plan` is the collision-aware SmacPlanner2D path followed by MPPI. It may
  leave the reference line for LiDAR obstacles, a road blockage, or an
  off-road destination.

The authoritative collision map remains `/dtu_static_map`. The centreline map
is a global-only soft preference; it is not loaded into the local costmap.

## Rebuild the generated prior

The generator uses the four EPSG:32643 layers `roads`, `road_corridors`,
`buildings`, and `operational_boundary`. It rasterizes the source road
LineStrings directly (no skeletonization), normalizes each road cell by its
centre-to-local-edge half-width, and builds a metric graph in the `map` frame.

From the repository root:

```bash
python3 -B src/dtu_prior_map/scripts/generate_centerline_navigation.py \
  src/dtu_prior_map/source/dtu_map.gpkg \
  src/dtu_prior_map \
  --snap-tolerance 1.0
```

This reproducibly writes:

```text
src/dtu_prior_map/maps/dtu_centerline_cost.pgm
src/dtu_prior_map/maps/dtu_centerline_cost.yaml
src/dtu_prior_map/maps/dtu_road_graph.json
```

The raster retains the existing `4218 x 4195`, `0.25 m/cell`, origin
`[0, 0, 0]`, row orientation, and raw ROS occupancy encoding. Its costs are
approximately 0 on the centre, smoothly increase to 45 at each local corridor
edge, are 70 on traversable off-road ground, and are 100 in buildings or
outside the operational boundary.

Required offline Python packages are `python3-gdal`, `python3-numpy`, and
`python3-yaml`. The runtime node additionally uses the standard ROS packages
declared in `dtu_prior_map/package.xml`.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
source "$HOME/q_ws/install/bcr_bot/share/bcr_bot/package.bash"
cd "$HOME/UAS-DTU-Rover-Autonomy"
colcon build --packages-select dtu_prior_map uas_dtu_gz_sim --symlink-install
source install/setup.bash

ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0
```

For headless validation:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  headless:=true use_rviz:=false \
  enable_centerline_tether:=true
```

The preference switches have explicit precedence:

| `enable_centerline_tether` | `enable_road_preference` | Global preference |
| --- | --- | --- |
| `true` | either value | Normalized centreline mask; reference node runs |
| `false` | `true` | Legacy `0/70/100` road mask |
| `false` | `false` | No preference filter |

Only one `KeepoutFilter` instance is active. `headless:=true` and
`use_rviz:=false` remain independent. `cost_travel_multiplier` starts at 3.0
and is a launch argument rather than a hard-coded extreme value.

The supplied RViz configuration shows the reference route as a thick,
partially transparent blue path, `/plan` as a normal green path, and retains
the static map, low-opacity centreline map, both costmaps, LaserScan, robot
model, TF, and odometry.

## Route and replan behaviour

`dtu_reference_route_publisher.py` observes `/plan` only to obtain the
requested start and exact goal. It snaps both endpoints to valid road
segments, selects a connected shortest graph route, adds off-road connectors,
and densifies it at 0.5 m by default. Once published, replans for the same goal
do not alter the reference, so a temporary LiDAR obstacle cannot feed back
into nominal-route selection.

ROS 2 Humble's default
`navigate_to_pose_w_replanning_and_recovery.xml` contains
`RateController hz="1.0"` around `ComputePathToPose`. A live run measured
`/plan` at 0.973 Hz while `/dtu_reference_route` was not republished. Thus
`expected_planner_frequency` is only the server diagnostic; the active
behavior tree is the actual replan trigger.

## Exact validation commands

Run the metrics collector in a second terminal before sending a goal if the
complete initial path is required:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/UAS-DTU-Rover-Autonomy/install/setup.bash"
ros2 run dtu_prior_map dtu_path_metrics.py --ros-args \
  -p timeout_sec:=30.0 -p settle_time_sec:=2.0 \
  -p output_file:=/tmp/dtu_path_metrics.json
```

Test A, clear road:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 517.625, y: 359.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

Test B, restart with the reproducible partial-road box and send the same goal:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  headless:=true use_rviz:=false enable_centerline_tether:=true \
  spawn_test_obstacle:=true

gz model --list
ros2 topic echo --once /bcr_bot/scan
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 517.625, y: 359.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

The default box centre is map `(477.294, 385.465)`, its 2 m side is across the
road, and all dimensions and its pose remain configurable in the launch file.

For the separate four-barricade physical course, see
[BARRICADE_COURSE.md](BARRICADE_COURSE.md). The existing single-box test and
its `spawn_test_obstacle` argument remain unchanged.

Test C, restart without the box and send the off-road goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 506.625, y: 352.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

Additional topic evidence:

```bash
ros2 topic hz /plan
ros2 topic echo --once /dtu_reference_route
ros2 topic echo --once /dtu_centerline_cost
ros2 topic echo --once /global_costmap/costmap
ros2 topic echo --once /local_costmap/costmap
```

## Recorded validation results

These are settled `/plan` snapshots from the installed headless simulation.
Clearance is measured from the path centreline to true costmap-lethal cells
(occupancy 100); occupancy 99 is Nav2's inscribed/inflated band, not lethal.

| Test | Actual / reference length (m) | Within 0.5 / 1 / 2 m | Min clearance (m) | Lethal cells | Road / off-road |
| --- | ---: | ---: | ---: | ---: | ---: |
| A: clear road | 75.237 / 79.998 | 92.32% / 97.70% / 100% | 9.441 | 0 | 100% / 0% |
| B: partial box | 76.308 / 79.998 | 87.16% / 95.20% / 100% | 0.255 | 0 | 100% / 0% |
| C: off-road goal | 69.939 / 76.959 | 86.32% / 94.71% / 100% | 9.441 | 0 | 93.90% / 6.10% |

In Test B, Gazebo listed `dtu_partial_road_box`, the LaserScan measured a
nearest finite return of 11.754 m, the reference remained exactly 79.998 m and
167 poses, and the actual plan moved around the box before returning to the
reference. Test B completed with status `SUCCEEDED`; its final rover position
was map `(517.401, 359.244)`, 0.442 m from the requested goal. Test A also
completed with status `SUCCEEDED` and ended 0.482 m from the goal. Test C
completed with status `SUCCEEDED`; its final rover position was map
`(506.951, 352.949)`, 0.459 m from the exact goal under the existing 0.5 m
tolerance. All three completed without a recovery.

The graph contains 435 nodes, 469 nonzero unique edges, and one connected
component within the operational boundary. The source lines were already
topologically connected there: no crossing-only intersection splits and no
endpoint snap connectors were required. No connectivity defect was found.
