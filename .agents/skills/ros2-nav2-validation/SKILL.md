---
name: ros2-nav2-validation
description: Validate DTU rover ROS 2 Humble Nav2 changes in simulation or on the real rover, including focused builds, lifecycle state, TF ownership, maps, paths, obstacle response, and shutdown. Use after changes to dtu_prior_map, uas_dtu_gz_sim, or Nav2 configuration; physical actuation still requires explicit authorization.
---

# ROS 2 and Nav2 validation

Validate observable behavior without expanding the requested change.

## Establish scope and preserve state

1. Classify the task as `simulation`, `real-rover read-only`, or `real-rover actuating`. If physical motion, arming, RTCM injection, flight-controller changes, or navigation goals were not explicitly authorized, remain read-only.
2. From the repository root, inspect `git status --short` and the focused diff. Preserve unrelated and untracked work.
3. Check for existing ROS and Gazebo processes before launching. Do not kill unrelated processes automatically. Avoid duplicate Nav2, map-server, simulator, localization, and TF publishers.
4. Confirm every node in a run uses the same clock mode.

## Build the affected overlay

Source ROS first:

```bash
source /opt/ros/humble/setup.bash
```

For BCR simulation, confirm the external package exists and source it:

```bash
source "$HOME/q_ws/install/bcr_bot/share/bcr_bot/package.bash"
```

Build only packages affected by the focused diff. The usual DTU simulation build is:

```bash
colcon build \
  --packages-select dtu_prior_map uas_dtu_gz_sim \
  --symlink-install
source install/setup.bash
```

If `bcr_bot` itself changed, build it in `~/q_ws` first. Do not rebuild every workspace by default.

Run package tests after a successful build:

```bash
colcon test --packages-select dtu_prior_map uas_dtu_gz_sim
colcon test-result --verbose
```

## Simulation validation

Prefer headless mode for automated checks and use a normal RViz run for visual confirmation. Start with centreline guidance enabled:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  headless:=true use_rviz:=false \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0
```

Run the launch in a managed terminal or background process with a cleanup trap. Stop it with SIGINT and wait for child processes to exit. Never suspend it with `Ctrl+Z`.

Check lifecycle nodes through services, for example:

```bash
ros2 service call /dtu_static_map_server/get_state \
  lifecycle_msgs/srv/GetState "{}"
ros2 service call /planner_server/get_state \
  lifecycle_msgs/srv/GetState "{}"
ros2 service call /controller_server/get_state \
  lifecycle_msgs/srv/GetState "{}"
ros2 service call /bt_navigator/get_state \
  lifecycle_msgs/srv/GetState "{}"
```

Check required data and TF:

```bash
ros2 topic echo /dtu_static_map --once
ros2 topic echo /dtu_reference_route --once
ros2 topic hz /bcr_bot/odom
ros2 topic hz /bcr_bot/scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info -v /tf
ros2 topic info -v /tf_static
```

Use the current goals and metrics commands documented in `src/uas_dtu_gz_sim/CENTERLINE_NAVIGATION.md`; do not duplicate coordinates that may become stale. After planner, costmap, controller, map, or centreline changes, test:

- a clear goal on the road;
- the same route with the physical test obstacle enabled;
- a free off-road goal.

For each test, verify that the action result succeeds, `/plan` avoids lethal static cells, the obstacle test deviates from and returns toward `/dtu_reference_route`, and the off-road goal remains reachable. Record path/reference distance metrics when the metrics node is available.

## Real-rover validation

Do not use simulation launch commands or a fixed simulation `map -> odom` transform.

Before any authorized motion, verify read-only evidence:

- RTK/GPS topic, fix status, covariance, update rate, and timestamps;
- IMU and odometry topic frames, rates, and timestamps;
- a complete single-parent TF chain from `map` through `odom` to the robot base;
- one owner for `map -> odom` and one owner for `odom -> base_link` or `base_footprint`;
- active Nav2 lifecycle nodes, current costmaps, and command topic routing;
- no simulation nodes or stale bag-time processes.

If the requested physical test lacks a bounded area, emergency stop method, speed limit, or named operator, stop after read-only checks and ask for those details.

## Report

Return a compact evidence table with each build/test, command, result, and any relevant measurement. Separate:

- verified runtime facts;
- static inspection only;
- failures or tests not run;
- files changed by the task;
- unrelated pre-existing worktree state.

Do not call a system validated when only topics exist; require the relevant TF, lifecycle, planning, sensor, and action behavior for the task.
