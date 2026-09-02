# UAS-DTU Rover Autonomy instructions

## Repository and environment

- This repository targets Ubuntu 22.04, ROS 2 Humble, Nav2, and Gazebo Sim Harmonic.
- The main checkout is `~/UAS-DTU-Rover-Autonomy`.
- The external BCR simulation package is built separately in `~/q_ws`.
- Before ROS commands, source `/opt/ros/humble/setup.bash`. For BCR simulation work, also source `~/q_ws/install/bcr_bot/share/bcr_bot/package.bash`, then source this repository's `install/setup.bash` after building.
- Prefer focused builds with `colcon build --packages-select <affected-packages> --symlink-install`.

## Preserve the worktree

- Treat all existing tracked modifications, deletions, untracked packages, maps, bags, and calibration data as user-owned.
- Inspect `git status --short` before editing. Do not discard, overwrite, stage, commit, clean, or reformat unrelated work.
- Never run `git reset --hard`, `git clean -fd`, or checkout/restore paths unless the user explicitly requests the exact destructive operation.
- Do not modify generated `build/`, `install/`, or `log/` files directly.
- Keep edits minimal and show a focused diff for every changed file.

## Simulation and real-rover separation

### DTU simulation

- Simulation integration belongs in `src/dtu_prior_map` and `src/uas_dtu_gz_sim` unless the user explicitly expands scope.
- Keep `use_sim_time: true` throughout a simulation run.
- In the validated BCR simulation, TF ownership is: the simulation placement publishes `map -> odom`; Gazebo publishes `odom -> base_footprint`; `robot_state_publisher` publishes the robot body and sensor frames.
- Do not start SLAM Toolbox, AMCL, GPS localization, `robot_localization`, or a second `map -> odom` publisher in this simulation.
- `/dtu_static_map` is the hard occupancy map. `/dtu_centerline_cost` or the road preference mask is a soft global planning preference. Live LiDAR obstacles must remain capable of overriding that preference.
- The standard centreline test launch is:

  ```bash
  ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
    enable_centerline_tether:=true \
    cost_travel_multiplier:=3.0
  ```

### Real rover

- Real-rover configuration belongs under `src/lirovo` and related hardware packages. Keep `use_sim_time: false`.
- Do not copy the simulation's fixed `map -> odom` placement into the real rover. The real transform must come from the GPS/global localization design.
- Do not launch Gazebo, spawn models, or alter simulation-only parameters during a real-rover task.
- Never arm the vehicle, change flight-controller parameters, inject RTCM, publish motion commands, or send a navigation goal to physical hardware unless the user explicitly authorizes that action and gives a stop condition.
- For physical tests, begin with read-only checks of topics, TF, localization covariance, Nav2 lifecycle state, and command outputs.

## Navigation invariants

- The static map remains authoritative for buildings and the operational boundary.
- The centreline is guidance, not a physical obstacle and not a command trajectory.
- The global planner creates the collision-aware `/plan`; the controller follows that path while the local costmap handles current sensor obstacles.
- Off-road goals must remain reachable when their endpoint is free in the static map.
- Exactly one publisher owns each TF edge. Never mask a missing localization component with an unexplained static transform.

## Validation expectations

- For changes to `dtu_prior_map` or `uas_dtu_gz_sim`, use the `ros2-nav2-validation` skill.
- Build only affected packages first. Run their package tests and a headless launch smoke test when the environment supports ROS/Gazebo.
- Verify lifecycle state through each node's `get_state` service; do not assume the optional `ros2 lifecycle` CLI extension is installed.
- Verify map topics, TF connectivity and ownership, odometry, scan data, `/dtu_reference_route`, `/plan`, and `/cmd_vel` as applicable.
- Test a clear road goal, an obstacle detour, and a reachable off-road goal after navigation behavior changes.
- End launched ROS processes with `Ctrl+C` or SIGINT and wait for shutdown. Do not use `Ctrl+Z`, and do not leave duplicate launch graphs running.

## Completion report

- State what changed, what was deliberately left unchanged, the exact commands run, and their results.
- Report unexecuted tests and blockers honestly. Do not claim runtime validation from static inspection alone.
- Include `git status --short` and focused diffs for the files touched; do not include unrelated worktree noise as if it were part of the task.
