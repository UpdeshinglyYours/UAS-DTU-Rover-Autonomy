# UAS DTU Gazebo Harmonic simulation

The DTU prior-map/Nav2 integration is launched by
`bcr_bot_dtu_sim.launch.py`. Its verified interfaces, TF ownership, exact GPS
conversion, staged road-preference test, goal coordinates and validation
commands are documented in [BCR_DTU_NAV2.md](BCR_DTU_NAV2.md).

The centreline preference mask, nominal road-graph route, launch precedence,
physical obstacle test, and current path metrics are documented in
[CENTERLINE_NAVIGATION.md](CENTERLINE_NAVIGATION.md).

The selectable four-obstacle physical course is documented in
[BARRICADE_COURSE.md](BARRICADE_COURSE.md).

This package supplies a reproducible obstacle variant of the BCR Bot small
warehouse for LiDAR, mapping, Nav2, and autonomous-exploration testing. The
warehouse world was copied from the working `bcr_bot` package and retains its
original models, lighting, physics, sensors, and plugins. Ten coloured static
SDF boxes are added in open floor regions.

The main autonomy repository does not contain a BCR Bot description package.
This package therefore has an explicit runtime dependency on the working
`bcr_bot` package in `~/q_ws`; it does not duplicate or shadow the `bcr_bot`
package name. The AWS RoboMaker warehouse asset directories referenced by the
world are included here so world resources resolve from this package rather
than from a build, install, or log tree.

## Build and run

Source the BCR Bot workspace before the autonomy workspace so the latter is the
top overlay while `bcr_bot` remains discoverable:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/q_ws/install/setup.bash"
cd "$HOME/UAS-DTU-Rover-Autonomy"
colcon build --packages-select uas_dtu_gz_sim --symlink-install
source install/setup.bash
ros2 launch uas_dtu_gz_sim warehouse_obstacles.launch.py
```

The launch file also accepts the working BCR Bot spawn arguments, for example:

```bash
ros2 launch uas_dtu_gz_sim warehouse_obstacles.launch.py \
  position_x:=0.0 position_y:=0.0 orientation_yaw:=0.0 \
  camera_enabled:=true stereo_camera_enabled:=false \
  two_d_lidar_enabled:=true odometry_source:=world
```

The default spawn is `(x, y, z) = (0, 0, 0.28)`. The closest obstacle boundary
is more than 2 m away. The red and yellow passage pillars leave a 5.25 m opening
between their inner faces, exceeding the 3.5 m minimum. Their outside gaps to
the existing shelf rows are under 0.34 m and are intentionally non-traversable,
so they cannot be mistaken for narrower alternate passages.

## Included obstacle layout

| Model | Position x, y, z (m) | Size x, y, z (m) |
| --- | --- | --- |
| `lidar_box_west_orange` | `-3.2, 0.3, 0.45` | `0.9, 0.9, 0.9` |
| `lidar_box_northeast_blue` | `1.9, 2.1, 0.6` | `0.8, 1.0, 1.2` |
| `wide_passage_west_red` | `-4.0, -3.2, 0.75` | `0.8, 1.0, 1.5` |
| `wide_passage_east_yellow` | `2.05, -3.2, 0.65` | `0.8, 1.0, 1.3` |
| `offset_barrier_southwest_green` | `-3.0, -5.4, 0.35` | `1.4, 0.6, 0.7` |
| `offset_barrier_southcentral_purple` | `0.2, -6.2, 0.55` | `0.6, 1.5, 1.1` |
| `lidar_cube_southeast_cyan` | `1.8, -8.2, 0.4` | `0.8, 0.8, 0.8` |
| `offset_barrier_upper_magenta` | `0.7, 4.7, 0.5` | `1.5, 0.6, 1.0` |
| `lidar_cube_upper_west_lime` | `-3.4, 6.8, 0.6` | `0.8, 0.8, 1.2` |
| `lidar_barrier_upper_center_teal` | `1.0, 6.8, 0.45` | `0.6, 1.4, 0.9` |

## Source and licence

The base world and warehouse assets were copied from the local working checkout
of [Black Coffee Robotics' BCR Bot](https://github.com/blackcoffeerobotics/bcr_bot),
commit `a443038`, plus the user's current Harmonic-compatible working changes.
That project declares the Apache License 2.0; its complete `LICENSE` file is
included in this package. No Fuel models, generated files, or build artifacts
are included.
