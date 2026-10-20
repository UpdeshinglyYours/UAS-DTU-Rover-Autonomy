# lirovo/stack2: the laptop's `start_hardware_stack2.sh` + `lirovo3` stack in the container

These are new files only; nothing else in the repo is changed.

| File | Laptop source | Notes |
|---|---|---|
| `start_hardware_stack2_docker.sh` | `~/start_hardware_stack2.sh` | Run inside the container. Same 6-pane tmux layout. No lidar gate: lirovo3 starts 5 s after the drivers. |
| `lirovo3.launch.py` | `~/vortex_ws/lirovo/launch/lirovo3.launch.py` | Same nodes and TFs. Reads the params and BT XML from this folder. |
| `nav2_params3.yaml` | `~/vortex_ws/lirovo/config/nav2_params3.yaml` | Byte-identical copy. The BT XML path is rewritten at launch time. |
| `vortex_custom_polygon_tree.xml` | `~/vortex_ws/nav2_custom_bt_nodes/config/` | The laptop version: 8 approach retries, `SkipBlockedApproach`. |
| `mavros_apm_pluginlists.yaml` | (new) | An allowlist that includes `imu`, `rc_io` and `global_position`. The repo's list says `imu_pub`, which is not a MAVROS 2 plugin. |
| `new_omega_fixed5.py` | `~/new_omega_fixed5.py` | Stops itself when the mode leaves MANUAL. `lirovo/lirovo/new_omega_fixed5.py` lacks this. |
| `casualty_bt_nodes_clockwise.patch` | `~/vortex_ws/nav2_custom_bt_nodes` | Makes the polygon orbit run clockwise, as on the laptop. It is not applied, because it edits an existing file. |

`vortex_interfaces/` and `vortex_mission/` (at the repo root) are needed for `rover_console`, which provides the keyboard E-STOP in pane 5.

## Install (inside the container)
```bash
cd /workspace/rover_ws
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1 --packages-select vortex_interfaces vortex_mission
# optional, clockwise polygon orbit like the laptop:
#   cd src/UAS-DTU-navigation-simulation && git apply lirovo/stack2/casualty_bt_nodes_clockwise.patch
#   cd /workspace/rover_ws && colcon build --packages-select nav2_custom_bt_nodes
```

## Run
```bash
docker exec -it uas_rover_container bash \
  /workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2/start_hardware_stack2_docker.sh
```
Environment variables:
- `FCU_URL`: default `/dev/serial/by-id/usb-CubePilot_*-if00`;
- `SF45B_PORT`: default `/dev/ttyUSB0`;
- `ROS_DOMAIN_ID`: default 2;
- `RVIZ=1`: run RViz on the Orin;
- `ODOM_SOURCE`: `mavros` (default) or `genz`, see below;
- `GCS_URL`: forward MAVLink to a ground station through MAVROS, e.g. `udp://@192.168.0.41:14555`.

## Odometry source (`ODOM_SOURCE`, default `mavros`)
The switch selects where `odom_to_tf` takes the pose for `map->base_link`. Everything that uses TF (Nav2, the costmaps) follows it. `new_omega`'s velocity feedback always comes from `/mavros/local_position/odom`.

| `ODOM_SOURCE` | Topic | `map->base_link` (`odom_to_tf`) |
|---|---|---|
| `mavros` (default) | `/mavros/local_position/odom` | Cube EKF; needs a GPS fix |
| `genz` | `/genz/odometry` | GenZ-ICP; the pipeline is started by `lirovo3.launch.py` |

```bash
ODOM_SOURCE=genz .../lirovo/stack2/start_hardware_stack2_docker.sh   # whole stack
ros2 launch .../lirovo/stack2/lirovo3.launch.py odom_source:=genz     # lirovo3 only
```
- With `genz`, GenZ starts 3 s after the static transforms. When it started before the `base_link->lidar` transform existed, its attitude diverged.
- GenZ needs `/mavros/imu/data`, so keep the `imu` plugin.
