# UAS-DTU Rover Autonomy Stack

ROS2 Humble autonomy stack for the UAS-DTU Rover.

---

# Build Workspace

```bash
cd ~/DARPA_rover_ws
colcon build --symlink-install
```

---

# Source Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/DARPA_rover_ws/install/setup.bash
```

---

# Rover Bringup

## Terminal 1 — APM / MAVROS

```bash
source ~/DARPA_rover_ws/install/setup.bash

ros2 launch mavros apm.launch
```

---

## Terminal 2 — Blickfeld Driver

```bash
source ~/DARPA_rover_ws/install/setup.bash

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node \
--ros-args \
-p host:=192.168.26.26 \
--remap __node:=bf_lidar \
-p publish_imu:=true \
-p publish_imu_static_tf_at_start:=true \
-p use_lidar_timestamp:=false
```

---

## Terminal 3 — LIROVO

```bash
source ~/DARPA_rover_ws/install/setup.bash

ros2 launch lirovo lirovo.launch.py
```

Launches:

- PointCloud → LaserScan
- GenZ ICP Odometry
- SLAM Toolbox
- TF publishers

---

## Terminal 4 — Navigation2

```bash
source ~/DARPA_rover_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py
