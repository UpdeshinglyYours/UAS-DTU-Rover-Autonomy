# UAS-DTU Rover Navigation Stack

This document describes the complete startup sequence for the autonomous rover navigation stack using:

* ArduRover (Cube Orange)
* MAVROS
* Blickfeld LiDAR
* Genz-ICP
* Lirovo
* Nav2
* Target Explorer

---

# Prerequisites

* Ubuntu 22.04
* ROS2 Humble
* ArduPilot Rover firmware
* MAVROS installed
* Blickfeld LiDAR connected
* Pixhawk connected through USB
* Workspace built successfully

```bash
source /opt/ros/humble/setup.bash
source ~/DARPA_rover_ws/install/setup.bash
```

---

# Hardware Connections

Verify the following before starting:

* Pixhawk connected 
* Blickfeld LiDAR reachable at

```
192.168.26.26
```

* IMU publishing correctly
* Battery connected
* RC transmitter connected (recommended)
---
# 1. Start Blickfeld Driver

```bash
ros2 run blickfeld_driver blickfeld_driver_node \
  --ros-args \
  -p host:=192.168.26.26 \
  --remap __node:=bf_lidar \
  -p publish_imu:=true \
  -p publish_imu_static_tf_at_start:=true \
  -p use_lidar_timestamp:=false \
  -p publish_intensities:=true \
  -p publish_point_time_offset:=true \
  -p imu_acceleration_unit:=meters_per_second_squared
```

Wait until:

* Point cloud is publishing
* IMU data is publishing


# 2. Start MAVROS

```bash
ros2 launch mavros apm.launch \
    fcu_url:=/dev/ttyACM0:115200 \
    gcs_url:=udp://@127.0.0.1:14550
```

Verify:

```bash
ros2 topic echo /mavros/state
```

Check that:

* Connected = true
* Mode is updating correctly

---

# 3. Set Rover Mode to GUIDED

Using MAVProxy:

```bash
mode GUIDED
```


# 4. Set GPS Origin

If operating without a GPS fix or when required by your localization pipeline, set the EKF origin.

Example using MAVProxy:
module load message 
```
message COMMAND_INT 0 0 0 179 0 0 0 0 0 0 -353630000 1491650000 575
message COMMAND_LONG 0 0 179 0 0 0 0 0 -35.363 149.165 575
message SET_GPS_GLOBAL_ORIGIN 0 -353621474 1491651746 600000 0
```

Example:

```text
```

Alternatively, use Mission Planner's **Set EKF Origin** functionality if supported.

Confirm that the local position is being published:

```bash
ros2 topic echo /mavros/local_position/odom
```

---

# 5. Start Genz-ICP

```bash
ros2 launch genz_icp bf_lidar_genz_pipeline.launch.py use_sim_time:=false
```



# 6. Start Lirovo

```bash
ros2 launch lirovo lirovo.launch.py
```


---

# 7. Start Adaptive Velocity Controller

in /src/lirovo/lirovo
```bash
python3 adaptive_velocity_controller_both2.py
```

Ensure the node is forwarding velocity commands correctly.

---

# 8. Start Target Explorer

```bash
ros2 run target_explorer target_explorer_node \
  --ros-args \
  --params-file \
  /home/aryaman/UAS-DTU-Rover-Autonomy/src/target_explorer/config/target_explorer_params.yaml \
  -p robot_base_frame:=base_link \
  -p target_x:=4.0 \
  -p target_y:=0.0
```

This sends the rover to the specified target.

---

# Recommended Startup Order

1. Power Rover
2. Connect Pixhawk
3. Connect LiDAR
4. Launch Blickfeld Driver
5. Launch MAVROS
6. Verify MAVROS connection
7. Set GUIDED mode
8. Set GPS/EKF Origin
9. Launch Genz-ICP
10. Launch Lirovo
11. Start Adaptive Velocity Controller
12. Start Target Explorer

---

# Stack Overview

```
Blickfeld LiDAR
        │
        ▼
Blickfeld Driver
        │
        ▼
     Genz-ICP
        │
        ▼
      Lirovo
        │
        ▼
 Localization / TF
        │
        ▼
Adaptive Velocity Controller
        │
        ▼
 MAVROS → ArduRover
        ▲
        │
 Target Explorer
```

