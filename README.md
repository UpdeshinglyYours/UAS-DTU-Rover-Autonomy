# UAS-DTU Rover Navigation Stack

This document describes the complete startup sequence for the autonomous rover navigation stack using:

* ArduRover (Cube Orange)
* MAVROS
* Blickfeld LiDAR
* Genz-ICP
* Lirovo
* Nav2

---

# Prerequisites

* Ubuntu 22.04
* ROS2 Humble
* ArduPilot Rover firmware
* MAVROS installed
* Blickfeld LiDAR (192.168.26.26) connected (keep your ip 192.168.26.42)
* Pixhawk connected through USB

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
```

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

# 3. Set Rover Mode MANUAL or GUIDED
You either use new_omega_fixed5.py (works on manual) or you can use adaptive_velocity_controller.py

# 4. Set GPS Origin (if you wanna use GUIDED)

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

# 7. Start Adaptive Velocity Controller or NEW_OMEGA_FIXED5.py

in /lirovo/lirovo
```bash
python3 adaptive_velocity_controller_both2.py
```
or
```bash
python3 new_omega_fixed5.py
```

Ensure the node is forwarding velocity commands correctly.

---

# 8. Start Casualty Dispatcher

in /lirovo/lirovo
```bash
python3 casualty_dispatcher.py


# Recommended Startup Order

1. Launch MAVROS
2. Launch Blickfeld Driver
3. Set GUIDED mode or MANUAL mode
4. Set GPS/EKF Origin (for guided)
5. Launch Genz-ICP
6. Launch Lirovo (open nav2 rviz separately)
7. Start New Omega Controller or Adaptive Velocity Controller
8. Start Casualty Dispatcher

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
```

