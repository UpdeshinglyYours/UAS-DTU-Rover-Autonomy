# UAS-DTU Rover Autonomy Stack (ROS2 Humble)

## Overview

This repository contains the simulation and autonomy stack used by the **UAS-DTU Rover Team**.

The system is built on top of the **BCR Bot** framework but has been extensively modified to match the hardware configuration and software architecture of the UAS-DTU rover.

### Key Differences from BCR Bot

| BCR Bot | UAS-DTU Rover |
|----------|-------------|
| 2D LiDAR | 3D LiDAR |
| Generic simulation robot | Hardware-matched rover model |
| Default sensor frequencies | Sensor frequencies tuned to match real hardware |
| Generic Nav2 configuration | Hardware-tested Nav2 parameters |
| Basic simulation | Full SLAM + Navigation pipeline |

### Customizations

- ROS2 Humble based stack
- 3D LiDAR integration
- PointCloud → LaserScan conversion pipeline
- Hardware-matched Nav2 parameters
- Hardware-matched sensor update rates
- SLAM Toolbox integration
- Nav2 autonomous navigation
- Gazebo simulation environment for testing and validation

> Sensor update rates and Nav2 parameters have been intentionally configured to resemble the real rover hardware. As a result, motion in simulation may appear slightly jittery compared to idealized simulations. This behavior is expected and helps achieve more realistic testing conditions.

---

# System Architecture

```text
3D LiDAR
    ↓
PointCloud2
    ↓
pointcloud_to_laserscan
    ↓
LaserScan
    ↓
SLAM Toolbox
    ↓
Map Generation
    ↓
Nav2
    ↓
Autonomous Navigation
```

---

# Launching the Simulation

Start the Gazebo simulation:

```bash
ros2 launch bcr_bot gazebo.launch.py
```

---

# Mapping (SLAM)

Launch SLAM Toolbox:

```bash
ros2 launch bcr_bot mapping.launch.py
```

---

# Navigation (Nav2)

Launch Nav2:

```bash
ros2 launch bcr_bot nav2.launch.py
```

---

# PointCloud to LaserScan Conversion

The rover uses a 3D LiDAR. Since SLAM Toolbox and Nav2 primarily operate on LaserScan data, PointCloud2 data is converted into a virtual 2D scan using the following node:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args \
-p target_frame:=base_footprint \
-p transform_tolerance:=0.05 \
-p min_height:=0.4 \
-p max_height:=2.0 \
-p angle_min:=-3.14159 \
-p angle_max:=3.14159 \
-p angle_increment:=0.00872665 \
-p scan_time:=0.3 \
-p range_min:=0.1 \
-p range_max:=200.0 \
-p use_inf:=true \
-p inf_epsilon:=1.0 \
-p queue_size:=50 \
-p use_sim_time:=true \
-p qos_overrides./bcr_bot/scan.publisher.reliability:=best_effort \
-r cloud_in:=/points
```

---

# Complete Startup Sequence

## Terminal 1 – Gazebo

```bash
ros2 launch bcr_bot gazebo.launch.py
```

## Terminal 2 – PointCloud Conversion

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
--ros-args \
-p target_frame:=base_footprint \
-p transform_tolerance:=0.05 \
-p min_height:=0.4 \
-p max_height:=2.0 \
-p angle_min:=-3.14159 \
-p angle_max:=3.14159 \
-p angle_increment:=0.00872665 \
-p scan_time:=0.3 \
-p range_min:=0.1 \
-p range_max:=200.0 \
-p use_inf:=true \
-p inf_epsilon:=1.0 \
-p queue_size:=50 \
-p use_sim_time:=true \
-p qos_overrides./bcr_bot/scan.publisher.reliability:=best_effort \
-r cloud_in:=/points
```

## Terminal 3 – Mapping

```bash
ros2 launch bcr_bot mapping.launch.py
```

## Terminal 4 – Navigation

```bash
ros2 launch bcr_bot nav2.launch.py
```

---

# Notes

- This simulation is intended to closely resemble the real UAS-DTU rover hardware.
- Navigation parameters are identical to those deployed on the physical rover.
- Sensor update frequencies are reduced to realistic hardware rates rather than ideal simulation rates.
- Slightly non-smooth or jittery motion in simulation is expected and reflects real-world operating conditions.
- The stack is designed for testing SLAM, localization, path planning, and autonomous navigation before deployment on the physical rover.

---

# Technology Stack

- ROS2 Humble
- Gazebo Classic
- Nav2
- SLAM Toolbox
- Robot Localization
- PointCloud to LaserScan
- RViz2
- 3D LiDAR-based Perception

This version clearly explains why this fork exists, what has been changed from the original BCR Bot project, and how the simulation environment closely matches the actual UAS-DTU rover hardware.
