# 🛠️ Setup Instructions

### Create a workspace and add an src folder to it

``` mkdir nav2_ws/src && cd src/```

### Clone the repo

```git clone https://github.com/AkshatKaushal25/UAS-DTU-navigation-simulation.git . ```

### head back to root directory

```cd ..```

### Install Dependencies 

```rosdep install -y --from-paths ./src --ignore-src```

### Build the directory 

```colcon build --symlink-install```


# 🛠️ Lidar Setup Instructions

### Installing Dependencies

1. **Blickfeld scanner library**  
   ```sudo apt update
      sudo apt update
      bash sudo apt install -y wget libprotobuf-dev libprotobuf23 
      wget https://github.com/Blickfeld/blickfeld-scanner-lib/releases/latest/download/blickfeld-scanner-lib-dev-testing-Linux.deb  
      sudo dpkg -i blickfeld-scanner-lib-dev-testing-Linux.deb``` 

2. **Diagnostic updater**  
    ```sudo apt install ros-humble-diagnostic-updater 
       sudo apt install ros-humble-diagnostic-updater 
       sudo apt install ros-humble-diagnostic-msgs```

### Build and Source the package
### Run The Node

```RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=192.168.26.26 --remap __node:=bf_lidar -p publish_imu:=true -p publish_imu_static_tf_at_start:=true```

# 🛠️ Lirovo Setup Instructions

### main launch command

``` ros2 launch lirovo lirovo.launch.py```

### Initialiase LiDAR

```RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run blickfeld_driver blickfeld_driver_node --ros-args -p host:=192.168.26.26 --remap __node:=bf_lidar -p publish_imu:=true -p publish_imu_static_tf_at_start:=true```

### Launch SITL

```sim_vehicle.py -v Rover --console --map```

### Launch mavros

```ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14550@ -p baud_rate:=57600```

    
    
       


