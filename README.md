WORKING FR FR:


(lidar frame p working h)

-
(rosbag pehle ofc :))

-

 ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node   --ros-args   -r cloud_in:=/synced_pointcloud   -r scan:=/scan   -p target_frame:=lidar -p transform_tolerance:=0.5 -p angle_min:=-3.14159   -p angle_max:=3.14159   -p angle_increment:=0.00872665   -p scan_time:=0.8 -p use_inf:=true   -p inf_epsilon:=1.0   -p queue_size:=50   -p use_sim_time:=true
 
-

ros2 topic echo /scan 
!!!!!!!!!!!!!!!!!!

-
ros2 run odom_fixer odom_fixer --ros-args -p use_sim_time:=true 

(pointcloud processor h name vese)

-
ros2 launch genz_icp odometry.launch.py   topic:=/synced_pointcloud   config_file:=/home/reet/q_ws/src/genz-icp/ros/config/outdoor.yaml   base_frame:=lidar odom_frame:=odom  use_sim_time:=true

(outdoor.yaml m publish tf on krna h)

-

ros2 run slam_toolbox async_slam_toolbox_node   --ros-args   --params-file ~/q_ws/src/pot/config/mapper_params_online_async.yaml   -p use_sim_time:=true   -p scan_topic:=/scan  -p odom_topic:=/genz/odometry

(comment out publishtf_map from params, base frame lidar krna h uss file m)

-
rviz2 pe /map kjwdbvcuigew




(base link frame p working h)


-

(rosbag chalana h)

-

ros2 run tf2_ros static_transform_publisher   --x 0 --y 0 --z 0.25   --yaw 0 --pitch 0 --roll 0   --frame-id base_link   --child-frame-id lidar --ros-args -p use_sim_time:=true

-

ros2 run odom_fixer odom_fixer --ros-args -p use_sim_time:=true 

(pointcloud processor h name vese)

-

ros2 launch genz_icp odometry.launch.py   topic:=/synced_pointcloud   config_file:=/home/reet/q_ws/src/genz-icp/ros/config/outdoor.yaml   base_frame:=lidar odom_frame:=odom  use_sim_time:=true

(outdoor.yaml m publish tf off krna h)

-

ros2 launch robot_localization ekf.launch.py

(comment out everything other than /genz/odometry usko odom0 krna h in ekf.yaml, for odom to base link ok h)

-

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node   --ros-args   -r cloud_in:=/synced_pointcloud   -r scan:=/scan   -p target_frame:=lidar -p transform_tolerance:=0.5 -p angle_min:=-3.14159   -p angle_max:=3.14159   -p angle_increment:=0.00872665   -p scan_time:=0.8 -p use_inf:=true   -p inf_epsilon:=1.0   -p queue_size:=50   -p use_sim_time:=true

-

ros2 topic echo /scan 
!!!!!!!!!!!!!!!!!!

-

ros2 run slam_toolbox async_slam_toolbox_node   --ros-args   --params-file ~/q_ws/src/pot/config/mapper_params_online_async.yaml   -p use_sim_time:=true   -p scan_topic:=/scan  -p odom_topic:=/genz/odometry
(comment out publishtf_map from params ,base frame is base_link)

- 

rviz2



