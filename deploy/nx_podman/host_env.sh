# Source in host shells that run ROS natively (host Jazzy: HR_lidar_RGB,
# frame_extraction ...) so they join the podman containers' DDS:
#   source ~/podman_rover/host_env.sh
# Needs ros-jazzy-rmw-cyclonedds-cpp on the host (debs/ in this folder).
# ROS_NET=net before sourcing = autodetected network discovery, matching
# containers started with ROS_NET=net.
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
if [[ "${ROS_NET:-local}" != net ]]; then
    export CYCLONEDDS_URI=file://$HOME/podman_rover/cyclonedds_local.xml
else
    unset CYCLONEDDS_URI
fi
