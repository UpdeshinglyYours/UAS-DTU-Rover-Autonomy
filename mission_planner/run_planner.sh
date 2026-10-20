#!/usr/bin/env bash
# Open mission_planner_live on the NX desktop (NoMachine): click waypoints on the satellite map,
# press 'g' to send them to Nav2 (NavigateThroughPoses).
#
#   ~/mission_planner/run_planner.sh [planner args]    e.g. --map-lat 28.74 --map-lon 77.11
#   or double-click "Mission Planner" on the desktop (~/Desktop/mission_planner.desktop)
#
# The planner runs inside rover_humble (the host has no pyqtgraph / nav2_msgs). Before opening it
# this checks that the container and the autonomy stack are running, and warns when the stack uses
# GenZ odometry: the planner's goals are in MAVROS's GPS frame (x = east, y = north), so outdoors
# the stack must run with ODOM_SOURCE=mavros. Env: MAP=<png in ~/mission_planner> (default
# map_satellite_1km.png, 1 km around 28.7484727, 77.1171744).
set -u

DIR=/home/rover1/mission_planner
CONTAINER=rover_humble
SESSION=rover_stack2
S=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo
DDS=file://$S/stack2/cyclonedds_wifi.xml
STARTUP_LOG=/home/rover1/rover_ws/hardware_stack2_startup.log
MAP="${MAP:-map_satellite_1km.png}"

# Keep the terminal open at the end, so messages can be read when started from the desktop icon.
finish() { echo; read -r -p "Press Enter to close this window." _; }
trap finish EXIT

echo "== Mission Planner Live =="

if ! sudo podman ps --format '{{.Names}}' | grep -x "$CONTAINER" >/dev/null; then
    echo "Container $CONTAINER is not running. Start it:  cd ~/podman_rover && ./rover.sh up rover"
    exit 1
fi

if [ ! -f "$DIR/$MAP" ]; then
    echo "Map image $DIR/$MAP not found."
    exit 1
fi

# pyqtgraph and Pillow were installed by hand in the running container; a recreated container
# does not have them until the image includes them.
if ! sudo podman exec "$CONTAINER" python3 -c "import pyqtgraph, PIL" 2>/dev/null; then
    echo "pyqtgraph / Pillow missing in $CONTAINER. Install them (keeps numpy 1.21, which ROS needs):"
    echo "  sudo podman exec $CONTAINER bash -c 'apt-get update && apt-get install -y python3-pil && pip3 install --no-deps pyqtgraph==0.13.3'"
    exit 1
fi

if ! sudo podman exec "$CONTAINER" tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "WARNING: the autonomy stack is not running, so nothing will receive the goals. Start it:"
    echo "  cd ~/podman_rover && ODOM_SOURCE=mavros ./start_autonomy.sh --detach"
else
    ODOM=$(grep "Orchestration starting" "$STARTUP_LOG" 2>/dev/null | tail -1 | grep -o "ODOM_SOURCE=[a-z]*" | cut -d= -f2)
    if [ "${ODOM:-}" != mavros ]; then
        echo "WARNING: the stack runs with ODOM_SOURCE=${ODOM:-unknown}. The planner's goals are in MAVROS's"
        echo "  GPS frame (x = east, y = north); with GenZ they land in the wrong place. Outdoors restart with:"
        echo "  cd ~/podman_rover && ODOM_SOURCE=mavros ./start_autonomy.sh --detach"
    fi
    STATE=$(sudo podman exec -e CYCLONEDDS_URI="$DDS" "$CONTAINER" bash -c \
        "source /opt/ros/humble/setup.bash; timeout 8 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null | grep -o \"label='[a-z]*'\"")
    echo "Nav2 bt_navigator: ${STATE:-not answering (Nav2 not up yet?)}"
fi

ARGS=$(printf ' %q' "$@")
echo
echo "Opening the planner: wait until the origin shows LOCKED (needs a GPS fix), click waypoints,"
echo "press 'g' to send them, 'c' to clear. Close the planner window to quit."
echo
sudo podman exec -it \
    -e CYCLONEDDS_URI="$DDS" \
    -e DISPLAY="${DISPLAY:-:1001}" \
    -e XAUTHORITY=/home/rover1/.Xauthority \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    "$CONTAINER" bash -c "source /workspace/rover_ws/install/setup.bash; cd $DIR; exec python3 mission_planner_live.py --map $MAP$ARGS"
