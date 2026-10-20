#!/usr/bin/env bash
# Bring up the rover's sensors in the uas_rover container (ROS 2 Humble) and
# record them into one rosbag, in one tmux session:
#
#   window 0  mavros     MAVROS on the flight controller (waits for one if absent)
#   window 1  blickfeld  Blickfeld Cube 1 LiDAR -> /bf_lidar/point_cloud_out, imu_out
#   window 2  sf45b      LightWare SF45/B -> /lightwarelidar/pointcloud, scan
#   window 3  camera     fcb_driver: FCB RGB/IR camera, VISCA zoom, AVI+CSV (respawns)
#   window 4  control    fcb_keys: zoom and RGB/IR mode by service call
#   window 5  bag        ros2 bag record (MCAP) of the camera, LiDAR and vehicle topics
#
# Run inside the container (lirovo/lirovo/run_container.sh):
#
#   ./record_sensors.sh                 start everything, or re-attach if running
#   ./record_sensors.sh --no-bag        start without the rosbag window
#   ./record_sensors.sh --all-topics    bag every topic except raw image_raw
#                                       (default: the sensor + core telemetry list)
#   ./record_sensors.sh --no-mavros     no flight controller (fc_source none)
#   ./record_sensors.sh --no-sf45b      no LightWare SF45/B
#   ./record_sensors.sh --no-blickfeld  no Blickfeld LiDAR
#   ./record_sensors.sh --no-camera     no camera/control windows: the FCB driver runs
#                                       in the fcb_jazzy podman container (rover.sh);
#                                       its topics are still bagged
#   ./record_sensors.sh --fcu URL       pin MAVROS's fcu_url, e.g. /dev/ttyACM1:115200
#   ./record_sensors.sh --sysid N       the autopilot's SYSID_THISMAV (default: read it)
#   ./record_sensors.sh --sf45b-port P  SF45/B serial port (default /dev/ttyUSB0)
#   ./record_sensors.sh --bf-host IP    Blickfeld address (default: driver_config.yaml)
#   ./record_sensors.sh --bag           (re)start only the rosbag, in a new window
#   ./record_sensors.sh --status        what is running
#   ./record_sensors.sh --stop          stop everything (asks first, see below)
#
# Every sensor is stamped with this machine's clock: the camera with each
# frame's capture time, the LiDARs with ros now() on receipt (Blickfeld
# use_lidar_timestamp: false -- the scanner's own clock is not synced to
# anything). All nodes run here, in one container on the host network, so
# they share one clock and one DDS (CycloneDDS). Check a bag with
#   python3 check_bag_sync.py <bag dir>
#
# Detach with Ctrl-b then d; everything keeps running. Ctrl-C in the
# control or bag window ends only that window's program.
#
# DO NOT stop the camera driver between bags. On the Oppila board, closing
# the video stream and reopening it usually leaves the board sending
# nothing until its 12 V is switched off and on. Start a new bag with
# --bag and leave the driver running. Only one program may hold the camera:
# stop the host's Jazzy stack (~/fcb_orin_ros/fcb_ros.sh --stop) first.
set -u

SESSION=rover_rec
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
BAG_DIR="${ROVER_BAG_DIR:-$WS/bags}"
RECORD_DIR="${FCB_RECORD_DIR:-$WS/flight_recordings}"   # the driver's AVI + CSV
FCU_URL="${FCU_URL:-}"
FCU_SYSID="${FCU_SYSID:-}"           # "" = read it from the autopilot's heartbeat
SF45B_PORT="${SF45B_PORT:-/dev/ttyUSB0}"
BF_CONFIG="$REPO_DIR/Blickfeld/config/driver_config.yaml"
BF_HOST="${BF_HOST:-}"               # "" = host: in driver_config.yaml
# No Jetson GStreamer encoder (nvv4l2h264enc) in this Ubuntu 22.04 image,
# so the driver's own AVI is encoded on the CPU. The bag carries the JPEG
# stream either way.
FCB_ENCODER="${FCB_ENCODER:-cpu}"
WITH_BAG=1
BAG_ALL=0
WITH_MAVROS=1
WITH_SF45B=1
WITH_BLICKFELD=1
WITH_CAMERA=1
ACTION=start

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-bag)       WITH_BAG=0; shift ;;
        --all-topics)   BAG_ALL=1; shift ;;
        --no-mavros)    WITH_MAVROS=0; shift ;;
        --no-sf45b)     WITH_SF45B=0; shift ;;
        --no-blickfeld) WITH_BLICKFELD=0; shift ;;
        --no-camera)    WITH_CAMERA=0; shift ;;
        --fcu)          FCU_URL="$2"; shift 2 ;;
        --sysid)        FCU_SYSID="$2"; shift 2 ;;
        --sf45b-port)   SF45B_PORT="$2"; shift 2 ;;
        --bf-host)      BF_HOST="$2"; shift 2 ;;
        --bag)          ACTION=bag; shift ;;
        --status)       ACTION=status; shift ;;
        --stop)         ACTION=stop; shift ;;
        -h|--help)      sed -n '2,49p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *)              echo "unknown option: $1 (see --help)" >&2; exit 1 ;;
    esac
done

[[ -f "$ROS_SETUP" ]] || { echo "error: $ROS_SETUP not found -- run this inside the uas_rover container" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || {
    echo "error: $WS is not built -- run: cd $WS && colcon build --symlink-install" >&2
    exit 1; }
command -v tmux >/dev/null || { echo "error: tmux is not installed (rebuild the image, or apt install tmux)" >&2; exit 1; }

# Same middleware for every process here, whatever the calling shell had.
ENV="source $ROS_SETUP && source $WS/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

running() { tmux has-session -t "$SESSION" 2>/dev/null; }

# Single-quote a command for bash -c '...'.
q() { printf '%s' "$1" | sed "s/'/'\\\\''/g"; }

# The flight controller's serial port, by USB identity rather than ttyACM
# number (those shuffle on every replug). The camera board's own ports
# (Cypress FX3 / NeoHD) and the SF45/B's USB-serial adapter are never it.
NOT_FCU='*Cypress*|*FX3*|*NeoHD*|*04b4*|*FTDI*|*CP210*|*Silicon_Labs*|*LightWare*'
find_fcu() {
    local link
    for link in /dev/serial/by-id/*; do
        [[ -e "$link" ]] || continue
        case "$link" in
            *Cypress*|*FX3*|*NeoHD*|*04b4*|*FTDI*|*CP210*|*Silicon_Labs*|*LightWare*) continue ;;
            *ArduPilot*|*CubePilot*|*Holybro*|*Pixhawk*|*PX4*|*mRo*|*Hex*|*ProfiCNC*|*Matek*|*SpeedyBee*)
                echo "$(readlink -f "$link"):115200"; return 0 ;;  # ttyACMx: MAVROS rejects the by-id path (CubeOrange+)
        esac
    done
    return 1
}

BAG_TOPICS=(
    # RGB / IR camera
    /fcb_driver/image_raw/compressed
    /fcb_driver/snapshot/compressed
    /fcb_driver/status
    /fcb_driver/status_text
    # Blickfeld LiDAR
    /bf_lidar/point_cloud_out
    /bf_lidar/imu_out
    # LightWare SF45/B
    /lightwarelidar/pointcloud
    /lightwarelidar/scan
    # frames
    /tf
    /tf_static
    # vehicle
    /mavros/state
    /mavros/rc/in
    /mavros/imu/data
    /mavros/global_position/global
    /mavros/global_position/rel_alt
    /mavros/global_position/local
    /mavros/local_position/pose
    /mavros/local_position/velocity_local
    /mavros/vfr_hud
    /mavros/battery
    /rosout
)

#: Never bagged: raw 1080p60 is ~370 MB/s, and camera_info rides with it
#: (subscribing to it makes the driver copy every raw frame). The video is
#: in the bag as the 960 px JPEG stream, and at full quality in the AVI.
BAG_EXCLUDE='^/fcb_driver/(image_raw|camera_info)$'

bag_command() {
    local select
    if (( BAG_ALL )); then
        select="-a -x '$BAG_EXCLUDE'"
    else
        select="${BAG_TOPICS[*]}"
    fi
    # Humble's ros2 bag takes the topics as positional arguments (no
    # --topics), and -x for the exclude regex. The cache is raised from
    # 100 MB so a burst of point clouds never waits on the disk.
    cat <<EOF
$ENV && mkdir -p "$BAG_DIR" && echo "rosbag -> $BAG_DIR (Ctrl-C stops the bag only)" && \
B="$BAG_DIR/rover-\$(date +%Y%m%d-%H%M%S)" && \
ros2 bag record -s mcap --max-cache-size 268435456 -o "\$B" $select; \
echo; echo "bag stopped: \$B"; echo "check it:  python3 $SCRIPT_DIR/check_bag_sync.py \$B"; \
echo "./record_sensors.sh --bag starts a new one."; exec bash
EOF
}

case "$ACTION" in
status)
    if running; then
        tmux list-windows -t "$SESSION" -F '  #{window_index} #{window_name}: #{pane_current_command}'
        bash -c "$ENV && timeout 5 ros2 topic echo --once /fcb_driver/status_text 2>/dev/null" \
            | sed -n 's/^data: /  camera: /p'
    else
        echo "session '$SESSION' is not running"
    fi
    exit 0 ;;
stop)
    running || { echo "session '$SESSION' is not running"; exit 0; }
    echo "Stopping the camera driver closes the camera stream. On the Oppila board"
    echo "the video usually does not come back until its 12 V is power-cycled."
    read -r -p "Stop everything anyway? [y/N] " answer
    [[ "$answer" == [yY]* ]] || { echo "left running"; exit 0; }
    # Ctrl-C to each window, bag first, so the bag and the AVI/CSV are finalised.
    for w in bag control camera sf45b blickfeld mavros; do
        tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null
    done
    for _ in $(seq 1 30); do
        pgrep -f "fcb_orin_driver/fcb_driver|ros2 bag record|blickfeld_driver_node|lightwarelidar/sf45b|mavros_node" >/dev/null || break
        sleep 1
    done
    tmux kill-session -t "$SESSION"
    echo "stopped"
    exit 0 ;;
bag)
    running || { echo "session '$SESSION' is not running -- start it first"; exit 1; }
    tmux kill-window -t "$SESSION:bag" 2>/dev/null
    tmux new-window -d -t "$SESSION" -n bag "bash -c '$(q "$(bag_command)")'"
    echo "rosbag restarted in window 'bag'"
    exit 0 ;;
esac

if running; then
    echo "session '$SESSION' is already running -- attaching (Ctrl-b d detaches)"
    exec tmux attach -t "$SESSION"
fi

if (( WITH_CAMERA )); then
# Only one program may hold the camera. The host's Jazzy fcb_ros.sh runs in
# another PID namespace, so it is looked for on the ROS graph instead.
if bash -c "$ENV && timeout 10 ros2 node list --no-daemon --spin-time 3 2>/dev/null" | grep -qx "/fcb_driver"; then
    echo "error: an fcb_driver is already running (the host's ~/fcb_orin_ros/fcb_ros.sh?)." >&2
    echo "Only one program may hold the camera. Stop it first: ~/fcb_orin_ros/fcb_ros.sh --stop" >&2
    exit 1
fi
fi

# -- MAVROS -------------------------------------------------------------------
# MAVROS ignores an autopilot whose system ID is not tgt_system, so it is
# read off the heartbeat unless given.
if [[ -n "$FCU_SYSID" ]]; then
    SYSID_CMD="SYS=$FCU_SYSID"
else
    SYSID_CMD="SYS=\$(python3 $REPO_DIR/fcb_camera/fcu_sysid.py \$FCU)"
fi
if (( WITH_MAVROS )); then
    FC_SOURCE=mavros
    if [[ -z "$FCU_URL" ]]; then
        FCU_URL=$(find_fcu) || FCU_URL=""
    fi
    if [[ -n "$FCU_URL" ]]; then
        echo "flight controller: $FCU_URL"
        MAVROS_CMD="$ENV && FCU=$FCU_URL && $SYSID_CMD && echo \"autopilot system id \$SYS\" && ros2 launch mavros apm.launch fcu_url:=\$FCU tgt_system:=\$SYS"
    else
        echo "no flight controller found yet -- the mavros window waits for one"
        MAVROS_CMD="$ENV && echo 'waiting for a flight controller on /dev/serial/by-id ...' && \
while true; do \
  for l in /dev/serial/by-id/*; do [[ -e \$l ]] || continue; case \$l in $NOT_FCU) continue;; esac; FCU=\$(readlink -f \$l):115200; break 2; done; sleep 3; \
done; echo \"found \$FCU\"; sleep 2; $SYSID_CMD; echo \"autopilot system id \$SYS\"; ros2 launch mavros apm.launch fcu_url:=\$FCU tgt_system:=\$SYS"
    fi
else
    FC_SOURCE=none
    MAVROS_CMD="echo 'mavros not started (--no-mavros): no flight controller, start recordings with r in the control window'"
fi

# -- Blickfeld ----------------------------------------------------------------
# Run directly rather than through live_scanner_node.launch.py, whose
# on_exit=Shutdown leaves nothing behind when the scanner is not reachable
# yet: this waits for it, and restarts the driver if the link drops.
if [[ -z "$BF_HOST" ]]; then
    BF_HOST=$(sed -n 's/^ *host: *"\{0,1\}\([^"]*\)"\{0,1\}.*/\1/p' "$BF_CONFIG" | head -1)
fi
if (( WITH_BLICKFELD )); then
    BLICKFELD_CMD="$ENV && echo 'Blickfeld at $BF_HOST' && \
trap 'exit 130' INT TERM; while true; do \
  until timeout 2 bash -c '</dev/tcp/$BF_HOST/8000' 2>/dev/null; do echo \"\$(date +%T) waiting for the Blickfeld at $BF_HOST:8000 ...\"; sleep 5; done; \
  ros2 run blickfeld_driver blickfeld_driver_node --ros-args -r __node:=bf_lidar --params-file $BF_CONFIG -p host:=$BF_HOST; \
  echo \"\$(date +%T) blickfeld driver exited -- restarting in 5 s (Ctrl-C stops)\"; sleep 5; \
done"
else
    BLICKFELD_CMD="echo 'Blickfeld not started (--no-blickfeld)'"
fi

# -- SF45/B -------------------------------------------------------------------
if (( WITH_SF45B )); then
    SF45B_CMD="$ENV && \
trap 'exit 130' INT TERM; while true; do \
  until [[ -e $SF45B_PORT ]]; do echo \"\$(date +%T) waiting for the SF45/B on $SF45B_PORT ...\"; sleep 5; done; \
  ros2 launch lightwarelidar sf45b.launch.py port:=$SF45B_PORT; \
  echo \"\$(date +%T) sf45b driver exited -- restarting in 5 s (Ctrl-C stops)\"; sleep 5; \
done"
else
    SF45B_CMD="echo 'SF45/B not started (--no-sf45b)'"
fi

# -- camera -------------------------------------------------------------------
CAMERA_CMD="$ENV && mkdir -p $RECORD_DIR && ros2 launch fcb_orin_driver fcb_driver.launch.py \
fc_source:=$FC_SOURCE publish_compressed:=true encoder:=$FCB_ENCODER record_dir:=$RECORD_DIR"

CONTROL_CMD="$ENV && cat <<'EOF'
Zoom and imaging mode, by service call. Keys here (they call the same services):
  1 RGB   2 IR   3 RGB+IR   4 AUTO   i cycle      z <ratio> Enter   exact zoom
  + / -   one 0.5x detent             a           zoom back to the RC knob
  s snapshot (drawn here)   r start / r r stop    x x stop and name the flight
From any shell:
  ros2 service call /fcb_driver/set_zoom fcb_orin_msgs/srv/SetZoom '{ratio: 7.3}'
  ros2 service call /fcb_driver/set_imaging_mode fcb_orin_msgs/srv/SetImagingMode '{mode: ir}'
  ros2 service call /fcb_driver/zoom_to_rc std_srvs/srv/Trigger
Ctrl-C leaves the keys only; the driver and any recording carry on.
EOF
ros2 run fcb_orin_driver fcb_keys; echo 'keys stopped -- run: ros2 run fcb_orin_driver fcb_keys'; exec bash"

# Each window keeps a shell after its program ends, so a crash leaves its
# last output on screen instead of the window vanishing.
tmux new-session -d -s "$SESSION" -n mavros "bash -c '$(q "$MAVROS_CMD"); exec bash'"
sleep 2
tmux new-window -d -t "$SESSION" -n blickfeld "bash -c '$(q "$BLICKFELD_CMD"); exec bash'"
tmux new-window -d -t "$SESSION" -n sf45b "bash -c '$(q "$SF45B_CMD"); exec bash'"
if (( WITH_CAMERA )); then
    tmux new-window -d -t "$SESSION" -n camera "bash -c '$(q "$CAMERA_CMD"); exec bash'"
    tmux new-window -t "$SESSION" -n control "bash -c '$(q "$CONTROL_CMD")'"
fi
if (( WITH_BAG )); then
    tmux new-window -d -t "$SESSION" -n bag "bash -c '$(q "$(bag_command)")'"
fi
tmux select-window -t "$SESSION:control" 2>/dev/null || tmux select-window -t "$SESSION:bag" 2>/dev/null

echo "started tmux session '$SESSION' -- attaching (Ctrl-b d detaches, Ctrl-b <n> switches window)"
exec tmux attach -t "$SESSION"
