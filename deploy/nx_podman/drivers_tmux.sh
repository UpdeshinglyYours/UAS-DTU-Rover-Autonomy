#!/usr/bin/env bash
# drivers_tmux.sh -- the rover's recording stack in one tmux window, one pane each:
#
#   +---------------------------+---------------------------+
#   | Blickfeld LiDAR           | MAVROS (Cube)             |
#   |   rover_humble            |   rover_humble            |
#   +---------------------------+---------------------------+
#   | FCB camera driver         | control shell (Humble)    |
#   |   r = AVI start, r r stop |   bag command pre-typed   |
#   +---------------------------+---------------------------+
#
#   ./drivers_tmux.sh            start everything and attach (re-attach if running)
#   ./drivers_tmux.sh --stop     stop Blickfeld, MAVROS and the bag, then ask about the camera
#   ./drivers_tmux.sh --stop -y  stop everything, camera included, without asking
#   ./drivers_tmux.sh zoom [X]   set the camera zoom to X (default 1.0)
#
# FCB_DRIVER picks the camera driver:
#   standalone (default)  ~/fcb_orin/fcb_record.py on the host, keys in its pane:
#                         r start AVI, r r stop (offers a name), z zoom, ? status.
#                         No ROS topics: the bag holds the LiDAR + MAVROS, the video
#                         is the AVI. Pinned to the board's VISCA port (if00): probing
#                         the other port (if04) hangs the board's USB control side.
#   ros                   fcb_orin_driver in the fcb_jazzy container (publishes
#                         /fcb_driver/image_raw/compressed for the bag; AVI by service).
# Only one may hold the camera. Switching needs a 12 V power cycle of the camera.
#
# The camera is set to FCB_START_ZOOM (default 1.0x) once its driver is up.
#
# Mouse: click a pane to select it, drag a border to resize, wheel to scroll back.
# Keys:  Alt+arrows  move between panes      Alt+z  zoom a pane in/out
#        Ctrl-b d    detach (everything keeps running)
#        Ctrl-c      in the Blickfeld / MAVROS pane stops that driver
# The FCB pane shows the recorder's own tmux session (nested): type its keys there
# directly; its Ctrl-b is Ctrl-b Ctrl-b.
#
# Environment: FCB_DRIVER, FCB_START_ZOOM, MAV_SYSID (default 1), FCU_URL (default:
# the CubePilot by-id port as /dev/ttyACMx:115200), FCB_ARGS (extra driver args).
set -u

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
DIR="$(dirname "$SELF")"
SESSION=rover_drivers
P="sudo podman"
ROVER=rover_humble
FCB=fcb_jazzy
FCB_DRIVER="${FCB_DRIVER:-standalone}"
STACK2=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2
FCB_ORIN="$HOME/fcb_orin"
VISCA_PORT=/dev/serial/by-id/usb-Cypress_FX3_CAMERA-if00

in_humble() {   # run a command line inside rover_humble, ROS + workspace sourced (.bashrc)
    exec $P exec -it $ROVER bash -ic "$1"
}

running() { [[ "$($P inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == true ]]; }

standalone_running() { pgrep -f "python3 -u $FCB_ORIN/fcb_record.py" >/dev/null; }

banner() { printf '\e[1;36m== %s ==\e[0m\n' "$*"; }

# Zoom: the board holds VISCA replies until 32 bytes build up, so fcb_zoom.py sends
# the zoom command without waiting for an answer; it works next to either driver.
set_zoom() { "$DIR/fcb_zoom.py" "$1" --no-confirm; }

zoom_on_start() {   # after the driver has opened the camera, drive the lens to FCB_START_ZOOM
    sleep 10
    printf '\e[1;36m== start zoom: %s ==\e[0m\n' "$(set_zoom "${FCB_START_ZOOM:-1.0}" 2>&1 | tail -1)"
}

case "${1:-}" in
# ---------------------------------------------------------------- pane programs
run-blickfeld)
    banner "Blickfeld LiDAR -> /bf_lidar/point_cloud_out, /bf_lidar/imu_out   (Ctrl-c stops it)"
    in_humble "ros2 launch blickfeld_driver live_scanner_node.launch.py" ;;
run-mavros)
    FCU="${FCU_URL:-}"
    if [[ -z "$FCU" ]]; then
        CUBE=$(ls /dev/serial/by-id/usb-CubePilot_*-if00 2>/dev/null | head -1)
        [[ -n "$CUBE" ]] || { echo "no CubePilot on /dev/serial/by-id -- plug in the Cube, then: $SELF run-mavros"; exit 1; }
        FCU="$(readlink -f "$CUBE"):115200"   # the patched libmavconn wants ttyACMx:baud
    fi
    banner "MAVROS on $FCU, system ${MAV_SYSID:-1}, plugin allowlist   (Ctrl-c stops it)"
    # "gcs_url:=''": the inner quotes must reach ros2 launch (an empty YAML string)
    in_humble "ros2 launch mavros node.launch fcu_url:=$FCU \"gcs_url:=''\" \
tgt_system:=${MAV_SYSID:-1} tgt_component:=1 \
pluginlists_yaml:=$STACK2/mavros_apm_pluginlists.yaml \
config_yaml:=\$(ros2 pkg prefix mavros)/share/mavros/launch/apm_config.yaml" ;;
run-fcb)
    if [[ "$FCB_DRIVER" == ros ]]; then
        if running $FCB; then
            banner "FCB ROS driver already running in $FCB -- following its log"
        else
            banner "starting the FCB ROS driver in $FCB"
            "$DIR/rover.sh" up fcb || exit 1
            zoom_on_start &
        fi
        banner "FCB log. Ctrl-c stops only this view; the driver keeps running (12 V cycle otherwise)"
        exec $P logs -f --tail 40 $FCB
    fi
    if ! standalone_running; then
        banner "starting the standalone FCB driver (fcb_record.py), VISCA pinned to if00"
        ( zoom_on_start & )
    else
        banner "standalone FCB driver already running -- attaching to it"
    fi
    banner "keys: r start AVI | r r stop | z zoom | 1-4 RGB/IR | s snapshot | ? status"
    # start_recorder.sh supervises fcb_record.py in its own tmux session `fcb` and
    # attaches to it; TMUX unset so it nests here instead of switching clients.
    exec env -u TMUX "$FCB_ORIN/start_recorder.sh" -- --no-mavlink \
        --port "$VISCA_PORT" --baud 9600 ${FCB_ARGS:-} ;;
run-control)
    cat <<EOF
== control shell (Humble, inside rover_humble) ==
  camera driver      FCB_DRIVER=$FCB_DRIVER
  AVI start/stop     standalone: r / r r in the FCB pane
                     ros:        ros2 service call /fcb_driver/start_recording std_srvs/srv/Trigger
                                 ros2 service call /fcb_driver/stop_recording fcb_orin_msgs/srv/StopRecording
  zoom (host shell)  ~/podman_rover/drivers_tmux.sh zoom 1
  check rates        ros2 topic hz /bf_lidar/point_cloud_out
                     ros2 topic hz /mavros/imu/data
  bag                pre-typed below: press Enter to start, Ctrl-c to stop
EOF
    exec $P exec -it $ROVER bash ;;
# ---------------------------------------------------------------- commands
zoom)
    set_zoom "${2:-1.0}" || exit 1
    exit 0 ;;
--stop)
    for pat in "ros2 bag record" blickfeld_driver mavros_node rqt_image_view; do
        $P exec $ROVER pkill -INT -f "$pat" 2>/dev/null && echo "stopped: $pat"
    done
    sleep 2
    tmux kill-session -t $SESSION 2>/dev/null && echo "tmux session $SESSION closed"
    if standalone_running || running $FCB; then
        if [[ "${2:-}" != -y ]]; then
            echo "The camera board needs a 12 V power cycle after its stream is closed."
            read -r -p "Stop the FCB driver too? [y/N] " a
            [[ "$a" == [yY]* ]] || { echo "FCB driver left running"; exit 0; }
        fi
        # both finish an open AVI before exiting
        standalone_running && "$FCB_ORIN/start_recorder.sh" --stop
        running $FCB && $P stop -t 45 $FCB >/dev/null && echo "FCB ROS driver stopped"
    fi
    exit 0 ;;
-h|--help)
    sed -n '2,37p' "$SELF" | sed 's/^# \?//'; exit 0 ;;
"") ;;
*)  echo "unknown option: $1 (see --help)" >&2; exit 1 ;;
esac

# -------------------------------------------------------------------- session
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "session $SESSION is running -- attaching (Ctrl-b d detaches)"
    exec tmux attach -t $SESSION
fi

case "$FCB_DRIVER" in standalone|ros) ;; *)
    echo "FCB_DRIVER must be standalone or ros (got '$FCB_DRIVER')" >&2; exit 1 ;; esac

# The Orin's RTC loses the time on a cold boot: file names and bag stamps would be 1970.
if (( $(date +%Y) < 2025 )); then
    echo "error: the clock says $(date). Set it first, from a laptop:" >&2
    echo "  ssh rover1@<orin> \"sudo date -u -s '\$(date -u '+%F %T')'\"" >&2
    exit 1
fi

# Only one program may hold the camera.
if [[ "$FCB_DRIVER" == standalone ]] && running $FCB; then
    echo "error: the ROS camera driver ($FCB) holds the camera. Stop it (sudo podman stop -t 45 $FCB)," >&2
    echo "power-cycle the camera's 12 V, then run this again -- or use FCB_DRIVER=ros." >&2
    exit 1
fi
if [[ "$FCB_DRIVER" == ros ]] && standalone_running; then
    echo "error: the standalone driver holds the camera ($FCB_ORIN/start_recorder.sh --stop," >&2
    echo "power-cycle the camera's 12 V), or use FCB_DRIVER=standalone." >&2
    exit 1
fi

# One MAVROS / Blickfeld at a time: rover.sh rec (rover_rec) or start_hardware_stack2
# (rover_stack2) would publish the same topics.
"$DIR/rover.sh" up rover || exit 1
if $P exec $ROVER bash -c 'tmux ls 2>/dev/null' | grep -qE '^(rover_rec|rover_stack2):'; then
    echo "error: rover_rec / rover_stack2 is running in $ROVER (its own MAVROS + Blickfeld)." >&2
    echo "Stop it first: ./rover.sh rec --stop, or tmux kill-session inside $ROVER." >&2
    exit 1
fi
for pat in mavros_node blickfeld_driver; do
    if $P exec $ROVER pgrep -f "$pat" >/dev/null 2>&1; then
        echo "error: $pat already runs in $ROVER -- ./drivers_tmux.sh --stop first" >&2
        exit 1
    fi
done

pane_cmd() { echo "bash -c '\"$SELF\" $1; echo; echo \"[$1 ended -- restart: $SELF $1]\"; exec bash'"; }

# pane ids (%N) stay fixed whatever the layout numbers the panes
BF=$(tmux new-session -d -P -F '#{pane_id}' -s $SESSION -n drivers -x 220 -y 56 \
     -e FCB_DRIVER="$FCB_DRIVER" "$(pane_cmd run-blickfeld)")
MAV=$(tmux split-window -h -P -F '#{pane_id}' -t "$BF" "$(pane_cmd run-mavros)")
CAM=$(tmux split-window -v -P -F '#{pane_id}' -t "$BF" "$(pane_cmd run-fcb)")
CTL=$(tmux split-window -v -P -F '#{pane_id}' -t "$MAV" "$(pane_cmd run-control)")
tmux select-pane -t "$BF"  -T "Blickfeld"
tmux select-pane -t "$MAV" -T "MAVROS"
tmux select-pane -t "$CAM" -T "FCB camera ($FCB_DRIVER)"
tmux select-pane -t "$CTL" -T "control (Humble)"

tmux set-option -t $SESSION mouse on
tmux set-option -t $SESSION history-limit 20000
tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format ' #{pane_index} #{pane_title} '
tmux set-option -t $SESSION status-right ' Alt+arrows move | Alt+z zoom | Ctrl-b d detach '
tmux set-option -t $SESSION status-right-length 60
tmux bind-key -n M-Left  select-pane -L
tmux bind-key -n M-Right select-pane -R
tmux bind-key -n M-Up    select-pane -U
tmux bind-key -n M-Down  select-pane -D
tmux bind-key -n M-z     resize-pane -Z

# the bag command, typed but not run: Enter starts it in the control pane
TOPICS="/bf_lidar/point_cloud_out /bf_lidar/imu_out /tf_static \
/mavros/imu/data /mavros/state /mavros/global_position/raw/fix"
[[ "$FCB_DRIVER" == ros ]] && TOPICS="/fcb_driver/image_raw/compressed /fcb_driver/status $TOPICS"
BAG="ros2 bag record -s mcap -o /workspace/rover_ws/bags/rover-\$(date +%Y%m%d-%H%M%S) $TOPICS"
( sleep 6; tmux send-keys -t "$CTL" "$BAG" ) &

tmux select-pane -t "$CAM"
if [[ -t 1 ]]; then
    exec tmux attach -t $SESSION
else
    echo "started tmux session $SESSION -- attach with: tmux attach -t $SESSION"
fi
