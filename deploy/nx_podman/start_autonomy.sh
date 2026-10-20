#!/usr/bin/env bash
# Start the whole autonomy stack from the host: MAVROS, Blickfeld, lirovo4 (GenZ + Nav2),
# rover_console, in tmux session rover_stack2 inside the rover_humble container.
#
#   ./start_autonomy.sh            start (fresh) and attach to tmux
#   ./start_autonomy.sh --viewer   also start the costmap viewer in pane 3 (http://<rover-ip>:8095/)
#   ./start_autonomy.sh --detach   start without attaching (attach later, see below)
#
# Wraps lirovo/lirovo/start_hardware_stack2.sh, which must run inside rover_humble (the host has
# only Jazzy) and whose defaults do not fit this rover: ODOM_SOURCE=mavros (needs GPS, so Nav2
# never starts indoors), FCU_URL without :115200, and a ttyACM0..9 probe when no Cube is found,
# which can hang the FCB camera board. This script passes FCU_URL from the Cube's by-id name,
# ODOM_SOURCE=genz and the Wi-Fi + lidar CycloneDDS config, and refuses to start without the Cube.
#
# Overrides: ODOM_SOURCE=mavros|genz (default genz), GCS_URL=udp://@<gcs-ip>:14555.
# A running rover_stack2 session is killed and started fresh. Full guide: AUTONOMY_RUN.md
set -euo pipefail

CONTAINER=rover_humble
SESSION=rover_stack2
S=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo
DDS=file://$S/stack2/cyclonedds_wifi2.xml
STARTUP_LOG=/home/rover1/rover_ws/hardware_stack2_startup.log
VIEWER=0
ATTACH=1

for arg in "$@"; do
    case "$arg" in
        --viewer) VIEWER=1 ;;
        --detach) ATTACH=0 ;;
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg (see --help)" >&2; exit 1 ;;
    esac
done

if ! sudo podman ps --format '{{.Names}}' | grep -x "$CONTAINER" >/dev/null; then
    echo "Container $CONTAINER is not running. Start it first:  ./rover.sh up rover" >&2
    exit 1
fi

CUBE_ID=$(ls /dev/serial/by-id/usb-CubePilot_*-if00 2>/dev/null | head -1 || true)
if [ -z "$CUBE_ID" ]; then
    echo "No Cube found (/dev/serial/by-id/usb-CubePilot_*-if00). Plug in its USB and try again." >&2
    echo "(Not starting: without the Cube the stack script would probe ttyACM ports, including the camera's.)" >&2
    exit 1
fi
CUBE=$(readlink -f "$CUBE_ID")
echo "Cube:  $CUBE"

if [ "$(cat /sys/class/net/enP8p1s0/carrier 2>/dev/null || echo 0)" = 1 ]; then
    echo "Lidar: Ethernet link up"
else
    echo "Lidar: NO Ethernet link on enP8p1s0. Check 12 V and the cable; the link needs ~90 s after power-on."
    echo "       The stack still starts: Nav2 waits for GenZ. Once the link is up, restart pane 1 (Up, Enter)."
fi

if sudo podman exec "$CONTAINER" tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session $SESSION is running: it will be killed and started fresh."
fi

ODOM_SOURCE="${ODOM_SOURCE:-genz}"
echo "Odometry: $ODOM_SOURCE   GCS: ${GCS_URL:-none}"

if [ "$VIEWER" = 1 ]; then
    # The stack script types its own placeholder into pane 3 a few seconds in, so wait until it
    # has finished before starting the viewer there.
    START_LINES=$(wc -l < "$STARTUP_LOG" 2>/dev/null || echo 0)
    (
        for _ in $(seq 1 60); do
            tail -n +"$((START_LINES + 1))" "$STARTUP_LOG" 2>/dev/null | grep "Orchestration finished" >/dev/null && break
            sleep 1
        done
        sudo podman exec "$CONTAINER" tmux send-keys -t "$SESSION:main.3" \
            "DISPLAY=:1001 XAUTHORITY=/home/rover1/.Xauthority python3 /home/rover1/podman_rover/costmap_view.py --http 8095" Enter
    ) >/dev/null 2>&1 &
    IP=$(ip -4 -o addr show wlx00c0cab86654 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
    echo "Viewer: http://${IP:-<rover-ip>}:8095/ (in pane 3, ~35% CPU)"
fi

ENVS=(-e "FCU_URL=$CUBE:115200" -e "ODOM_SOURCE=$ODOM_SOURCE" -e "CYCLONEDDS_URI=$DDS" -e DISPLAY=:1001)
[ -n "${GCS_URL:-}" ] && ENVS+=(-e "GCS_URL=$GCS_URL")

if [ "$ATTACH" = 1 ] && [ -t 0 ] && [ -t 1 ]; then
    echo "Attaching to tmux: Alt+arrows switch panes, Ctrl-b d detaches (the stack keeps running)."
    exec sudo podman exec -it "${ENVS[@]}" "$CONTAINER" bash "$S/lirovo/start_hardware_stack2.sh"
fi

sudo podman exec "${ENVS[@]}" "$CONTAINER" bash "$S/lirovo/start_hardware_stack2.sh" >/dev/null
[ "$VIEWER" = 1 ] && wait
echo "Started. Attach with:  sudo podman exec -it $CONTAINER tmux attach -t $SESSION"
