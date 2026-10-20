#!/usr/bin/env bash
# A rosbag of the FCB camera + Blickfeld LiDAR, recorded INSIDE fcb_jazzy
# (ROS 2 Jazzy). The Blickfeld driver runs in rover_humble; its topics reach
# Jazzy over the shared loopback CycloneDDS profile (rover.sh, dds_args), so
# one recorder in Jazzy gets both, on one clock.
#
#   ./jazzy_bag.sh start [name]   start a bag in the background
#                                 (~/rover_ws/bags/<name>, default jazzy-<date>)
#   ./jazzy_bag.sh stop           stop it cleanly (SIGINT: bag closed, metadata written)
#   ./jazzy_bag.sh status         running? which bag, how big; last bag's topic counts
#   ./jazzy_bag.sh topics         what would be recorded right now
#
# Camera topic: /fcb_driver/image_raw/compressed when the driver publishes it
# (rover.sh's launch: publish_compressed:=true), else /fcb_driver/image_raw +
# camera_info (e.g. the calibration launch: publish_compressed:=false,
# publish_rate_hz:=2). Raw 1080p is ~6 MB a frame: only sensible throttled.
#
# Environment:
#   CAM_TOPICS="..."     camera topics instead of the automatic choice
#   LIDAR_TOPICS="..."   default: /bf_lidar/point_cloud_out /bf_lidar/imu_out
#   EXTRA_TOPICS="..."   anything else to add (e.g. /tf /tf_static /mavros/state)
#   BAG_DIR              default ~/rover_ws/bags (next to rover.sh rec's rover-* bags)
#
# Needs fcb_jazzy up (./rover.sh up fcb) and the Blickfeld driver running in
# rover_humble (./rover.sh rec starts it; that also starts Humble's own bag in
# its 'bag' window -- Ctrl-C that window if you only want this one).
set -u

P="sudo podman"
FCB=fcb_jazzy
U_HOME=/home/rover1
BAG_DIR="${BAG_DIR:-$U_HOME/rover_ws/bags}"
STATE=$U_HOME/.jazzy_bag.current          # path of the bag being recorded
LIDAR_TOPICS="${LIDAR_TOPICS:-/bf_lidar/point_cloud_out /bf_lidar/imu_out}"

running() { [[ "$($P inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == true ]]; }

# a command inside fcb_jazzy, ROS sourced; the entrypoint drops to rover1
# (FCB_UID from the container env), so the bags belong to rover1, not root
jz() {
    $P exec $FCB \
        /usr/local/bin/fcb_entrypoint.sh "$@"
}

# the recorder itself (python .../bin/ros2), not a shell wrapping it
recorder_pid() { $P exec $FCB pgrep -f "/ros2 bag record .*-o $BAG_DIR/" 2>/dev/null | head -1; }

pick_topics() {
    local live cam=() t
    live=$(jz bash -c "ros2 topic list --no-daemon --spin-time 3 2>/dev/null")
    if [[ -n "${CAM_TOPICS:-}" ]]; then
        cam=($CAM_TOPICS)
    elif grep -qx /fcb_driver/image_raw/compressed <<<"$live"; then
        cam=(/fcb_driver/image_raw/compressed)
    elif grep -qx /fcb_driver/image_raw <<<"$live"; then
        cam=(/fcb_driver/image_raw /fcb_driver/camera_info)
    fi
    for t in "${cam[@]}" $LIDAR_TOPICS ${EXTRA_TOPICS:-} /fcb_driver/status /tf_static; do
        if grep -qx "$t" <<<"$live"; then echo "$t"; else echo "MISSING $t"; fi
    done
}

cmd_start() {
    running $FCB || { echo "$FCB is not running: ./rover.sh up fcb" >&2; exit 1; }
    local pid; pid=$(recorder_pid)
    [[ -n "$pid" ]] && { echo "already recording: $(cat "$STATE" 2>/dev/null) -- ./jazzy_bag.sh stop" >&2; exit 1; }

    local all topics=() missing=() t
    all=$(pick_topics)
    while read -r t; do
        [[ -z "$t" ]] && continue
        if [[ $t == MISSING* ]]; then missing+=("${t#MISSING }"); else topics+=("$t"); fi
    done <<<"$all"
    printf '%s\n' "${topics[@]}" | grep -q '^/fcb_driver/image_raw' \
        || echo "warning: no camera topic on the graph (driver up? ./rover.sh logs fcb)" >&2
    printf '%s\n' "${topics[@]}" | grep -qx /bf_lidar/point_cloud_out \
        || echo "warning: no /bf_lidar/point_cloud_out (Blickfeld not running: ./rover.sh rec)" >&2
    for t in "${missing[@]}"; do
        # absent now: recorded anyway, rosbag2 picks a topic up when it appears
        echo "note: $t not published yet -- recorded if it appears" >&2
        topics+=("$t")
    done
    (( ${#topics[@]} )) || { echo "nothing to record" >&2; exit 1; }

    local name="${1:-jazzy-$(date +%Y%m%d-%H%M%S)}"
    local bag="$BAG_DIR/$name"
    [[ -e "$bag" ]] && { echo "error: $bag exists" >&2; exit 1; }
    mkdir -p "$BAG_DIR"
    echo "recording in $FCB -> $bag"
    printf '  %s\n' "${topics[@]}"
    # -d: survives this terminal / an ssh drop. The cache is raised from 100 MB
    # so a burst of point clouds or frames never waits on the disk.
    $P exec -d $FCB \
        /usr/local/bin/fcb_entrypoint.sh bash -c \
        "exec ros2 bag record -s mcap --max-cache-size 268435456 -o $bag --topics ${topics[*]} > $BAG_DIR/.$name.log 2>&1"
    echo "$bag" > "$STATE"
    sleep 4
    if [[ -n "$(recorder_pid)" ]]; then
        echo "started. ./jazzy_bag.sh status | ./jazzy_bag.sh stop"
    else
        echo "error: the recorder exited:" >&2; tail -20 "$BAG_DIR/.$name.log" >&2; rm -f "$STATE"; exit 1
    fi
}

cmd_stop() {
    local pid bag; pid=$(recorder_pid); bag=$(cat "$STATE" 2>/dev/null)
    [[ -z "$pid" ]] && { echo "not recording"; rm -f "$STATE"; return 0; }
    echo "stopping (SIGINT) ..."
    $P exec $FCB kill -INT "$pid"
    for _ in $(seq 1 30); do [[ -z "$(recorder_pid)" ]] && break; sleep 1; done
    if [[ -n "$(recorder_pid)" ]]; then
        echo "still running after 30 s -- killing; fix the bag with: ros2 bag reindex $bag" >&2
        $P exec $FCB kill -KILL "$(recorder_pid)"
    fi
    rm -f "$STATE"
    [[ -n "$bag" ]] && { echo "bag: $bag ($(du -sh "$bag" | cut -f1))"; info "$bag"; }
}

info() {
    jz ros2 bag info "$1" 2>/dev/null | sed -n '/^Duration/p; /^Messages/p; /Topic:/s/^ *//p'
}

cmd_status() {
    local pid bag; pid=$(recorder_pid); bag=$(cat "$STATE" 2>/dev/null)
    if [[ -n "$pid" ]]; then
        echo "recording -> $bag ($(du -sh "$bag" 2>/dev/null | cut -f1))"
    else
        echo "not recording"
        bag=$(ls -dt "$BAG_DIR"/jazzy-* 2>/dev/null | head -1)
        [[ -n "$bag" ]] && { echo "last: $bag ($(du -sh "$bag" | cut -f1))"; info "$bag"; }
    fi
}

case "${1:-}" in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    topics) running $FCB || { echo "$FCB is not running" >&2; exit 1; }; pick_topics ;;
    *)      sed -n '2,/^set -u$/p' "$(readlink -f "$0")" | sed '$d; s/^# \?//'; exit 1 ;;
esac
