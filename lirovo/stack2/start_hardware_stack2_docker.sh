#!/usr/bin/env bash
#
# start_hardware_stack2_docker.sh -- ~/start_hardware_stack2.sh (laptop) for the uas_rover container,
# running the laptop's lirovo3 stack. Run it INSIDE the container (it builds a tmux session there):
#
#   docker exec -it uas_rover_container bash \
#       /workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2/start_hardware_stack2_docker.sh
#
# Panes (same layout as the laptop script):
#     0 MAVROS          1 Blickfeld
#     2 SF45/B          3 RViz (off by default on the Orin, see RVIZ)
#     4 lirovo3         5 rover_console (keyboard E-STOP / HOLD)
#
# Differences from the laptop script, and why:
#   * MAVROS: FCU_URL defaults to the Cube's /dev/serial/by-id name (ttyACM numbers move when the NeoHD
#     camera or the Pico mic is plugged in); falls back to probing ttyACM0..9 like the laptop script.
#     Plugin list = mavros_apm_pluginlists.yaml in this folder: an ALLOWLIST (sys_status, sys_time,
#     command, imu, local_position, global_position, rc_io, setpoint_velocity). The laptop's denylist
#     loads ~40 plugins, which on the NX with the Cube's 490 Hz streams pegged mavros_node at 173 % CPU
#     and dropped /mavros/state to 0.5 Hz (measured 2026-10-16).
#   * Blickfeld: the container's blickfeld_driver live_scanner_node.launch.py (node bf_lidar, host
#     192.168.26.26, same publish_* / IMU / timestamp settings as the laptop's ros2 run command).
#     No NetworkManager restart: the static-IP link belongs to the host, not the container.
#   * SF45/B: the container has the `lightwarelidar` driver, not the laptop's `lightwarelidar2`.
#     namespace:=sf45b + frame_id:=sf45b give the same /sf45b/scan topic and frame as the laptop's
#     `-r scan:=/sf45b/scan -p frameId:=sf45b`; updateRate 6 and +-45 deg match the laptop.
#   * No lidar gate: lirovo3 starts 5 s after the drivers, without waiting for
#     /bf_lidar/point_cloud_out and /sf45b/scan (the laptop waits up to 30 s for each and aborts).
#   * lirovo3.launch.py and nav2_params3.yaml come from this folder (see lirovo3.launch.py).
#   * RViz runs on the laptop over CycloneDDS by default (RVIZ=1 starts it here on DISPLAY).
#   * new_omega_fixed5.py is NOT started (same as the laptop). This folder has the laptop copy, which,
#     unlike lirovo/lirovo/new_omega_fixed5.py, stops itself when the mode leaves MANUAL:
#         python3 /workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/stack2/new_omega_fixed5.py
#     Its velocity feedback always comes from /mavros/local_position/odom, whatever ODOM_SOURCE is.
#   * ODOM_SOURCE picks the pose source of lirovo3's odom_to_tf (map->base_link):
#         mavros (default)  /mavros/local_position/odom  (Cube EKF, needs a GPS fix / EKF origin)
#         genz              /genz/odometry  (GenZ-ICP; lirovo3 starts its pipeline)
#
# GCS_URL forwards MAVROS's MAVLink to a ground station, e.g. GCS_URL=udp://@192.168.0.41:14555 (MAVROS's
# router passes every FCU message there and the GCS's commands back; no second program on the serial port).
#
# Environment overrides: ODOM_SOURCE=mavros|genz, GCS_URL, FCU_URL, SF45B_PORT, ROS_DOMAIN_ID (default 2, same as
# the laptop's ~/.bashrc), RVIZ=0|1, SESSION.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${HERE}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"

SESSION="${SESSION:-rover_stack2}"
LOG_FILE="${WS_DIR}/hardware_stack2_docker_startup.log"   # in the mounted workspace (survives --rm)

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
RVIZ="${RVIZ:-0}"
ODOM_SOURCE="${ODOM_SOURCE:-mavros}"
GCS_URL="${GCS_URL:-}"
if [ "$ODOM_SOURCE" != "mavros" ] && [ "$ODOM_SOURCE" != "genz" ]; then
    echo "ODOM_SOURCE must be 'mavros' or 'genz' (got '${ODOM_SOURCE}')" >&2
    exit 1
fi
SF45B_PORT="${SF45B_PORT:-/dev/ttyUSB0}"
ENV_SETUP="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=${ROS_DOMAIN_ID} DISPLAY=${DISPLAY:-:0} && source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash &&"

log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
    echo "$msg" | tee -a "$LOG_FILE"
}

pane_send() {
    tmux send-keys -t "${SESSION}:main.$1" "$2" Enter
}

run_orchestration() {
    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/setup.bash"
    log "=========================================="
    log "Orchestration starting (session ${SESSION}, RMW=${RMW_IMPLEMENTATION}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ODOM_SOURCE=${ODOM_SOURCE}, GCS_URL=${GCS_URL:-none})"
    log "=========================================="

    # ---- 1. MAVROS: by-id name of the Cube, else probe ttyACM0..9 ----
    MAVROS_PORT="${FCU_URL:-}"
    if [ -z "$MAVROS_PORT" ]; then
        MAVROS_PORT="$(ls /dev/serial/by-id/usb-CubePilot_*-if00 2>/dev/null | head -1)"
        [ -n "$MAVROS_PORT" ] && log "MAVROS: Cube found at ${MAVROS_PORT}"
    fi
    if [ -z "$MAVROS_PORT" ]; then
        log "MAVROS: no /dev/serial/by-id/usb-CubePilot_* -- probing ttyACM0..9"
        for i in 0 1 2 3 4 5 6 7 8 9; do
            DEV="/dev/ttyACM${i}"
            [ -e "$DEV" ] || continue
            log "MAVROS: probing ${DEV} ..."
            ros2 launch mavros node.launch fcu_url:="${DEV}" "gcs_url:=''" tgt_system:=1 tgt_component:=1 \
                pluginlists_yaml:="${HERE}/mavros_apm_pluginlists.yaml" \
                config_yaml:="$(ros2 pkg prefix mavros)/share/mavros/launch/apm_config.yaml" \
                > "/tmp/mavros_attempt_ACM${i}.log" 2>&1 &
            PROBE_PID=$!
            CONNECTED=false
            for _ in $(seq 1 15); do
                sleep 1
                STATE=$(timeout 4 ros2 topic echo --no-daemon /mavros/state --once --field connected 2>/dev/null | head -1)
                if [ "$STATE" = "true" ] || [ "$STATE" = "True" ]; then CONNECTED=true; break; fi
                kill -0 "$PROBE_PID" 2>/dev/null || break
            done
            kill -INT "$PROBE_PID" 2>/dev/null; wait "$PROBE_PID" 2>/dev/null
            if [ "$CONNECTED" = "true" ]; then MAVROS_PORT="$DEV"; log "MAVROS: probe CONNECTED on ${DEV}"; break; fi
            log "MAVROS: no connection on ${DEV}, trying next port"
        done
    fi
    if [ -n "$MAVROS_PORT" ]; then
        log "MAVROS: starting on ${MAVROS_PORT} in pane 0"
        pane_send 0 "${ENV_SETUP} ros2 launch mavros node.launch fcu_url:=${MAVROS_PORT} \"gcs_url:=${GCS_URL:-''}\" tgt_system:=1 tgt_component:=1 pluginlists_yaml:=${HERE}/mavros_apm_pluginlists.yaml config_yaml:=\$(ros2 pkg prefix mavros)/share/mavros/launch/apm_config.yaml"
        sleep 3
    else
        log "MAVROS: RESULT = FAILED -- no Cube found."
        pane_send 0 "echo '[MAVROS FAILED -- no Cube found. See ${LOG_FILE}]'"
    fi

    # ---- 2. Blickfeld (pane 1) ----
    log "Starting Blickfeld driver (bf_lidar) in pane 1 ..."
    pane_send 1 "${ENV_SETUP} ros2 launch blickfeld_driver live_scanner_node.launch.py 2>&1 | tee /tmp/bf_lidar_driver.log"

    # ---- 2.5. SF45/B (pane 2) ----
    log "Starting SF45/B driver on ${SF45B_PORT} in pane 2 ..."
    pane_send 2 "${ENV_SETUP} ros2 launch lightwarelidar sf45b.launch.py port:=${SF45B_PORT} namespace:=sf45b frame_id:=sf45b update_rate:=6 low_angle_limit:=-45.0 high_angle_limit:=45.0 2>&1 | tee /tmp/sf45b_driver.log"

    # ---- 3. RViz + lirovo3 (no lidar gate: Nav2 starts whether or not both lidars are up yet) ----
    sleep 5
    if [ "$RVIZ" = "1" ]; then
        pane_send 3 "${ENV_SETUP} rviz2 -d \$(ros2 pkg prefix vortex_mission)/share/vortex_mission/rviz/casualty_view.rviz"
    else
        pane_send 3 "echo '[RViz off on the Orin. On the laptop: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp rviz2 -d ~/vortex_ws/vortex_mission/rviz/casualty_view.rviz]'"
    fi
    log "Starting lirovo3 (odom_source:=${ODOM_SOURCE}) in pane 4 (no lidar gate) ..."
    pane_send 4 "${ENV_SETUP} ros2 launch ${HERE}/lirovo3.launch.py odom_source:=${ODOM_SOURCE}"

    # ---- 4. rover_console (pane 5) ----
    if ros2 pkg prefix vortex_mission >/dev/null 2>&1; then
        log "Starting rover_console in pane 5 ..."
        pane_send 5 "${ENV_SETUP} echo '[drive: python3 ${HERE}/new_omega_fixed5.py]' && ros2 run vortex_mission rover_console"
    else
        log "rover_console: vortex_mission not built (colcon build --packages-select vortex_interfaces vortex_mission)"
        pane_send 5 "echo '[vortex_mission not built -- no rover_console / keyboard E-STOP]'"
    fi

    log "=========================================="
    log "Orchestration finished. Full log: ${LOG_FILE}"
    log "=========================================="
}

# ---- tmux grid first, so it is visible whatever the orchestration does ----
if tmux has-session -t "$SESSION" 2>/dev/null; then
    log "Session '${SESSION}' exists -- killing it and starting fresh."
    tmux kill-session -t "$SESSION"
fi
tmux new-session -d -s "$SESSION" -n main
tmux set-option -g mouse on
tmux set-option -g remain-on-exit on
tmux bind-key -n M-Left  select-pane -L
tmux bind-key -n M-Right select-pane -R
tmux bind-key -n M-Up    select-pane -U
tmux bind-key -n M-Down  select-pane -D
tmux split-window -h -t "${SESSION}:main"
tmux split-window -h -t "${SESSION}:main.1"
tmux split-window -v -t "${SESSION}:main.0"
tmux split-window -v -t "${SESSION}:main.1"
tmux split-window -v -t "${SESSION}:main.4"
tmux select-layout -t "${SESSION}:main" tiled

pane_send 0 "echo '[MAVROS pane -- waiting for the Cube]'"
pane_send 1 "echo '[Blickfeld pane -- waiting]'"
pane_send 2 "echo '[SF45/B pane -- waiting]'"
pane_send 3 "echo '[RViz pane -- waiting]'"
pane_send 4 "echo '[lirovo3 pane -- waiting]'"
pane_send 5 "echo '[rover_console pane -- starts after the hardware checks]'"
log "tmux grid created."

nohup bash -c "$(declare -f log pane_send run_orchestration); HERE='${HERE}' WS_DIR='${WS_DIR}' SESSION='${SESSION}' LOG_FILE='${LOG_FILE}' ENV_SETUP='${ENV_SETUP}' ODOM_SOURCE='${ODOM_SOURCE}' GCS_URL='${GCS_URL}' FCU_URL='${FCU_URL:-}' SF45B_PORT='${SF45B_PORT}' RVIZ='${RVIZ}' run_orchestration" >/dev/null 2>&1 &

log "Attaching to ${SESSION} (Ctrl-b d to detach without stopping it) ..."
if [ -t 1 ]; then
    tmux attach -t "$SESSION"
else
    log "No terminal here: attach with   docker exec -it uas_rover_container tmux attach -t ${SESSION}"
fi
