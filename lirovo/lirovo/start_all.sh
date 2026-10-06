#!/usr/bin/env bash
# ==============================================================================
# UAS Rover Master Bringup Script
# Launches:
#   1. MAVROS (ArduPilot / Pixhawk)
#   2. Blickfeld LiDAR Driver
#   3. LightWare SF45/B LiDAR Driver
#   4. LIROVO Navigation Stack (lirovo.launch.py)
#   5. RViz2 (nav2_default_view.rviz)
#
# GUARANTEED INSTANT SHUTDOWN:
#   Pressing Ctrl+C instantly terminates all launch files, nodes, and RViz
#   without leaving orphaned or zombie processes.
# ==============================================================================

set -m  # Enable job control / process groups

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." 2>/dev/null && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../../" 2>/dev/null && pwd)"

# Configuration defaults (can be overridden via CLI args or environment variables)
FCU_URL="${1:-${FCU_URL:-/dev/ttyACM0}}"
SF45B_PORT="${2:-${SF45B_PORT:-/dev/ttyUSB0}}"

# Auto-locate RViz configuration
RVIZ_CONFIG="${REPO_DIR}/nav2_default_view.rviz"
if [ ! -f "${RVIZ_CONFIG}" ]; then
    RVIZ_CONFIG="$(find "${WORKSPACE_DIR}" -name "nav2_default_view.rviz" 2>/dev/null | head -n 1)"
fi

# Track all launched background PIDs
PIDS=()

# -----------------------------------------------------------------------------
# Clean & Instant Termination Trap
# -----------------------------------------------------------------------------
cleanup() {
    # Ignore subsequent signals so trap doesn't re-trigger
    trap '' SIGINT SIGTERM EXIT
    echo -e "\n\033[1;31m=========================================================="
    echo " [SHUTDOWN] Terminating all UAS Rover nodes instantly..."
    echo -e "==========================================================\033[0m"

    # 1. Send SIGTERM to directly recorded PIDs
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # 2. Kill the script's entire process group
    kill -TERM -$$ 2>/dev/null || true

    # 3. Brief 0.2s pause for ROS 2 destructors, then hard SIGKILL
    sleep 0.2
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    # 4. Instant nuke for any orphaned ROS 2 Python/C++ subprocesses
    pkill -9 -f "mavros_node|apm\.launch|live_scanner|blickfeld_driver|sf45b|lirovo\.launch|adaptive_velocity|casualty_dispatcher|odom_to_tf|new_omega|pointcloud_processor|nav2_|rviz2" 2>/dev/null || true

    echo -e "\033[1;32m[SHUTDOWN] All subsystems killed cleanly. Exited.\033[0m\n"
    exit 0
}

# Trap Ctrl+C (SIGINT), SIGTERM, and EXIT
trap cleanup SIGINT SIGTERM EXIT

# -----------------------------------------------------------------------------
# Environment Auto-Sourcing
# -----------------------------------------------------------------------------
if [ -z "${ROS_DISTRO}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
elif [ -f /workspace/DARPA_ws/install/setup.bash ]; then
    source /workspace/DARPA_ws/install/setup.bash
elif [ -f /home/vortex/DARPA_ws/install/setup.bash ]; then
    source /home/vortex/DARPA_ws/install/setup.bash
else
    echo -e "\033[1;31m[ERROR] No compiled workspace install/setup.bash found!\033[0m"
    echo -e "\033[1;33mPlease compile your workspace first inside the container:\033[0m"
    echo -e "   \033[1;36mcd ${WORKSPACE_DIR} && colcon build --symlink-install\033[0m\n"
    exit 1
fi

echo -e "\033[1;34m=========================================================="
echo " UAS Rover Full Autonomy Stack Bringup"
echo " FCU Port:      ${FCU_URL}"
echo " SF45/B Port:   ${SF45B_PORT}"
# echo " RViz Config:   ${RVIZ_CONFIG}"
echo -e "==========================================================\033[0m\n"

# -----------------------------------------------------------------------------
# 1. MAVROS
# -----------------------------------------------------------------------------
# echo -e "\033[1;33m[1/5] Launching MAVROS (FCU: ${FCU_URL})...\033[0m"
echo -e "\033[1;33m[1/4] Launching MAVROS (FCU: ${FCU_URL})...\033[0m"
ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
PIDS+=($!)
sleep 2

# -----------------------------------------------------------------------------
# 2. Blickfeld LiDAR Driver
# -----------------------------------------------------------------------------
# echo -e "\033[1;33m[2/5] Launching Blickfeld LiDAR Driver...\033[0m"
echo -e "\033[1;33m[2/4] Launching Blickfeld LiDAR Driver...\033[0m"
ros2 launch blickfeld_driver live_scanner_node.launch.py &
PIDS+=($!)
sleep 2

# -----------------------------------------------------------------------------
# 3. LightWare SF45/B LiDAR Driver
# -----------------------------------------------------------------------------
# echo -e "\033[1;33m[3/5] Launching LightWare SF45/B LiDAR Driver (Port: ${SF45B_PORT})...\033[0m"
echo -e "\033[1;33m[3/4] Launching LightWare SF45/B LiDAR Driver (Port: ${SF45B_PORT})...\033[0m"
ros2 launch lightwarelidar sf45b.launch.py port:="${SF45B_PORT}" &
PIDS+=($!)
sleep 1

# -----------------------------------------------------------------------------
# 4. LIROVO Navigation Stack
# -----------------------------------------------------------------------------
# echo -e "\033[1;33m[4/5] Launching LIROVO Navigation Stack...\033[0m"
echo -e "\033[1;33m[4/4] Launching LIROVO Navigation Stack...\033[0m"
ros2 launch lirovo lirovo.launch.py &
PIDS+=($!)
sleep 2

# -----------------------------------------------------------------------------
# 5. RViz2 (Disabled per user request - view topics on laptop via CycloneDDS)
# -----------------------------------------------------------------------------
# if [ -n "${DISPLAY}" ] && [ -f "${RVIZ_CONFIG}" ]; then
#     echo -e "\033[1;33m[5/5] Launching RViz2 with ${RVIZ_CONFIG}...\033[0m"
#     rviz2 -d "${RVIZ_CONFIG}" &
#     PIDS+=($!)
# elif [ -n "${DISPLAY}" ]; then
#     echo -e "\033[1;33m[5/5] Launching RViz2 (default config)...\033[0m"
#     rviz2 &
#     PIDS+=($!)
# else
#     echo -e "\033[1;35m[5/5] DISPLAY not set, skipping RViz2 GUI launch.\033[0m"
# fi
# if [ -n "${DISPLAY}" ] && xset q >/dev/null 2>&1; then
#     if [ -f "${RVIZ_CONFIG}" ]; then
#         echo -e "\033[1;33m[5/5] Launching RViz2 with ${RVIZ_CONFIG}...\033[0m"
#         rviz2 -d "${RVIZ_CONFIG}" &
#         PIDS+=($!)
#     else
#         echo -e "\033[1;33m[5/5] Launching RViz2 (default config)...\033[0m"
#         rviz2 &
#         PIDS+=($!)
#     fi
# else
#     echo -e "\033[1;35m[5/5] X11 GUI display not available/authorized, skipping RViz2.\033[0m"
#     echo -e "\033[1;35m      (Rover nodes are running! View topics in RViz2 on your laptop via CycloneDDS).\033[0m"
# fi

echo -e "\n\033[1;32m=========================================================="
# echo " All 5 Rover Subsystems Running!"
echo " All 4 Rover Subsystems Running! (MAVROS, Blickfeld, SF45/B, LIROVO)"
echo " Press Ctrl+C at any time to instantly kill everything."
echo -e "==========================================================\033[0m\n"

# Wait indefinitely until interrupted by user (Ctrl+C)
wait
