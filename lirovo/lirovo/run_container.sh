#!/usr/bin/env bash
# ==============================================================================
# UAS Rover Docker Launch Script
# Compatible with both x86_64 Laptop & Jetson Orin NX (aarch64)
# Enables GUI (X11), GPU (NVIDIA), Host Networking (ROS 2 DDS), and Device Access
# ==============================================================================

# Enable local X11 forwarding for RViz2 and GUI tools
xhost +local:root >/dev/null 2>&1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Auto-detect workspace root (walk up from lirovo/lirovo to workspace root)
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../../" 2>/dev/null && pwd)"

# Fallback check: if not detected, check common locations
if [ ! -d "${WORKSPACE_DIR}/src" ]; then
    if [ -d "/home/vortex/DARPA_ws" ]; then
        WORKSPACE_DIR="/home/vortex/DARPA_ws"
    elif [ -d "/home/vortex/uas_nav" ]; then
        WORKSPACE_DIR="/home/vortex/uas_nav"
    else
        WORKSPACE_DIR="$(pwd)"
    fi
fi

WORKSPACE_NAME="$(basename "${WORKSPACE_DIR}")"

# Auto-detect GPU runtime:
# Jetson Orin NX natively uses '--runtime nvidia', while desktops support both '--runtime nvidia' and '--gpus all'
GPU_ARGS="--gpus all"
if docker info 2>/dev/null | grep -i "Runtimes" | grep -q "nvidia"; then
    GPU_ARGS="--runtime nvidia"
fi

echo "=========================================================="
echo " Starting UAS Rover Docker Container"
echo " Workspace: ${WORKSPACE_DIR} -> /workspace/${WORKSPACE_NAME}"
echo " GPU Runtime: ${GPU_ARGS}"
echo " Network: Host mode (DDS / CycloneDDS active)"
echo "=========================================================="

docker run -it --rm \
    --name uas_rover_container \
    --net=host \
    --ipc=host \
    --privileged \
    ${GPU_ARGS} \
    -e NVIDIA_DISABLE_REQUIRE=1 \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "${WORKSPACE_DIR}:/workspace/${WORKSPACE_NAME}" \
    uas_rover:latest \
    /bin/bash
