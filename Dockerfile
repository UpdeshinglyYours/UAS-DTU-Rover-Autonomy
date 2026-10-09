# =============================================================================
# UAS-DTU Rover Autonomy Development Container
#
# Workspaces:       uas_nav + Mavros_ws
# Host:             Ubuntu 24.04 / JetPack 7 / Ubuntu 22.04 LTS
# Container:        Ubuntu 22.04 / ROS 2 Humble / CUDA 12.6
# =============================================================================

# -----------------------------------------------------------------------------
# 1. Base Image
# -----------------------------------------------------------------------------
ARG BASE_IMAGE=nvcr.io/nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04
FROM ${BASE_IMAGE}

# -----------------------------------------------------------------------------
# 2. Environment Configuration
# -----------------------------------------------------------------------------
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Hardware acceleration: graphics for RViz2 OpenGL, compute for CUDA
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Force Cyclone DDS (Mandatory for Blickfeld LiDAR driver stability in Humble)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS environment variables
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

SHELL ["/bin/bash", "-c"]

# -----------------------------------------------------------------------------
# 3. System Packages & Build Tools
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    tzdata \
    git \
    wget \
    unzip \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    libeigen3-dev \
    libyaml-cpp-dev \
    bash-completion \
    nano \
    vim \
    gedit \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 4. ROS 2 Humble Repository
# -----------------------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list

# -----------------------------------------------------------------------------
# 5. Full ROS Dependencies (uas_nav, Mavros_ws, Blickfeld, Nav2, SLAM)
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ros2launch \
    # Navigation2 & Costmaps
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-laser-geometry \
    # RViz2 & Plugins
    ros-humble-rviz2 \
    ros-humble-nav2-rviz-plugins \
    ros-humble-interactive-markers \
    ros-humble-visualization-msgs \
    # DDS Middleware
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    # Vision & Transforms (Blickfeld & VINS)
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-eigen \
    # Robot description & localization
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    # MAVROS build dependencies (for your custom Mavros_ws)
    ros-humble-mavros-msgs \
    ros-humble-angles \
    ros-humble-eigen-stl-containers \
    ros-humble-trajectory-msgs \
    ros-humble-geographic-msgs \
    libgeographic-dev \
    geographiclib-tools \
    libasio-dev \
    libconsole-bridge-dev \
    python3-empy \
    python3-zmq \
    ros-humble-robot-localization \
    ros-humble-test-msgs \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-ompl \
    libgraphicsmagick++1-dev \
    libnanoflann-dev \
    libomp-dev \
    # Ceres & SLAM Toolbox / VINS math dependencies
    libceres-dev \
    libsuitesparse-dev \
    liblapack-dev \
    libtbb-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5widgets5 \
    # Blickfeld protobuf dependencies
    libprotobuf-dev \
    libprotobuf23 \
    protobuf-compiler \
    # Colcon tools & Python libraries
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-click \
    python3-numpy \
    python3-yaml \
    # LightWare SF45/B LiDAR & serial tools
    python3-serial \
    setserial \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 6. GeographicLib Datasets (Required by MAVROS for GPS/geoid conversions)
# -----------------------------------------------------------------------------
RUN geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015

# -----------------------------------------------------------------------------
# 7. Blickfeld Scanner Library (BSL v2.20.6)
# -----------------------------------------------------------------------------
# Ubuntu 18/20 deb (relies on libprotobuf17):
# RUN wget -q https://github.com/Blickfeld/blickfeld-scanner-lib/releases/download/v2.20.6/blickfeld-scanner-lib-dev-Linux.deb -O /tmp/blickfeld.deb \
#     && dpkg -i /tmp/blickfeld.deb \
#     && rm /tmp/blickfeld.deb
# Ubuntu 22.04 deb (matches libprotobuf23):
# RUN wget -q https://github.com/Blickfeld/blickfeld-scanner-lib/releases/download/v2.20.6/blickfeld-scanner-lib-dev-testing-Linux.deb -O /tmp/blickfeld.deb \
#     && dpkg -i /tmp/blickfeld.deb \
#     && rm /tmp/blickfeld.deb
# Multi-architecture support: amd64 deb on laptop/PC, source build on Jetson Orin NX (arm64):
RUN ARCH=$(dpkg --print-architecture) \
    && if [ "$ARCH" = "amd64" ]; then \
        wget -q https://github.com/Blickfeld/blickfeld-scanner-lib/releases/download/v2.20.6/blickfeld-scanner-lib-dev-testing-Linux.deb -O /tmp/blickfeld.deb \
        && dpkg -i /tmp/blickfeld.deb \
        && rm /tmp/blickfeld.deb; \
    else \
        echo "Detected architecture $ARCH (e.g. Jetson Orin NX). Building Blickfeld C++ library from source..." \
        && git clone --depth 1 --branch v2.20.6 https://github.com/Blickfeld/blickfeld-scanner-lib.git /tmp/bsl \
        && cd /tmp/bsl \
        && git submodule update --init thirdparty/asio \
        && mkdir -p build && cd build \
        && cmake .. -DCMAKE_BUILD_TYPE=Release -DBF_BUILD_EXAMPLES=OFF -DBF_BUILD_TESTS=OFF \
        && make -j$(nproc) && make install \
        && ldconfig \
        && rm -rf /tmp/bsl; \
    fi

# -----------------------------------------------------------------------------
# 8. rosdep Init & Auto-Sourcing Environment
# -----------------------------------------------------------------------------
RUN rosdep init 2>/dev/null || true \
    && rosdep update --include-eol-distros

# Auto-source ROS 2 and Workspaces dynamically when opening bash shells
# (Preserved previous hardcoded workspace sourcing as comments per policy):
# echo "if [ -f /workspace/Mavros_ws/install/setup.bash ]; then source /workspace/Mavros_ws/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/Blickfeld_ws/install/setup.bash ]; then source /workspace/Blickfeld_ws/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/lightwarelidar/install/setup.bash ]; then source /workspace/lightwarelidar/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/uas_nav/install/setup.bash ]; then source /workspace/uas_nav/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/DARPA_ws/install/setup.bash ]; then source /workspace/DARPA_ws/install/setup.bash; fi" >> /root/.bashrc
# echo "if [ -f /workspace/rover_docker/install/setup.bash ]; then source /workspace/rover_docker/install/setup.bash; fi" >> /root/.bashrc
RUN echo "" >> /root/.bashrc \
    && echo "# Auto-source ROS 2 and Workspaces dynamically" >> /root/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /root/.bashrc \
    && echo 'for ws in /workspace/*/install/setup.bash; do if [ -f "$ws" ]; then source "$ws"; fi; done' >> /root/.bashrc \
    && echo 'mkdir -p /home/vortex 2>/dev/null || true' >> /root/.bashrc \
    && echo 'for d in /workspace/*; do if [ -d "$d" ]; then b=$(basename "$d"); [ ! -e "/home/vortex/$b" ] && ln -s "$d" "/home/vortex/$b" 2>/dev/null || true; fi; done' >> /root/.bashrc \
    && echo "export PS1='\[\e[1;31m\]\u@uas-rover\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> /root/.bashrc \
    && echo "alias colcon-release='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> /root/.bashrc

# -----------------------------------------------------------------------------
# 9. Path Compatibility & Workspace Directory
# -----------------------------------------------------------------------------
# Ensures any paths resolve seamlessly inside container
# (Preserved previous hardcoded workspace symlinks as comments per policy):
# ln -s /workspace/DARPA_ws /home/vortex/DARPA_ws
# ln -s /workspace/uas_nav /home/vortex/uas_nav
# ln -s /workspace/rover_docker /home/vortex/rover_docker
RUN mkdir -p /home/vortex

WORKDIR /workspace
CMD ["/bin/bash"]
