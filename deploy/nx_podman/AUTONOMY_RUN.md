# Running the autonomy stack on the Orin NX

Written 2026-10-18. Everything runs inside the `rover_humble` container (image `uas_rover:latest`,
which has the tf2 0.25.24 fix). One script, `lirovo/lirovo/start_hardware_stack2.sh`, opens a tmux
window with every piece. Nav2 (`stack2/lirovo4.launch.py`) waits by itself until GenZ publishes
odometry.

## Quick start

After the hardware steps in section 0, on the rover:

```bash
cd ~/podman_rover
./start_autonomy.sh            # start the whole stack (fresh) and attach to tmux
./start_autonomy.sh --viewer   # same, plus the costmap viewer: http://192.168.0.132:8095/
./start_autonomy.sh --detach   # start without attaching
```

`start_autonomy.sh` checks that `rover_humble` is running and the Cube is plugged in (it refuses to
start without the Cube), warns if the lidar link is down, and then runs the commands of section 3
for you. Do not run `start_hardware_stack2.sh` by itself: it must run inside `rover_humble`, and its
defaults (`ODOM_SOURCE=mavros`, a ttyACM probe without the Cube) do not fit this rover.
Overrides: `ODOM_SOURCE=mavros ./start_autonomy.sh`, `GCS_URL=udp://@<gcs-ip>:14555 ./start_autonomy.sh`.

The sections below are the manual steps the script runs.

## 0. Hardware, before you log in

1. Power the rover. The **Cube** must be plugged into the Orin by USB.
2. Power the **Blickfeld lidar (12 V)** and plug its Ethernet into the Orin's RJ45 port
   (`enP8p1s0`). After power-on the link takes about **90 s** to come up. If the driver starts
   before that, it times out.
3. The SF45/B is not connected at the moment. Its pane will show an error; ignore it.

## 1. Log in and check the containers

```bash
ssh rover1@192.168.0.132
cd ~/podman_rover
./rover.sh status
```

If `rover_humble` is not listed (e.g. after a reboot): `./rover.sh up rover`.
The camera (`./rover.sh up fcb`) and teleops are separate and **not** needed for autonomy.

## 2. Quick pre-checks

```bash
ls -l /dev/serial/by-id/ | grep CubePilot      # Cube present (note the ttyACM number)
cat /sys/class/net/enP8p1s0/carrier            # 1 = lidar link up (0 = wait / check cable and 12 V)
```

## 3. Start the stack

Paste these **one line at a time** (long lines break when pasted into the terminal):

```bash
S=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo
CUBE=$(readlink -f /dev/serial/by-id/usb-CubePilot_*-if00); echo $CUBE
DDS=file://$S/stack2/cyclonedds_wifi2.xml
sudo podman exec -it -e FCU_URL=$CUBE:115200 -e ODOM_SOURCE=genz -e CYCLONEDDS_URI=$DDS -e DISPLAY=:1001 rover_humble bash $S/lirovo/start_hardware_stack2.sh
```

- `FCU_URL`: the Cube's port for MAVROS. The by-id name means the script never probes other ttys
  (the FCB camera board hangs if its serial port is probed).
- `ODOM_SOURCE=genz`: GenZ-ICP lidar odometry gives `map -> base_link`.
- `CYCLONEDDS_URI`: the DDS config for Wi-Fi + the lidar Ethernet. Every ROS tool you use against
  the stack needs this same value, or it sees no topics.

The script opens tmux session `rover_stack2` and attaches you to it:

| Pane | Runs | Healthy when |
|---|---|---|
| 0 | MAVROS | `CON: Got HEARTBEAT, connected` ("No GPS fix" indoors is normal) |
| 1 | Blickfeld driver | no "Connection timeout" |
| 2 | SF45/B | error (not connected, ignore) |
| 3 | spare / costmap viewer | - |
| 4 | **lirovo4**: static TFs, patchwork, GenZ, `odom_to_tf`, Nav2 | `/genz/odometry is up: starting Nav2 (full)`, then `Managed nodes are active` |
| 5 | rover_console (keyboard E-STOP / HOLD) | `mode=... armed=...` |

tmux:
- **Alt + arrow keys** switch panes (or click with the mouse).
- **Ctrl-b, then d** detaches; the stack keeps running.
- To reattach: `sudo podman exec -it rover_humble tmux attach -t rover_stack2`

If the lidar was not up at start, pane 4 waits ("Nav2 waits for the first /genz/odometry
message"). Once the link is up, restart only pane 1 (click it, Up, Enter) and Nav2 starts by
itself.

lirovo4 options (pane 4): `nav2_nodes:=costmaps` runs only the controller + planner servers (the
costmaps; nothing can drive), `nav2_delay:=<s>` changes the wait after the first odometry message.

## 4. Costmap viewer (optional, ~35% CPU)

In pane 3:

```bash
DISPLAY=:1001 XAUTHORITY=/home/rover1/.Xauthority python3 /home/rover1/podman_rover/costmap_view.py --http 8095
```

Then open **http://192.168.0.132:8095/** in a browser on the laptop. Global costmap = grey/blue,
local = yellow/red, plus footprint, heading and the Nav2 plan. `+`/`-` zoom in the NX window.

Do not run RViz on the NX: it takes a full CPU core and that starves GenZ.

## 5. Check that it is healthy

In a second SSH terminal:

```bash
S=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo
sudo podman exec -it -e CYCLONEDDS_URI=file://$S/stack2/cyclonedds_wifi2.xml rover_humble bash
source /opt/ros/humble/setup.bash
ros2 topic hz /genz/odometry        # ~10 Hz (also /bf_lidar/point_cloud_out, /patchwork/nonground)
ros2 topic hz /mavros/imu/data      # ~250 Hz
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState    # label='active'
```

CPU / memory on the host: `top` or `tegrastats`. The stack alone uses about 2.8 of the 8 cores
(GenZ ~90%, MAVROS ~70%, Blickfeld ~50%, Nav2 ~50%) and about 0.75 GB of RAM.

## 6. Stop it

- Ctrl-C in each pane, then `sudo podman exec rover_humble tmux kill-session -t rover_stack2`.
  (Running the start command again also kills the old session first.)
- **Disarm the Cube** from the RC or a GCS. Stopping the software does not disarm it.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Blickfeld "Connection timeout" | Lidar link not up yet: check `carrier` = 1, then restart pane 1. |
| Nav2 never starts | No `/genz/odometry`: lidar not publishing, or no `/mavros/imu/data` (MAVROS down). |
| Costmaps empty, `buffer updated in N s` warnings | tf2 deadlock (see below). In the container, `dpkg -l ros-humble-tf2-ros` must be **0.25.24**. |
| GenZ roll/pitch far from the Cube's | GenZ diverged (more likely under heavy CPU load). Restart pane 4 (Ctrl-C, Up, Enter). No builds, image loads or `git add` of big folders while it runs. |
| MAVROS crashed (exit -11) | Restart pane 0 (Up, Enter). |

Known open problem: **GenZ starts a few degrees tilted and can drift further.** Compare its
attitude with the Cube's before trusting a run.

## Background: the costmap freeze (fixed 2026-10-18)

With tf2 0.25.23 both costmaps stopped receiving lidar clouds within 1-3 minutes, with no error.
A stack trace (eu-stack) showed an ABBA deadlock in `tf2_ros::Buffer` between
`waitForTransform` (the costmap's cloud thread) and `BufferCore::testTransformableRequests` (the
TF listener thread). It is fixed upstream in ros2/geometry2 9997e96955, released in tf2 0.25.24.
The Dockerfile (section 7c) now upgrades tf2 and fails the build if it is older than 0.25.24.
After the upgrade, a 20-minute soak test showed no freeze. Rollback image: `uas_rover:pre-tf2`
(podman and docker).
