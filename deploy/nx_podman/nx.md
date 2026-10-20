# Orin NX (`uasdtu`): containers, how to run, FCB drivers

Rover1 @ 192.168.0.132 (seen before at 192.168.1.121; the Wi-Fi address changes
with the network). SSH key: `~/.ssh/rover1`. User `rover1`, home `/home/rover1`
(renamed from `/home/shadow2` on 2026-10-16; `/home/shadow2` is a symlink to it).
Host: Ubuntu 24.04, JetPack R39, ROS 2 Jazzy installed natively. No internet on
the Orin: images are built on the Spark (aarch64) and copied over.

---------------------------------------------------------------------------

## 1. What exists

```
Orin NX
 ├── Rootful podman (sudo podman, /var/lib/containers)  ── the stack in use
 │    ├── rover_humble   image uas_rover:latest   ROS 2 Humble, Ubuntu 22.04
 │    └── fcb_jazzy      image fcb_jazzy:latest   ROS 2 Jazzy,  Ubuntu 24.04
 │         both: host network, host IPC, --privileged, NVIDIA GPU (CDI),
 │               CycloneDDS on loopback only, domain 0, the host's clock
 │
 ├── Docker  ── old stack, kept until podman testing is finished (do not delete)
 │    ├── images: uas_rover:latest (16.2 GB), uas_rover:pre-fcb (16 GB), ubuntu:latest
 │    └── containers: unruffled_jones (ubuntu, exited). The rover container
 │        (uas_rover_container) is --rm: it exists only while run_container.sh runs.
 │
 ├── Rootless podman (podman without sudo): only hello-world, unused
 │
 └── Host-native
      ├── ROS 2 Jazzy, FastDDS only (CycloneDDS debs ready in ~/podman_rover/debs)
      ├── ~/fcb_orin_ros       original ROS FCB driver workspace + fcb_ros.sh
      ├── DL / pipelines: ~/gate_2, ~/triage_system, ~/frame_extraction(_final),
      │   ~/HR_lidar_RGB, ~/gate_1_pipeline, ~/llama.cpp, ~/gate1_weights, ~/gate2_models
      └── venvs: ~/gate2_env, ~/frame_extraction/.venv, ~/gate2_audio_env
```

### rover_humble (podman, Humble)
- Image `uas_rover:latest`: the docker image copied into podman unchanged
  (built from `rover_ws/src/UAS-DTU-navigation-simulation/Dockerfile`).
  CUDA 12.6.3 + cuDNN 9 on Ubuntu 22.04; ROS Humble ros-base, Nav2 + STVL,
  RViz2, pcl_ros, robot_localization, cv_bridge, TF2, pointcloud_to_laserscan,
  CycloneDDS; MAVROS deps + GeographicLib data; Ceres/SuiteSparse/TBB; Blickfeld
  Scanner Library 2.20.6; rosbag2 MCAP; tmux, pymavlink.
- Mounts:

  | host | container | why |
  |---|---|---|
  | `~/rover_ws` | `/workspace/rover_ws` | colcon workspace (auto-sourced by .bashrc) |
  | `/home/rover1` | `/home/rover1` | whole home at the same path: DL models, venvs, data |
  | `/dev` | `/dev` | Cube (ttyACM*), SF45/B (ttyUSB*), GPU nodes |
  | `/tmp/.X11-unix` | same | rviz2 / GUIs on the Orin's display |
  | `~/podman_rover/cyclonedds_local.xml` | `/etc/cyclonedds_local.xml` | shared DDS profile |
- Packages in `rover_ws/src/UAS-DTU-navigation-simulation`: Blickfeld, mavros +
  mavlink, lightwarelidar (SF45/B), navigation2 + nav2_custom_bt_nodes, lirovo,
  genz-icp, ground_segmentation, patchwork++, pointcloud_to_laserscan,
  jsk_recognition_msgs, vortex_interfaces, vortex_mission, fcb_camera (only its
  `fcb_orin_msgs` matters now: lets Humble record the camera's Status messages).
- Process: idles on `sleep infinity`; work goes in through `podman exec`
  (`rover.sh shell`, `rover.sh rec`).

### fcb_jazzy (podman, Jazzy)
- Image `fcb_jazzy:latest` (~1.9 GB), built on the Spark from
  `~/big_vlm/fcb_podman/Containerfile.fcb` (copy in `~/podman_rover/build/`):
  `ros:jazzy-ros-base` + rmw_cyclonedds_cpp, mavros_msgs, rosbag2 MCAP, Ubuntu's
  OpenCV 4.6 (with GStreamer), GStreamer base/good/bad, v4l-utils, pyserial,
  pymavlink. `fcb_orin_driver` + `fcb_orin_msgs` are built into `/opt/fcb_ws`
  (a driver change = rebuild the image).
- Entrypoint `fcb_entrypoint.sh`: points libv4l2 at NVIDIA's, sources ROS, drops
  to rover1 (uid 1000, with its groups) so AVIs/CSVs/logs are not root-owned.
- Mounts: `/dev` + `/run/udev` (camera video + VISCA port, hot-plug),
  `/home/rover1` (recordings to `~/flight_recordings`, logs to `~/.ros`), the
  host's `libgstnvvideo4linux2.so` + `libgstnvvidconv.so` (Jetson hardware H.264
  encoder), NVIDIA CDI device, the DDS profile.
- Process: `ros2 launch fcb_orin_driver fcb_driver.launch.py fc_source:=mavros
  publish_compressed:=true record_dir:=~/flight_recordings`; the launch respawns
  the driver; the container has `--restart on-failure`, `--init`,
  `--stop-signal SIGINT` (clean AVI finalisation on stop).

### How they talk
- Jazzy -> Humble: `/fcb_driver/image_raw/compressed`, `/fcb_driver/status`, ...
  Humble -> Jazzy: `/mavros/rc/in`, `/mavros/state`, telemetry (driver reads RC
  ch7 zoom, ch8 record, ch9 snapshot, ch10 RGB/IR).
- Same RMW (CycloneDDS), same loopback profile, same domain: tested both ways.
- One bag, recorded in Humble, everything on one clock.

### Support files
- `~/podman_rover/`: `rover.sh` (control script), `cyclonedds_local.xml`,
  `host_env.sh`, `debs/` (CycloneDDS RMW for host Jazzy), `build/`, `README.md`,
  `nx.md` (this file), `test_bags/`.
- `/etc/cdi/nvidia-podman.yaml`: GPU spec podman 4.9 can parse (rover.sh rewrites
  it whenever NVIDIA's `/var/run/cdi/nvidia.yaml` is newer).
- `/etc/sudoers.d/podman`: rover1 may run podman, systemctl, date without a password.
- On the Spark: `~/big_vlm/fcb_podman/` (Containerfile, entrypoint, driver
  source copy, orin/ scripts, debs).

---------------------------------------------------------------------------

## 2. How and where to run things

Everything runs on the Orin from `~/podman_rover`.

```bash
# rover_humble + fcb_jazzy start by themselves at boot (./rover.sh autostart on|off|status;
# a @reboot line in rover1's crontab runs ./rover.sh boot, log ~/podman_rover/boot.log).
# If the GPU fails to come up after a power cut they start WITHOUT it: reboot to get it back.
# after a cold boot: set the clock (the RTC resets to 1970; no NTP offline)
ssh rover1@<orin> "sudo date -u -s '$(date -u '+%F %T')'"     # from a laptop/Spark

cd ~/podman_rover
./rover.sh up             # rover_humble + fcb_jazzy (fixes the GPU spec if needed)
./rover.sh doctor         # self-test: Humble<->Jazzy topics + hardware encoder
./rover.sh rec            # tmux: mavros | blickfeld | sf45b | bag -> ~/rover_ws/bags
                          #   Ctrl-b d detaches; ./rover.sh rec --bag = new bag
                          #   ./rover.sh rec --stop = end the session cleanly
./rover.sh keys           # camera: zoom, RGB/IR, r = start/stop AVI -> ~/flight_recordings
./rover.sh status         # containers, camera status line, ROS nodes
./rover.sh logs fcb       # driver output (logs rover = Humble container)
./rover.sh shell          # Humble shell: ros2 CLI, colcon, Nav2/lirovo, rviz2
./rover.sh shell fcb      # Jazzy shell in the camera container
./rover.sh down           # stop (asks before stopping the camera driver)
```

| what | where | how |
|---|---|---|
| MAVROS, Blickfeld, SF45/B, rosbag | rover_humble | `./rover.sh rec` (= `record_sensors.sh --no-camera`) |
| Nav2, lirovo, genz-icp, start_all.sh | rover_humble | `./rover.sh shell`, then as in docker (`/workspace/rover_ws`) |
| build rover_ws | rover_humble | `./rover.sh shell` -> `colcon-release` |
| FCB driver + AVI recorder | fcb_jazzy | starts with `./rover.sh up`; leave it running between flights |
| camera control | fcb_jazzy | `./rover.sh keys`, RC ch7-10, or `ros2 service call /fcb_driver/...` from either shell |
| DL / Python pipelines | host, own folders + venvs | unchanged; for ROS use: once `sudo dpkg -i ~/podman_rover/debs/*.deb`, then `source ~/podman_rover/host_env.sh` |
| change the camera driver | Spark `~/big_vlm/fcb_podman` | edit `src/`, `docker build -f Containerfile.fcb -t fcb_jazzy:latest .`, ship (README) |

Laptop rviz / off-board topics: `ROS_NET=net ./rover.sh up` (restart both
containers so they match; host shells: `ROS_NET=net source host_env.sh`).

Do NOT run at the same time as the podman stack:
- docker `run_container.sh` / `start_all.sh` in docker (duplicate MAVROS/Blickfeld)
- host `~/fcb_orin_ros/fcb_ros.sh` or any non-ROS FCB program (camera conflict)

Do not stop `fcb_jazzy` between flights: on the Oppila board, closing the video
stream usually needs a 12 V power cycle before the camera sends video again.

### Gate-3 casualty pipeline (~/teleops) on podman — camera switch

Runs in its own Humble container `teleops_humble` (image `teleops:humble` = uas_rover:latest +
torch/whisper/ultralytics..., built in podman), next to rover_humble and fcb_jazzy, same DDS profile.
`./rover.sh teleops` runs `~/teleops`'s own launchers (since 2026-10-17 evening): `Teleops.sh`
(Audio_Thread + Trauma_Thread, the normal run), `Audio_Thread.sh` (single-thread tests) or
`Trauma_Thread.sh` (`--program Trauma_Thread.py`). They check camera / driver, Pico, port 8090,
GPU and locks first (`STOP: ... / fix: ...`, nothing started) and stop the program if the ssh
session drops. Details: `~/teleops/ABOUT.md` sections 2 / 2b, `pipeline_launch.md` section 2.6.

```bash
cd ~/podman_rover
./rover.sh up                                   # rover_humble + fcb_jazzy (JPEG stream 1920 px @ 15 Hz)
./rover.sh rec                                  # MAVROS (mode triggers), Blickfeld, bag
./rover.sh teleops --camera ros                 # = ~/teleops/Teleops.sh --fcbros (reads /fcb_driver/image_raw/compressed)
# or, non-ROS camera (fcb_jazzy must NOT run; Oppila then needs a 12 V cycle):
./rover.sh down fcb
./rover.sh teleops --camera direct              # fcbbbb (alias non-ros); --driver manipulator skips the checks
# options: --bypass-audio, --trauma (always orbit), --timer-segments, --file data/<clip>,
#          --program Trauma_Thread.py ; launcher/program args after --, e.g. -- --bypass, -- --dry-run
./rover.sh down teleops
```

| mode | frames | camera held by | bag has video |
|---|---|---|---|
| `--camera ros` | ROS topic, JPEG decoded in the pipeline | fcb_jazzy (AVI, RC zoom/record keep working) | yes |
| `--camera direct` | `~/fcbbbb/fcb_base_driver` (or fcb_manipulator) in-process, no ROS | the pipeline | no |

Streamed crops are also saved on the NX in `~/teleops/saved_stream_images/`; DGX viewer:
`python3 ~/NX_rover1_pipeline/show_streamed_image.py http://<NX-IP>:8090`.

Tested live 2026-10-17: both modes on the real FCB (direct: non-ros + manipulator; ros: JPEG 13.5 Hz),
motor 10.6 fps on the ROS stream, crops pulled by the DGX. DGX side: `run_trauma.py --obc http://192.168.0.132:8090`
(give the URL: `--obc` alone defaults to the old 192.168.1.102).

---------------------------------------------------------------------------

## 3. FCB drivers on the Orin

Only ONE program may hold the camera (video + VISCA port) at a time.

ROS:
| where | what |
|---|---|
| `fcb_jazzy` container | the one in use: `fcb_orin_driver` (from `~/fcb_orin_ros/src`) |
| `~/fcb_orin_ros` | same driver, host-native Jazzy, run by `fcb_ros.sh` (tmux) |
| `~/gate_1_pipeline/GATE_1_PIPELINE/fcb_ros` | older `fcb_camera` driver + `fcb_commands` (fcb_cheatsheet.md) |
| `~/fcb_rec_tiaga/ros2_ws` | another `fcb_camera` + `fcb_record_session.sh` |
| `rover_ws/.../fcb_camera` | copy inside the Humble workspace; only its msgs are used now |

Non-ROS (open the camera directly, conflict with fcb_jazzy):
| where | what |
|---|---|
| `~/fcbbbb` | the non-ROS recorder bundle (`fcb_record.py`, `fly.sh`, `fcb_base_driver/`): the pipeline's `--camera direct` uses its `fcb_base_driver` (the default, `fcbbbb`) |
| `~/fcb_orin` | original standalone recorder: `fcb_record.py` (camera + VISCA + AVI/CSV + RC via pymavlink), `fcb_view.py`, `fly.sh`, GCS scripts, `fcb_base_driver/` |
| `~/fcb_feed` | `camfeed.py` + its own copy of fcb_orin (MP4s in `~/camfeed`) |
| `~/frame_extraction(_final)` | `start_camera.sh` / `frame_sources.py` can read the camera itself |
| `~/gate_2` | `run_gate2.py fcb` opens the camera directly |

To run a DL pipeline alongside the podman driver it must take frames from
`/fcb_driver/image_raw/compressed` (960 px JPEG, 10 Hz) instead of the device.

---------------------------------------------------------------------------

## 4. Fixes made during the migration (2026-10-16)

| problem | fix | backup |
|---|---|---|
| camera in Humble docker starved (0.3-3.5 fps): no hardware encoder on 22.04 | driver moved to fcb_jazzy (24.04 + host NVENC) | docker image untouched |
| home renamed shadow2 -> rover1 broke venvs/workspaces | paths rewritten, `fcb_orin_ros` + `sf45b_ws` rebuilt, symlink | `~/shadow2_rename_backup_*.tgz`, `*.shadow2_bak` dirs |
| podman 4.9 rejects NVIDIA CTK 1.19's CDI spec (`additionalGids`) | rover.sh writes `/etc/cdi/nvidia-podman.yaml` | - |
| `/var/run/cdi` missing after some boots | rover.sh starts `nvidia-cdi-refresh` | - |
| MAVROS never connected (no `/mavros` topic in any bag): vendored mavros 2.14 checks the serial baud against the 65535 port limit | `libmavconn/src/interface.cpp` per-caller limit, libmavconn rebuilt | `interface.cpp.before_baud_fix` |
| FCB driver with no camera probed the Cube's serial ports every 10 s -> MAVROS heartbeat timeouts | `recorder.py`: under `fc_source:=mavros` only the camera board's ports are probed | `recorder.py.before_mavros_probe_fix` |
| fcb_jazzy ignored podman stop (SIGKILL after timeout, AVI cut) | `--init --stop-signal SIGINT` | - |
| `record_sensors.sh` started its own camera | `--no-camera` flag; FCU path resolved to /dev/ttyACMx | `record_sensors.sh.before_podman` |
| Orin clock at 1970 after cold boot | `sudo date -s` (manual each cold boot) | - |
| containers not running after a reboot | `./rover.sh autostart on`: cron @reboot -> `rover.sh boot` (GPU-aware) | `autostart off` |
| GPU sometimes fails at boot after a power cut (`nvgpu: Bootstrap HS ACR failed`, nvidia-ctk writes an empty spec) | containers start without the GPU; saved `/etc/cdi/nvidia-podman.yaml` used; reboot to recover | - |

Note: the host's `fcb_ros.sh` runs the host's apt-installed Jazzy MAVROS, which
the libmavconn patch above does not touch. Whether that build has the same baud
check is unverified.

---------------------------------------------------------------------------

## 5. Test status

| test | result |
|---|---|
| Humble <-> Jazzy topics (both directions) | OK |
| hardware H.264 encoder in fcb_jazzy as rover1 | OK |
| Blickfeld -> bag in podman | OK, 9.8 Hz (when the lidar was connected) |
| one bag: lidar (Humble) + 150 kB images at 10 Hz (Jazzy) | OK: 9.75 Hz + 10.00 Hz, 0 drops, check_bag_sync OK |
| FCB driver without camera | runs as rover1 under podman-init, waits for the camera |
| MAVROS -> Cube in rover_humble | connects after the baud fix; stays connected with the driver running (probe fix) |
| stopping fcb_jazzy | 1 s, driver logs "shut down cleanly" (was SIGKILL after 60 s) |
| full `rover.sh rec` bag (rover-20261016-231414, 94 s) | Blickfeld 9.6 Hz, /mavros/state + battery ~1 Hz (first bag ever with MAVROS), /fcb_driver/status from Jazzy |
| real camera, end to end (2026-10-16 23:25) | OK: 60 fps capture in fcb_jazzy; AVI via ~/start_recording from Humble, nvenc, 1080p 59.5 fps, 96.8 MB/31 s, owned by rover1; bag rover-20261016-232548 (40 s): camera ~9 Hz, lidar 9.75 Hz, mavros state/battery, check_bag_sync OK, camera-lidar median 26 ms |
| zoom service Humble -> Jazzy driver | OK (set_zoom 5.0) |
| camera after an Orin reset | board stuck (0 fps) until its 12 V is cycled; driver then picks it up by itself (hot-plug OK) |

Known gaps:
- `/mavros/rc/in`, imu, GPS, vfr_hud: 0 messages. The Cube only sends heartbeat-level
  streams until asked: set its SR0_*/SR1_* stream-rate parameters (or call
  `/mavros/set_stream_rate`). The camera driver needs `/mavros/rc/in` for RC ch7-10.
- The Orin keeps losing power: hard resets at 18:27, 18:33, 20:57, 22:11, 23:01, 23:09
  on 2026-10-16, no shutdown records, kernel log simply stops (zero-filled) -- a supply
  cut (battery sag, connector, current spike), not a software crash. Bags open at the
  moment of a cut lose their metadata.yaml (`ros2 bag reindex` recovers them).

Open items: install host CycloneDDS debs; at boot only the containers start (`autostart`):
set the clock after a cold boot, start `./rover.sh rec` / `teleops` by hand; retire docker only after testing.
