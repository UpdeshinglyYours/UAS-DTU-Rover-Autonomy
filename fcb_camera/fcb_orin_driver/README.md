# fcb_orin_driver

ROS 2 driver and flight recorder for the Sony FCB-EV9520L on its USB board
(Twiga USB3 NeoHD `04b4:00f9`, Oppila LVDS-USB3 `04b4:0040`), ported from
`~/fcb_orin/fcb_record.py`. It has the same features and the same
fallbacks. The terminal pane is replaced by ROS topics and services, and
`fcb_keys` puts the old keys back in a terminal.

`fcb_base_driver/` is the fcb_orin driver package, copied in **unchanged**
(VISCA, link padding, device discovery, frame grabber, zoom servo, RC
switches, pymavlink source). `recorder.py` is `fcb_record.py`'s `Recorder`
with the terminal parts taken out.

## Humble build (uas_rover container)

This copy is the ROS 2 Humble build of `~/fcb_orin_ros` (Jazzy, on the
host), for the `uas_rover` container, so the camera, both LiDARs and MAVROS
run in one ROS graph and go into one bag. Same package names, topics,
services and messages as the Jazzy one. Changes for Humble / Python 3.10:

- `Recorder.submit` turns `concurrent.futures.TimeoutError` into the builtin
  `TimeoutError` (they are only the same class from 3.11), so a busy
  capture loop answers "busy" again.
- Snapshots are handed to the message as `array('B')`; Humble's generated
  setter walks a `bytes` object one int at a time.
- No Jetson GStreamer encoder in the image: `record_sensors.sh` passes
  `encoder:=cpu` for the AVI. The bag carries the JPEG stream regardless.

Only one program may hold the camera: stop the host's `fcb_ros.sh` before
starting this one.

## Quick start in the container

    # on the rover, once per image change
    cd ~/rover_ws/src/UAS-DTU-navigation-simulation && docker build -t uas_rover:latest .
    lirovo/lirovo/run_container.sh

    # inside the container
    cd /workspace/rover_ws && colcon build --symlink-install \
        --packages-select fcb_orin_msgs fcb_orin_driver
    source install/setup.bash
    src/UAS-DTU-navigation-simulation/lirovo/lirovo/record_sensors.sh
    # tmux: mavros | blickfeld | sf45b | camera | control | bag

Bags go to `/workspace/rover_ws/bags` (`~/rover_ws/bags` on the host), the
driver's AVI + CSV to `~/rover_ws/flight_recordings`. Check a bag's clocks:

    python3 src/UAS-DTU-navigation-simulation/lirovo/lirovo/check_bag_sync.py <bag>

`record_sensors.sh --help` lists the options (`--no-bag`, `--all-topics`,
`--no-mavros`, `--no-sf45b`, `--no-blickfeld`, `--fcu`, `--sysid`,
`--sf45b-port`, `--bf-host`, `--bag`, `--status`, `--stop`).

**Do not stop the driver between flights.** On the Oppila board, closing
the video stream usually leaves the board sending nothing until its 12 V is
power-cycled. Start a new bag with `record_sensors.sh --bag`, and start and
stop AVI recordings with RC ch8 or `r` / `r r` in the control window.
Detach with `Ctrl-b d`.

## What is kept from fcb_record.py

| Feature / fallback | Where |
|---|---|
| Camera found by USB ID / card name, never a laptop webcam by mistake; blind scan only without `v4l2-ctl` | `devices.autodetect_video` |
| YUYV vs UYVY decoded from the negotiated format; raw 2-channel frames converted | `open_camera`, `to_bgr` |
| Oppila: repeated frames dropped, misreported 30 fps ignored, nominal fps used | `FrameGrabber(drop_repeats)` |
| No frames: reopen with back-off 5/10/20/40/60 s; never exits | `capture_loop` |
| `CAMERA BOARD IS STUCK` / `board_power_cycle_cmd` at most every 90 s | `board_is_stuck` |
| Blank (all-green) picture detection | `check_blank` |
| VISCA port found by protocol, all three bauds, board's own ports first; retried every 10 s if absent at launch | `make_visca_link` |
| Oppila VISCA padding/resync and reply validation | `visca_link` (unchanged) |
| Servo rebuilds a dead link; zoom by camera-side Direct on padded links | `zoom_servo` (unchanged) |
| Image stabilizer re-applied and read back on every link | `apply_stabilizer` |
| Imaging mode left as found unless `icr` is set | `start_icr` |
| Detented zoom (0.5x), `ratio`/`log`/`raw` curves, exact zoom by hand overrides the knob | `set_zoom`, `zoom_to_rc` |
| RC ch7 zoom, ch8 record (0.3 s debounce, stale RC holds), ch9 snapshot, ch10 RGB / RGB+IR / IR | `_attach_source` |
| No FC: insistent warnings, never a silent "no"; late autopilot picked up | `wants_recording`, `_find_mavlink_later` |
| Started by hand: not stopped by ch8 sitting low; stop suppresses restart while ch8 is high | `apply_record_switch` |
| NVENC via GStreamer, loud CPU fallback, whole-number fps | `VideoRecorder` |
| AVI segments in one session with one CSV; gap rows during outages; ≤3 filled frames | `open_segment`, `write_frame` |
| Write failure stops cleanly and blocks new recordings; disk warnings | `on_write_failure`, `check_disk` |
| Empty-video detection | `close_segment` |
| Rename every file of a session or none; never overwrites | `rename_recording` |
| Daily log file in `record_dir`; state file for GCS panes | `setup_logging`, `publish_state` |
| SIGHUP ignored; SIGINT/SIGTERM finish the recording before exit | `node.main` |
| pip OpenCV without GStreamer bypassed for the system one | `prefer_cv2` |

What is new:

- `fc_source:=mavros` reads RC and telemetry from MAVROS, which owns the
  autopilot's port on a ROS drone. `pymavlink` (the default, as before)
  would fight MAVROS for the port. With MAVROS, `fc_time_boot_ms` is empty
  in the CSV and attitude comes from `imu/data`, converted back to NED.
- Services run on the capture thread, through a queue, so the recorder
  keeps its single-threaded state handling.
- `fcb_keys` is a separate process. Closing it, or losing SSH, can never
  stop a recording.

## Interface

Topics (`/fcb_driver/...`): `image_raw` (bgr8, only when subscribed),
`camera_info`, `image_raw/compressed` (with `publish_compressed`, a 960 px
JPEG stream at 10 Hz for bags), `snapshot/compressed`, `status`
(`fcb_orin_msgs/Status`), `status_text`.

Services:

    ros2 service call /fcb_driver/set_zoom fcb_orin_msgs/srv/SetZoom '{ratio: 7.3}'
    ros2 service call /fcb_driver/zoom_to_rc std_srvs/srv/Trigger
    ros2 service call /fcb_driver/set_imaging_mode fcb_orin_msgs/srv/SetImagingMode '{mode: ir}'   # rgb | ir | rgb+ir | auto
    ros2 service call /fcb_driver/cycle_imaging_mode std_srvs/srv/Trigger
    ros2 service call /fcb_driver/snapshot std_srvs/srv/Trigger
    ros2 service call /fcb_driver/start_recording std_srvs/srv/Trigger
    ros2 service call /fcb_driver/stop_recording fcb_orin_msgs/srv/StopRecording
    ros2 service call /fcb_driver/rename_recording fcb_orin_msgs/srv/RenameRecording '{base: "", name: "pass 3"}'

Parameters: every `fcb_record.py` option, with dashes as underscores and
the same defaults (`config/fcb_driver.yaml` lists the common ones). Empty
string or 0 means "autodetect" where the option had no default. Pass an
empty string on the command line as `-p "video:=''"`.

## Rosbag

`record_sensors.sh` records MCAP into `/workspace/rover_ws/bags`: the JPEG
stream, the driver's status and snapshots, the Blickfeld cloud and IMU, the
SF45/B cloud and scan, `/tf`, `/tf_static` and the MAVROS telemetry. Every
one is stamped on this machine's clock (camera: capture time; LiDARs:
receipt, `use_lidar_timestamp: false`). Raw 1080p60 (about 370 MB/s) is left out on purpose:
the full-quality video is the driver's own AVI next to it. `record_sensors.sh
--bag` starts a fresh bag; Ctrl-C in the bag window stops only the bag.

## Tests

    cd src/fcb_orin_driver && python3 -m pytest -q test

These are fcb_orin's regression tests, ported, plus tests for the
service queue, the MAVROS source and the key reader.
