# Rover on podman (rootful)

Two containers on the host network, one CycloneDDS profile, one clock:

| container      | image                    | ROS    | runs                                                    |
|----------------|--------------------------|--------|---------------------------------------------------------|
| `rover_humble` | `uas_rover:latest`       | Humble | MAVROS, Blickfeld, SF45/B, Nav2, lirovo ... and the bag  |
| `fcb_jazzy`    | `localhost/fcb_jazzy`    | Jazzy  | FCB camera driver, hardware H.264 (NVENC) AVI recorder   |

The docker image and container are untouched; do not run the docker stack and
these at the same time (two MAVROS / two Blickfeld drivers = duplicated topics).

## At boot

`./rover.sh autostart on` (done 2026-10-17): rover1's crontab runs `./rover.sh boot` at every boot,
which starts rover_humble + fcb_jazzy (log `boot.log`). Without a working GPU on that boot they
start without it. `rec` and `teleops` stay manual. `./rover.sh autostart off` undoes it.

## Daily use

    cd ~/podman_rover
    ./rover.sh up              # both containers (CDI spec fixed up if needed)
    ./rover.sh rec             # tmux: mavros | blickfeld | sf45b | bag  (Humble records)
    ./rover.sh keys            # camera: zoom, RGB/IR, r = start/stop AVI
    ./rover.sh rec --bag       # new bag, driver keeps running
    ./rover.sh status
    ./rover.sh shell [rover|fcb]
    ./rover.sh logs fcb
    ./rover.sh doctor          # Humble<->Jazzy DDS + NVENC self-test
    ./rover.sh down            # asks before stopping the camera driver
    ./rover.sh teleops [--camera ros|direct] [--bypass-audio] [--trauma] [-- --dry-run]
                               # gate-3 pipeline via ~/teleops/Teleops.sh (checks + clean-up);
                               # see pipeline_launch.md

`rec` = `record_sensors.sh --no-camera` inside `rover_humble`: the camera window
is skipped because the driver lives in `fcb_jazzy`; its topics are still bagged.
Bags: `~/rover_ws/bags/rover-*`. AVIs: `~/flight_recordings` (owned by rover1).

## Why it is built like this

- **FCB in Jazzy / 24.04**: the Humble (22.04) image cannot load the Jetson's
  GStreamer encoder, so the AVI recorder ran on the CPU and the bagged camera
  stream fell to 0.3-3.5 fps. In `fcb_jazzy` the host's `nvv4l2h264enc` loads.
- **Loopback-only DDS** (`cyclonedds_local.xml`, both containers): Cyclone
  otherwise binds to whichever NIC it picks at start; a Wi-Fi drop then cuts
  the nodes off from each other. For rviz from a laptop: `ROS_NET=net ./rover.sh up`
  (restart both containers so they match).
- **`/etc/cdi/nvidia-podman.yaml`**: NVIDIA Container Toolkit 1.19 writes a CDI
  0.7 spec that podman 4.9 rejects (`additionalGids`); rover.sh writes a
  compatible copy whenever the NVIDIA one is newer.
- **Rootful**: `/etc/sudoers.d/podman` lets rover1 run podman, systemctl and date
  without a password.

## Rebuilding fcb_jazzy (the Orin is offline)

On an aarch64 machine with internet (the Spark: `~/big_vlm/fcb_podman`):

    docker build -f Containerfile.fcb -t fcb_jazzy:latest .
    docker save fcb_jazzy:latest | gzip -1 | ssh rover1@<orin> 'sudo podman load'
    ssh rover1@<orin> 'sudo podman tag docker.io/library/fcb_jazzy:latest localhost/fcb_jazzy:latest'

The driver source is baked in (`/opt/fcb_ws`); copy `~/fcb_orin_ros/src` into
the build context first when the driver changes.

## Clock

The Orin's RTC does not keep time and it has no NTP offline: after a cold boot
it is 1970 and bags/AVIs get 1970 names. Set it before recording, e.g. from a
laptop:  `ssh rover1@<orin> "sudo date -u -s '$(date -u '+%F %T')'"`
