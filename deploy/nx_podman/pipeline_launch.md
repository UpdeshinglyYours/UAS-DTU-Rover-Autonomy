# Pipeline launch: gate-3 casualty pipeline on the NX, frames to the DGX

Last updated 2026-10-17 (evening: `rover.sh teleops` now runs `~/teleops`'s own launchers
`Teleops.sh` / `Audio_Thread.sh` / `Trauma_Thread.sh`, section 2.6). Sections: 1 overview · 2 how it works · 3 launch on the NX ·
4 receive on the DGX · 5 failsafes · 6 troubleshooting · 7 files.

---

## 1. Overview

Two machines:

- **NX** (Orin NX on the rover): `rover1@192.168.0.132` (address changes with the Wi-Fi; also seen at
  192.168.1.121), ssh key `~/.ssh/rover1`. Ubuntu 24.04 / JetPack 7. Runs the sensors and the
  casualty pipeline in **rootful podman** containers.
- **DGX** (this machine, 192.168.0.210): pulls the pipeline's person crops over HTTP and runs the
  trauma (injury) inference.

```
NX  (sudo podman ...)                                                 DGX (this machine)
┌──────────────────────────────────────────────────────────┐
│ rover_humble  ROS 2 Humble (uas_rover image)             │
│   MAVROS (Cube) ── /mavros/state, set_mode, statustext ──┼──┐
│   Blickfeld lidar, SF45/B, rosbag recorder               │  │
│ fcb_jazzy     ROS 2 Jazzy                                │  │  ROS 2 topics between the
│   FCB camera driver ── /fcb_driver/image_raw/compressed ─┼──┤  containers: CycloneDDS,
│   (JPEG 1920 px ~14 Hz), AVI recorder (hardware H.264)   │  │  loopback only, domain 0
│ teleops_humble ROS 2 Humble (teleops image)              │  │
│   Audio_Thread: verbal + motor, triggered by flight mode ◄──┘
│   Trauma_Thread: person crops per segment (rover / 4 s)  │
│     HTTP :8090  /runs /status manifests crops  ◄─────────┼──── polled ──── run_trauma.py --obc
└──────────────────────────────────────────────────────────┘                 (+ vLLM Qwen engine)
```

The containers are rootful: list them with `sudo podman ps -a` (plain `podman ps` shows a separate,
empty rootless store) or `./rover.sh status`. The docker images of the old stack are kept, unused.

---

## 2. How it works

### 2.1 The containers and their images

| container | image (podman storage) | made by | runs |
|---|---|---|---|
| `rover_humble` | `uas_rover:latest` (the team's docker image, copied: `docker save \| sudo podman load`) | `rover.sh up` | idles (`sleep infinity`); MAVROS, Blickfeld, SF45/B, bag, Nav2... are started inside it with `rover.sh rec` / `rover.sh shell` |
| `fcb_jazzy` | `fcb_jazzy:latest` (built on the DGX from `~/big_vlm/fcb_podman/Containerfile.fcb`: `ros:jazzy-ros-base` + the FCB driver `fcb_orin_driver`) | `rover.sh up` | the camera driver itself (main process) |
| `teleops_humble` | `teleops:humble` (built on the NX by `~/teleops/humble.sh build` = `uas_rover:latest` + torch/whisper/ultralytics...) | `rover.sh teleops` -> `~/teleops/Teleops.sh` (or `Audio_Thread.sh` / `Trauma_Thread.sh`) -> `humble.sh run` | the casualty pipeline (`Audio_Thread.py` or `Trauma_Thread.py`) |

Shared by all three: `--network host --ipc host --privileged`, `/dev` mounted (devices renumber on
replug), the home folder mounted at its own path (code, weights, data are used in place), the same
CycloneDDS profile (`~/podman_rover/cyclonedds_local.xml`), the NVIDIA GPU through CDI
(`--device nvidia.com/gpu=all`) when the GPU is up.

Why two ROS distributions: the rover stack (MAVROS, Blickfeld, Nav2) is ROS 2 Humble (Ubuntu
22.04); the FCB driver runs on Jazzy (Ubuntu 24.04, the same OS as the NX), because only there the
NX's hardware video encoder (GStreamer `nvv4l2h264enc`) loads. Humble and Jazzy talk to each
other over DDS with the same message definitions (`fcb_orin_msgs`, `mavros_msgs`, standard msgs).

### 2.2 Boot sequence

1. The NX boots. cron (`@reboot` line in rover1's crontab, `./rover.sh autostart on`) runs
   `./rover.sh boot` (log `~/podman_rover/boot.log`).
2. `boot` waits up to 2 min for the GPU (`/dev/nvgpu/igpu0/ctrl`), makes sure podman can use it
   (CDI spec, section 5.1), then starts `rover_humble` and `fcb_jazzy`.
3. `fcb_jazzy` finds the camera by USB ID, opens the video (1920x1080 UYVY, ~60 fps), finds the
   camera's control port (VISCA) and drives the zoom to **1x** (`start_zoom`, the camera otherwise
   keeps its last zoom across power cycles). It publishes the JPEG stream and its status, and
   waits for MAVROS (RC switches, telemetry).
4. Manual from here: the clock after a cold boot, `./rover.sh rec` (MAVROS / lidar / bag) and
   `./rover.sh teleops` (the pipeline).

### 2.3 The camera: two modes (`./rover.sh teleops --camera ros|direct`)

- **ros** (default): `fcb_jazzy` holds the camera and publishes `/fcb_driver/image_raw/compressed`
  (`sensor_msgs/CompressedImage`, JPEG, full 1920 px, ~13.5 Hz measured; set by `FCB_JPEG_WIDTH` /
  `FCB_JPEG_HZ` when fcb_jazzy starts). The pipeline subscribes and decodes it (`--source ros`).
  The camera keeps working for everything else at the same time: the bag records the stream, the
  AVI recorder (RC ch8 or `./rover.sh keys`) records full quality, zoom / RGB-IR work by RC or service.
  The programs call this `--fcbros` (frames from `fcb_orin_ros`, the ROS driver).
- **direct**: the pipeline opens the camera itself (`--source fcb`) with a non-ROS driver package:
  `--driver fcbbbb` (default; `~/fcbbbb/fcb_base_driver`, mounted read-only into the container;
  `non-ros` is accepted as the same name) or `--driver manipulator`
  (`~/teleops/fcb_manipulator/fcb_base_driver`; runs without the launchers' checks, which only
  know fcbbbb). Both have the same capture code. `fcb_jazzy` must be stopped; no video in the
  bag; no zoom control.
- **file**: `--file data/<clip>` (inside `~/teleops`) instead of a camera, for bench tests.

### 2.4 The casualty pipeline (`Audio_Thread.py`, in `teleops_humble`)

Started once; loads every model once (Whisper, SepFormer, Qwen3.5-0.8B triage, YOLO11m-pose,
YOLOE, Sapiens2), opens the camera and subscribes to `/mavros/state`. Then it waits for the pilot:

```
pilot stops at a casualty, switches to GUIDED
 └ Audio_Thread (only a CHANGE into GUIDED triggers; the first /mavros/state only records the mode)
     in parallel: 10 s of camera feed kept in memory (5 fps)
     1. verbal: speaker "Describe your injuries..." -> 10 s on the Pico 8-mic array
        -> WPE -> GSC -> SepFormer -> MVDR -> Whisper -> Qwen triage
        -> NORMAL / ABNORMAL / ABSENT + injured regions; a 2nd attempt if not NORMAL
     2. motor: posture from YOLO11m-pose (2 s); standing -> NORMAL; lying / sitting ->
        "raise one arm" prompt, watch 5.3 s (2 tries) -> NORMAL / movement check -> ABNORMAL / ABSENT
     3. "Recording completed." -> decision.json (~/teleops/data/triage_runs/<casualty>/)
     4. trauma still needed (verbal not NORMAL, or --trauma = always)?  -> Trauma_Thread.begin(): crops per segment,
        /mavros/set_mode MANUAL, pilot told (GCS text + /teleops/operator) "ORBIT, then GUIDED"
pilot orbits in MANUAL, then switches to GUIDED again
 └ Trauma_Thread.finish(): last segment published, run closed -> DGX gives the verdict
```

**Audio bypass** (`--bypass-audio`, section 3): step 1 is skipped. No Pico is needed and the
verbal models are not loaded; the verbal result is recorded as `BYPASSED`, which never counts as
"trauma done", so every GUIDED goes on to the motor task (unless `--no-motor`) and then straight
to the trauma orbit (MANUAL, crops, GUIDED closes it) exactly as above.

**Orbit segments: who cuts them.** The run (one casualty) starts and ends on the flight mode;
inside it, segments are cut by the **rover** (`SEGMENTS = "rover"`, the default):

- the rover sends `region start` / `region stop` events, by HTTP `POST /trigger`
  `{"kind": "region", "state": "start"}` to `http://127.0.0.1:8090` (vortex_mission's
  `casualty_models_bridge`: a new segment every 90 deg swept around the casualty from the rover's
  pose; or `~/teleops/rover_trigger.py --server http://127.0.0.1:8090 region start` by hand) or as
  `std_msgs/String "region start"` on `/frame_extraction/trigger`;
- they count only while an orbit runs; `global start/stop` from the rover are ignored (logged);
- **failsafe**: no rover event for 12 s (`ROVER_SEGMENT_TIMEOUT`) -> a **4 s timer**
  (`ORBIT_SEGMENT_SEC = 4.0`) cuts new segments until the rover sends again (logged as `FAILSAFE`
  / `rover segment events are back`). So without any rover data an orbit still produces segments.

For testing without the rover: `--segments timer` (rover.sh: `--timer-segments`) = the 4 s timer
only. Settings at the top of `~/teleops/Trauma_Thread.py`: `SEGMENTS`, `ORBIT_SEGMENT_SEC`,
`ROVER_SEGMENT_TIMEOUT` (flags `--segments`, `--orbit-segment-sec`, `--rover-segment-timeout`).

Trauma side per segment: 1 frame/s kept (near-duplicates dropped) -> up to 4 candidates ->
person crop -> Sapiens2 (7 body regions) on the crop -> blur / off-body / shadow gates -> best 2
crops (max 1 per side of the casualty) published on HTTP `:8090`, all kept frames saved to the
segment's `feed/` folder.

**The NX's frame-extraction model: Sapiens2 0.8B, FP4** (2026-10-17). The fine-tuned 7-region
model (base `facebook/sapiens2-seg-0.8b`) with its 224 Linear layers stored as 4-bit E2M1 + a
bf16 scale per 16 weights (`~/teleops/sapiens_fp4.py`; 1.66 GB -> 538 MB,
`~/teleops/weights/sapiens2_regions_0.8b_fp4`). No FP4 version is published (only INT4 /
Apple-MLX builds of Meta's generic body-part model), so it was quantized from our own model.
Against bf16: 99.5% of pixels get the same label, region IoU 0.94-0.98 (56 casualty crops). The Orin
has no FP4 math: weights are expanded to bf16 per layer, so it saves memory, not time.
**The DGX keeps the full model** (`run_trauma.py`: `sapiens2_regions_all/best`).

**Person finder** (who makes the crop), `Trauma_Thread.py` `PERSON`:
- default, **Sapiens2 alone**: one Sapiens pass on the whole frame (768x576 canvas, landscape
  frames turned to portrait), its body pixels (confidence >= 0.3) joined into blobs, the
  casualty = the biggest blob nearest the centre. YOLOE is not loaded. On 105 casualty frames the
  crop held >= 90% of YOLOE's person box every time, at about the same crop size.
- **`--yoloe`**: YOLOE's person box, then Sapiens2 regions on its crop (the earlier way).
  `--yoloe --no-sapiens` = YOLOE only. Settings: `SAPIENS_PERSON_CONF / _MIN_AREA / _SCALE`,
  `SAPIENS_MODEL` (`--sapiens-model weights/sapiens2_regions_all_bf16` = the bf16 original).

**Speed (NX)**: one Sapiens pass is ~2 s on an idle Orin (1024x768), more while the gate-2
llama-server shares the GPU; the person pass costs ~half of that. Per candidate frame that is
~3 s (YOLOE way: ~2 s), so 4 candidates per 4 s segment fall behind and segments queue (seen:
9.3 s for a 2-frame segment with the GPU shared). The queue drains after the orbit; for less lag
lower `--seg-scan` (e.g. 2) or use `--yoloe`.

**`--trauma`**: every casualty gets the orbit, whatever the verbal / motor results. The decision is
still computed and saved (`decision.json` `trauma_forced: true`, log `-> ORBIT (--trauma: forced)`).
Works with `--bypass-audio` too.

### 2.5 Frames to the DGX

The NX only serves (`ThreadingHTTPServer` on `0.0.0.0:8090`): `GET /runs`, `GET /status`,
`GET /<segment>/gallery_manifest.jsonl`, `GET /<segment>/<run>/<crop>.jpg`. The DGX polls it
(`obc_receiver.py`, inside `run_trauma.py --obc`): a new run = one casualty; every finished segment
is pulled and its crops judged at once (Sapiens2 regions + Qwen3.5-2B injury model through vLLM);
when the run closes, all its frames are pooled into the final per-region verdict (`verdict.json`).

Every image the NX serves (crops, `--camfeed` frame, `--smoke` crops) is also copied, flat, to
`~/teleops/saved_stream_images/<run>_s<seg>_f<frame>.jpg` on the NX (`Trauma_Thread.py`
`SAVED_STREAM_DIR`, "" = off; given back to rover1 after each run). At start the server prints a
`DGX viewer:` line: `python3 ~/NX_rover1_pipeline/show_streamed_image.py http://<NX-IP>:8090` on
the DGX shows the crops as they arrive (no model; saves them next to the script).

### 2.6 The launch layers (what `./rover.sh teleops` runs)

```
./rover.sh teleops [--camera ros|direct] [--file clip] [--bypass-audio] [--trauma] ...   (~/podman_rover)
  podman side: GPU description (CDI), starts fcb_jazzy for --camera ros, checks its JPEG stream,
               refuses --camera direct while fcb_jazzy holds the camera
  └ ~/teleops/Teleops.sh   (both threads, the normal run)  |  Audio_Thread.sh (single-thread tests:
    -> Audio_Thread.sh         --bypass/--test/--smoke)    |  Trauma_Thread.sh (--program Trauma_Thread.py)
      run_lib.sh checks, each failure = "STOP: ... / fix: ...", exit 2, nothing started:
        camera: --fcbros needs fcb_jazzy up; fcbbbb needs it down and /dev/video* free;
                --file must be inside ~/teleops; always ONE source, --fallback-source ""
        locks /tmp/teleops_{camera,pico,port_8090}.lock: a second program is refused
        port 8090 free; GPU up (/dev/nvgpu/igpu0/ctrl; --allow-cpu overrides); rover container up (warning)
      └ ~/teleops/humble.sh run Audio_Thread.py ...   (teleops_humble: Humble, Cyclone loopback, ~/fcbbbb)
      after it ends / ssh drops (HUP) / SIGTERM: a program still running in the container is stopped
      (SIGINT, 5 s, SIGKILL), so it never keeps the camera or port; saved frames chowned to rover1
```

The launchers can be used without rover.sh (`cd ~/teleops && ./Teleops.sh --fcbros`), but then
nobody starts fcb_jazzy or checks its stream settings. `--dry-run` anywhere = checks + the command,
nothing started.

---

## 3. Launch the pipeline on the NX

```bash
ssh -i ~/.ssh/rover1 rover1@192.168.0.132
cd ~/podman_rover

# 0. after a COLD boot only: the clock resets to 1970 (no RTC battery, no NTP offline). From the DGX:
#    ssh -i ~/.ssh/rover1 rover1@<NX-IP> "sudo date -u -s '$(date -u '+%F %T')'"

# 1. containers (started at boot)
./rover.sh status                 # rover_humble + fcb_jazzy "Up"? camera line shows fps?
./rover.sh up                     # only if they are not up
#    camera "0 fps / CAMERA BOARD IS STUCK" -> switch the camera board's 12 V off and on
#    "GPU not available on this boot"     -> section 6.1

# 2. MAVROS (flight mode = the pipeline's trigger), lidar, rosbag
./rover.sh rec                    # tmux: mavros | blickfeld | sf45b | bag   (Ctrl-b d detaches)

# 3. the pipeline, camera from the ROS driver (recommended)
./rover.sh teleops --camera ros --dry-run    # checks only: what is missing? (nothing started)
./rover.sh teleops --camera ros              # = ~/teleops/Teleops.sh --fcbros
#    or the pipeline opens the camera itself (no ROS driver):
#    ./rover.sh down fcb                                   # then 12 V cycle the camera board
#    ./rover.sh teleops --camera direct                    # fcbbbb (or --driver manipulator)
```

A refusal looks like `[Audio_Thread.sh] STOP: <what> / fix: <what to do>`, e.g. "another teleops
program is using the camera (Trauma_Thread.sh pid ... since ...)": stop that one first (Ctrl-C in
its terminal, or `./rover.sh down teleops`).

| | `--camera ros` | `--camera direct` |
|---|---|---|
| frames from | fcb_jazzy's JPEG stream | the camera, in the pipeline |
| driver | `fcb_orin_driver` (ROS 2 Jazzy) = `--fcbros` | `fcbbbb` (`~/fcbbbb`, alias `non-ros`) or `manipulator` |
| camera held by | fcb_jazzy | the pipeline |
| video in the bag, AVI, RC zoom | yes | no |

**Audio bypass: trauma only, no verbal task.** Add `--bypass-audio`:

```bash
./rover.sh teleops --camera ros --bypass-audio                  # GUIDED -> motor task -> trauma orbit
./rover.sh teleops --camera ros --bypass-audio -- --no-motor    # GUIDED -> straight to the trauma orbit
```

Same mode triggers as normal (GUIDED starts, the pipeline sets MANUAL for the orbit, the next
GUIDED closes it and the DGX gives the verdict). It starts without the Pico mics and does not load
Whisper / SepFormer / Qwen-triage (more GPU memory for the trauma models). `decision.json` still
gets written, with `verbal_alertness: BYPASSED`. Works with `--camera direct` too.

**Always orbit: `--trauma`** (with or without `--bypass-audio`):

```bash
./rover.sh teleops --camera ros --trauma                   # audio + motor, then ALWAYS the orbit
./rover.sh teleops --camera ros --bypass-audio --trauma    # motor, then always the orbit
```

**Timer segments for testing** (no rover segment events): add `--timer-segments` (= the program's
`--segments timer`): a new segment every 4 s.

```bash
./rover.sh teleops --camera ros --timer-segments
./rover.sh teleops --camera ros --program Trauma_Thread.py -- --bypass --segments timer   # orbit NOW, 4 s segments
~/teleops/rover_trigger.py --server http://127.0.0.1:8090 region start   # a rover segment event by hand
```

Not the same as the bench test flag `--bypass` (below), which runs one assessment immediately
without any mode trigger and exits.

**Smoke test with the rover's flight mode** (no models, no camera, no Pico): the integration check
of the RC mode switch -> both threads -> DGX.

```bash
./rover.sh up && ./rover.sh rec          # MAVROS must run: /mavros/state is the only rover input
./rover.sh teleops -- --smoke_ros        # = ~/teleops/Teleops.sh --smoke_ros
#   flip the RC mode switch:
#   GUIDED -> Audio: "Listening..." every second
#   MANUAL -> Trauma: an orbit run opens; a segment of 2 MAGENTA crops every 5 s
#   GUIDED -> the run closes -> the DGX pulls it like a real orbit
#   Ctrl-C stops both
```

The smoke segments are cut by a **5 s timer only** (`Trauma_Thread.py` `SMOKE_SEGMENT_SEC = 5.0`,
`--smoke-segment-sec`), never by the rover's region start/stop events (`POST /trigger` is refused,
no trigger topic). Nothing commands the rover. On the DGX:
`python3 ~/NX_rover1_pipeline/show_streamed_image.py http://192.168.0.132:8090` (or the receiver).
Single sides: `--program Trauma_Thread.py -- --smoke_ros` (orbit side only),
`-- --smoke --ros` (audio side only). If nothing reacts: `[mode] nothing received yet` = MAVROS
not publishing `/mavros/state` (Cube not connected, or `./rover.sh rec` not running).

Extras:

```bash
./rover.sh teleops --program Trauma_Thread.py -- --smoke     # no camera / models: magenta crops once -> DGX link test
./rover.sh teleops --program Trauma_Thread.py -- --camfeed   # ONE camera frame, served like a crop
./rover.sh teleops --program Trauma_Thread.py --file data/test_clip.mp4 -- --bypass   # orbit on a clip
./rover.sh teleops --camera ros --program Trauma_Thread.py -- --bypass   # trauma side only, orbit NOW
./rover.sh teleops --camera ros -- --bypass --verbal-wav data/test_take.wav --no-speaker
                                  # one full assessment now (recorded mic take, no speaker)
./rover.sh teleops --camera ros --bypass-audio -- --bypass --no-speaker
                                  # one assessment now without the audio task (motor only) -> exits
./rover.sh down teleops           # stop the pipeline container
./rover.sh keys                   # camera keys: zoom, RGB/IR, r = start/stop AVI
./rover.sh rec --bag              # a new bag, sensors keep running;  ./rover.sh rec --stop
./rover.sh doctor                 # self-test: Humble<->Jazzy topics + hardware encoder
./rover.sh logs fcb               # camera driver output
./rover.sh autostart status       # boot autostart on/off + last boot log
```

Knobs (environment, read when the container starts): `FCB_START_ZOOM` (default 1.0; 0 = keep the
camera's zoom), `FCB_JPEG_WIDTH` (0 = 1920), `FCB_JPEG_HZ` (15), `FCB_ARGS` (extra driver args),
`ROS_NET=net` (DDS over the network instead of loopback, e.g. rviz on a laptop; restart all
containers), `ROS_DOMAIN_ID` (0). Launcher flags (after `--`, or anything rover.sh does not know):
`--dry-run`, `--allow-cpu`, `--port N` (Trauma_Thread.sh). The program flags: `--fcbros`,
`--source ros|fcb`, `--fcb-driver fcbbbb|manipulator`, `--image-topic` (on `Audio_Thread.py` /
`Trauma_Thread.py`; the launchers choose the source themselves and refuse these), `--bypass-audio`,
`--trauma`, `--no-motor` (on `Audio_Thread.py`; hyperparameters `AUDIO`, `FORCE_TRAUMA`, `MOTOR`
at its top).

Do not run at the same time: the docker rover stack (`run_container.sh`), the host's
`~/fcb_orin_ros/fcb_ros.sh`, or any other camera program (`~/gate_2 run_gate2.py fcb`,
`fcb_record.py`, `fcb_track.py`): only one program may hold the camera.

---

## 4. Set up the DGX to receive the frames

```bash
# 1. the injury model engine (vLLM, port 8100; ~3 min until "up after ...s"; ~24 GB unified memory)
cd ~/big_vlm
FOLDS=custom MODEL_DIR=/home/uas/big_vlm/new_gcp_weights/testing_final_weights \
    TAG=final PORT=8100 ./serve_qwen35_triage.sh
curl -s http://127.0.0.1:8100/v1/models            # lists qwen35-triage-final

# 2. the NX answers?
curl -s http://192.168.0.132:8090/runs | head -c 300    # JSON (an empty list is fine);
                                                        # "connection refused" = pipeline not running
# 3. receive + judge, live
cd ~/big_vlm/finalised_pipeline
PY=/home/uas/miniconda3/envs/ai_env/bin/python
HF_HUB_OFFLINE=1 $PY run_trauma.py --obc http://192.168.0.132:8090 --out results_live --obc-skip-existing
```

- **Always give the URL**: a bare `--obc` uses the old NX address `http://192.168.1.102:8090`.
  New NX address: `ssh ... hostname -I` or the router.
- `--obc-skip-existing`: ignore runs already on the NX at start-up (else every old run is pulled
  and judged). `--obc-once`: handle what is there and exit. `--obc-local <dir>`: where the crops
  go (default `<out>/pulled`).
- Results per casualty in `results_live/<run_id>/`: `verdict.json` (final per-region verdict +
  latency after the run closes), `video_predictions.jsonl`; annotated crops in
  `results_live/annotated/<run_id>/`.
- Other model: `MODEL_DIR=.../pipe_control_b2x4_sft_ep2_merged`. The model must match the prompt
  the pipeline sends (`--prompt v1`, the default).
- Stop: Ctrl-C, then `./serve_qwen35_triage.sh stop`.

Link check only, no model:

```bash
cd ~/big_vlm/finalised_pipeline
python3 obc_receiver.py --server http://192.168.0.132:8090 --local /tmp/pulled_obc --skip-existing
# or watch the crops arrive in a window (the "DGX viewer:" line the NX prints):
python3 ~/NX_rover1_pipeline/show_streamed_image.py http://192.168.0.132:8090
```

With `./rover.sh teleops --program Trauma_Thread.py -- --smoke` on the NX this needs no camera
and no models: one closed run of magenta crops.

Verified 2026-10-17: every run, segment, manifest and crop JPEG the NX published arrived on the DGX.

---

## 5. Failsafes (what keeps it running, and what it does when something is missing)

### 5.1 Boot and containers

| situation | what happens |
|---|---|
| NX reboots / loses power | cron `@reboot` -> `rover.sh boot` recreates `rover_humble` + `fcb_jazzy` (no login needed) |
| GPU fails to initialise on a boot (`nvgpu: Bootstrap HS ACR failed`) | containers start **without** the GPU instead of not at all (warning "GPU not available on this boot"); the camera recorder falls back to the CPU encoder; models would run on CPU. Fix: section 6.1 |
| NVIDIA's GPU description (CDI spec) unusable by podman 4.9 / missing after a boot | `rover.sh` keeps a compatible copy in `/etc/cdi/nvidia-podman.yaml` (rewritten when NVIDIA's changes) and uses it when NVIDIA's own is missing |
| the camera driver process crashes | the launch respawns it after 3 s; if the whole container dies, podman restarts it (`--restart on-failure`) |
| a container is stopped | `fcb_jazzy` gets SIGINT (`--init --stop-signal SIGINT`): the AVI and CSV are finalised before exit (tested: 1 s, "shut down cleanly") |
| DDS between containers | loopback only: a Wi-Fi drop or address change does not cut the containers off from each other |

### 5.2 Camera

| situation | what happens |
|---|---|
| two programs want the camera | `rover.sh up fcb` refuses when a host camera program runs; `rover.sh teleops --camera direct` and the launchers refuse while `fcb_jazzy` or any process holds `/dev/video*`; `--fcbros` refuses when `fcb_jazzy` is NOT up; a camera lock (`/tmp/teleops_camera.lock`) admits one pipeline program; always ONE source with `--fallback-source ""`, so the modes never fight |
| camera not plugged in / not found at start | driver keeps running, looks again (video re-open with back-off 5-60 s, control port every 10 s); video joins when the camera appears |
| board stuck (0 fps after an NX boot or a stream close; Oppila board quirk) | driver logs `CAMERA BOARD IS STUCK ... cycle its 12 V`; after the 12 V cycle it picks the camera up by itself (seen: device renumbered, found again) |
| camera's zoom left anywhere | `start_zoom` (1x) once camera control comes up; the RC zoom knob takes over as soon as it reports |
| no camera control (VISCA) while MAVROS owns the Cube | the driver only searches the camera board's own serial ports (it used to also probe the Cube's port every 10 s, which broke MAVROS) |
| hardware encoder unavailable | the AVI recorder falls back to the CPU encoder (logged) |
| disk full / write error | recording stops cleanly and new recordings are refused until fixed; disk warnings |
| pipeline: no frames | camera watchdog warns every 10 s (never silently empty); segments report "NO frames"; in ROS mode the hint says to start fcb_jazzy |
| pipeline: Ctrl-C | once: the open run is closed (DGX inference) and the crops keep being served; twice: exit. Camera sources stop quietly (no false "no frame" warnings) |

### 5.3 Pipeline and mode triggers

| situation | what happens |
|---|---|
| start-up problems seen by the launchers (camera / driver state, port 8090 in use, GPU down, a second program on the same camera / Pico / port, a `--file` outside ~/teleops, a mistyped flag) | `STOP: ... / fix: ...`, exit 2, nothing started |
| start-up problems seen by the program (no ROS, missing weights, no camera frames within 15 s, no Pico, no GPU for Sapiens2) | `SetupError`: exit code 2 with one `ERROR: ... / fix: ...` block |
| ssh drops / terminal closed / launcher killed (SIGTERM) | the launcher stops the program left in the container (SIGINT, 5 s, SIGKILL): camera, port and locks are free again (tested: SIGTERM -> "stopping it", port free) |
| MAVROS silent / Cube not connected | state watchdog warns repeatedly ("Mode changes are NOT seen"); keeps waiting |
| first `/mavros/state` after start | only records the mode (no false trigger when started in GUIDED) |
| GUIDED while an assessment runs | ignored |
| `/mavros/set_mode` not available | pilot told to set MANUAL himself (GCS text + `/teleops/operator`) |
| verbal not NORMAL | second attempt; then trauma orbit requested |
| orbit segments: rover sends no segment events (bridge not running, no pose) | after 12 s without one, a 4 s timer cuts the segments (`FAILSAFE` in the log) until rover events come back; the orbit never stalls in one huge segment |
| rover sends `global start/stop` or segment events outside an orbit | ignored and logged: the run follows the flight mode only |
| `--trauma` | the orbit follows every assessment; the real decision is still saved (`trauma_forced: true`) |
| `--bypass-audio` | no verbal task, no Pico required at start, verbal models not loaded; verbal = `BYPASSED` -> the orbit always follows |
| motor frame rate below 10 fps or no person | result `NOT_TESTABLE` with flags in `decision.json` (never a guessed NORMAL) |
| no speaker | prompts go to ALSA card 0 (silent) with a warning at start; `--no-speaker` waits the prompt time |
| nothing downloads at run time | `HF_HUB_OFFLINE`, `TRANSFORMERS_OFFLINE`, `YOLO_OFFLINE`; all weights local |

### 5.4 DGX side

| situation | what happens |
|---|---|
| NX unreachable / pipeline not running | receiver prints "unreachable" and keeps polling |
| `run_trauma.py` restarted | restart-safe: `receiver_state.json` records what was delivered; a segment interrupted mid-inference is pulled again |
| old runs on the NX | `--obc-skip-existing` ignores them |

---

## 6. Troubleshooting

### 6.1 "GPU not available on this boot"

The Orin's GPU did not initialise. Check: `ls /dev/nvgpu/igpu0` shows only `power`;
`nvidia-smi -L` says "No devices found"; `journalctl -k -b | grep nvgpu` shows
`acr-gsp.data.encrypt.bin.prod ucode get fail` ... `Bootstrap HS ACR failed`.

The firmware files are intact (2026-10-17: `/lib/firmware/nvidia/ga10b/acr-gsp*` match the
`nvidia-l4t-firmware` checksums, `dpkg --verify` clean). **Likely cause** (found by the team,
`~/teleops/ABOUT.md`): the kernel command line has `firmware_class.path=/etc/firmware`, which does
not exist, so nvgpu sometimes cannot load `nvidia/ga10b/acr-gsp.*` ("failed with error -40");
some boots get lucky. Permanent fix (not applied yet, changes the system):
`sudo mkdir -p /etc/firmware && sudo cp -r /usr/lib/firmware/nvidia /etc/firmware/`, reboot, check
`journalctl -k -b | grep -c "ACR bootstrap failed"` = 0 (NVIDIA-AI-IOT/jetson-ai-lab issue #427).
Until then:

1. Power the NX fully off and on (cold start), or `sudo systemctl reboot`.
2. Check `ls /dev/nvgpu/igpu0` lists `as`, `channel`, `ctrl`, ... and `nvidia-smi -L` shows the GPU.
3. If the containers came up without the GPU: `./rover.sh down` (answer y for the camera) and
   `./rover.sh up` recreates them with it; then 12 V cycle the camera board.
4. If it keeps happening: the supply. The NX hard-resets often (8+ times on 2026-10-16/17, no
   shutdown record in `last -x`): power input, connectors, battery sag.

### 6.2 Other known issues

- Every NX reset or camera stream close needs a **12 V cycle of the camera board**.
- Motor task speed: 10.6 fps on the live ROS stream (minimum 10, tuned at 15): tight, worse while
  the gate-2 llama-server shares the GPU. A TensorRT pose engine is not built yet.
- The Cube sends only heartbeat-level MAVLink streams: `/mavros/rc/in`, IMU, GPS stay empty until
  its SR*_ stream rates are set (the RC zoom knob and record switch need `/mavros/rc/in`).
- Not started at boot: `./rover.sh rec`, `./rover.sh teleops`, the clock after a cold boot.
- Humble nodes sometimes print `invalid data size` / `string data is not null-terminated, at
  ./src/serdata.cpp:384`: a ROS message they cannot decode (likely one from the Jazzy side).
  Harmless so far (seen in the lidar window and a test node); which topic is not yet identified.

---

## 7. Files

| where | what |
|---|---|
| NX `~/podman_rover/` | `rover.sh` (control script), `cyclonedds_local.xml`, `nx.md`, `README.md`, `boot.log`, this file |
| NX `~/teleops/` | the pipeline (`Audio_Thread.py`, `Trauma_Thread.py`, `frame_sources.py`, `checks.py`, `segmenter.py`, `sapiens_fp4.py`, `ABOUT.md`; models in `weights/`: `sapiens2_regions_0.8b_fp4` (default), `sapiens2_regions_all_bf16`, `yoloe-11l-seg-person.pt` (`--yoloe`)), its launchers (`Teleops.sh`, `Audio_Thread.sh`, `Trauma_Thread.sh`, `run_lib.sh`, `humble.sh`), `saved_stream_images/` |
| NX `~/fcbbbb/` | the non-ROS FCB driver bundle (its `fcb_base_driver` = `--driver fcbbbb`) |
| NX `~/fcb_orin_ros/src/` | the ROS FCB driver source (baked into the `fcb_jazzy` image) |
| DGX `~/big_vlm/fcb_podman/` | `Containerfile.fcb` (fcb_jazzy image), driver source copy, `build_proxy.py` (temporary internet for builds on the NX), `nx_weights/sapiens2_regions_0.8b_fp4` (the NX's FP4 model; not used on the DGX) |
| DGX `~/big_vlm/finalised_pipeline/` | `run_trauma.py`, `obc_receiver.py` (DGX side) |
| DGX `~/NX_rover1_pipeline/` | a copy of `~/teleops` + `show_streamed_image.py` (the crop viewer) |
