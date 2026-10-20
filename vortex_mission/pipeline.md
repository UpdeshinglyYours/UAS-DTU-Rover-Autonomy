# Casualty pipeline: autonomy stack ↔ software-team models

How the rover's Nav2 autonomy stack and the software team's models (frame extractor, audio
triage) work together on the rover's single computer, a **Jetson Orin NX 16 GB**.

Per casualty:

> rover **approaches** and **faces** the casualty → **frame extractor + audio model start** →
> audio plays the triage question, records 10 s, transcribes and triages → **audio done →
> rover starts its orbit** → frame extractor keeps capturing the whole orbit, one segment per
> 90° → orbit done → extractor run closed → **next casualty**.

Both directions are covered:

* **autonomy → models**: "arrived at casualty N", "orbit started", "next quarter", "left the casualty";
* **models → autonomy**: "audio finished → start the orbit", plus the triage result.

---

## 1. Where everything runs

```
┌──────────────────────────── Jetson Orin NX (one OBC) ─────────────────────────────────┐
│                                                                                       │
│  HOST  (Ubuntu 24.04 · JetPack 7 · ROS 2 Jazzy)                                       │
│  ┌───────────────────────────┐  ┌──────────────────────────┐  ┌──────────────────────┐ │
│  │ FCB camera driver         │  │ frame_extractor.py       │  │ audio_service.py NEW │ │
│  │ fcb_orin_driver           │─►│ TRIGGER=http   :8090     │  │ :8091                │ │
│  │ /fcb_driver/image_raw     │  │ YOLOE + Sapiens2         │  │  aplay question.wav  │ │
│  └───────────────────────────┘  │ serves galleries ──► Spark│  │  audioPipe.py ──► Pico│ │
│                                 └────────────▲─────────────┘  └──────────▲───────────┘ │
│                                              │ HTTP (localhost)          │ HTTP        │
│  ────────────────────────────────────────────┼───────────────────────────┼─────────────│
│  DOCKER  uas_rover  (Ubuntu 22.04 · ROS 2 Humble · CycloneDDS · --net=host)            │
│                                              │                           │             │
│  ┌──────────────────────────┐  /rover_state ┌┴───────────────────────────┴──────────┐  │
│  │ casualty_manager_simple  │──────────────►│ casualty_models_bridge.py   NEW       │  │
│  │ (lirovo5 mission)        │◄──────────────│ /casualty_manager/start_orbit(id)     │  │
│  └────────────┬─────────────┘  start_orbit  │ /casualty_models/result  (+ CSV)      │  │
│               │ NavigateToPose / Spin /     └───────────────────────────────────────┘  │
│               │ NavigateThroughPoses                                                   │
│  ┌────────────▼─────────────────────────────────────────────────────────────────────┐  │
│  │ Nav2 (nav2_params5) · MAVROS · Blickfeld · SF45/B · patchwork++ · odom_to_tf    │  │
│  └──────────────────────────────────────┬───────────────────────────────────────────┘  │
└─────────────────────────────────────────┼──────────────────────────────────────────────┘
                                          ▼ /cmd_vel → new_omega_fixed5.py → MAVROS → Pixhawk
```

### Why HTTP between the container and the host
* The container runs with `--net=host` (see `lirovo/lirovo/run_container.sh` on the `docker`
  branch), so the container and the host share `127.0.0.1`.
* The two sides run different ROS 2 versions with different DDS vendors (Humble + CycloneDDS
  inside, Jazzy + its default DDS outside). ROS topics across that boundary would need custom
  messages built twice and cross-vendor DDS discovery, which is fragile on a field robot.
* The frame extractor **already** speaks HTTP (`POST /trigger`); its `rover_trigger.py` was
  written as the reference client for exactly this link.
* The models stay **ROS-free**, and the software team's folders are used **unchanged**.

ROS is used only *inside* the container, where everything is Humble.

---

## 2. Files

### New files (autonomy side, `~/vortex_ws/vortex_mission/`)

| File | Runs | Role |
|---|---|---|
| `vortex_mission/casualty_models_bridge.py` | container (ROS node) | The two-way bridge. Watches `/rover_state`, drives the extractor and the audio service over HTTP, calls `start_orbit` when audio is done, publishes results |
| `vortex_mission/orbit_segments.py` | imported by the bridge | `SweepTracker`: bearing swept around the casualty → "another 90° done". Pure Python, unit-tested |
| `test/test_orbit_segments.py` | pytest | 4 tests: clockwise / counter-clockwise, ±180° wrap, pose-jump rejection |
| `host/audio_service.py` | Orin host (no ROS, stdlib only) | HTTP wrapper around the audio team's pipeline: plays the question wav, runs `audioPipe.py`, serves the result |
| `host/start_models.sh` | Orin host | tmux session `models`: camera driver, frame extractor (`TRIGGER=http`), audio service |
| `pipeline.md` | this file | |

### New / changed files from this project (autonomy stack)

| File | Change |
|---|---|
| `lirovo/config/nav2_params5.yaml` | **new**: copy of `nav2_params4` without the `OrbitCasualty` behavior plugin, so the Docker image doesn't need `vortex_nav_plugins` (otherwise `behavior_server` fails and Nav2 never starts) |
| `lirovo/launch/lirovo5.launch.py` | now loads `nav2_params5.yaml` |
| `vortex_mission/launch/mission_simple.launch.py` | starts `casualty_models_bridge`; `models_bridge:=false` turns it off (RC / console triggers only) |
| `vortex_mission/config/casualty_params_simple.yaml` | new `casualty_models_bridge:` section (URLs, timeouts, segment size) |
| `vortex_mission/setup.py` | console script `casualty_models_bridge` |

### Existing files used unchanged

| Software team | What we use |
|---|---|
| `~/frame_extraction_final/frame_extractor.py` | HTTP API on :8090: `POST /trigger {"kind","state"}`, `GET /status` (gives `run_id`) |
| `~/frame_extraction_final/launch.sh`, `start_camera.sh` | started by `start_models.sh` with `TRIGGER=http` |
| `~/pipeline_v1_standalone/audioPipe.py` | run as `audioPipe.py --sec 10 --out <dir>`; we read `<dir>/triage.json` and `<dir>/transcript.txt` |

| Autonomy | What we use |
|---|---|
| `casualty_manager_simple.py` | publishes `/rover_state`; waits in STANDBY for `/casualty_manager/start_orbit(id)` |
| `rc_orbit_trigger.py`, `rover_console.py` | still work as a **manual override** for `start_orbit` and as the E-STOP |
| `keepout_monitor.py` | unchanged safety watchdog |

---

## 3. One casualty, step by step

```
time ──►
casualty_manager   TRAVEL ── approach + face ──► STANDBY(id) ··········· ORBIT(id) ············► TRAVEL (next)
                                                    │                    ▲  │                         │
bridge                                              │ arrived            │  │ orbit started           │ left casualty
                                                    ▼                    │  ▼                         ▼
frame extractor    ·········· global start ─ region s01 (parked view) ─┤region s02 ─ s03 ─ s04 ─ s05 ┤ global stop
                                                                       │   90°   180°  270°           │
audio service                POST /run ─► play question ─► record 10 s ─► Whisper + Qwen ─► done      │
                                                                       │                              │
bridge ─────────────── poll GET /status 1 Hz ─────────── done ──► region stop ──► start_orbit(id) ────┘
```

1. **Arrive.** `casualty_manager_simple` drives to the nearest waypoint (`NavigateToPose`), turns
   to face the casualty (`Spin`) and publishes **`/rover_state = STANDBY, casualty_id`**. The
   rover is now still (no Nav2 goals), so the audio recording isn't drowned by motor noise.
2. **Models start** (`standby_sequence` in the bridge):
   * extractor: `global start` (one run per casualty) + `region start` → segment **s01**, the
     parked front view. The bridge reads `GET /status` and records **casualty id ↔ extractor
     run id**; the Spark only ever sees run ids.
   * audio: `POST :8091/run {"casualty_id": N}`. The audio service plays the question wav
     (`aplay`), waits 0.5 s, then runs `audioPipe.py --sec 10 --out ~/casualty_audio_runs/cas<N>_<time>`.
     That records 10 s from the Pico array and runs WPE → GSC → SepFormer → MVDR → Whisper → Qwen.
   * the bridge polls `GET :8091/status` once a second.
3. **Audio done → orbit** (models → autonomy):
   * extractor `region stop`: s01 is handed to YOLOE/Sapiens2 for crop selection, in the
     background on the Orin;
   * the bridge publishes the triage on `/casualty_models/result` and adds a CSV row;
   * the bridge calls **`/casualty_manager/start_orbit(N)`**: the same call the RC switch makes.
4. **Orbit.** `casualty_manager_simple` sends the waypoint loop (`NavigateThroughPoses`, 2 × 180°)
   and publishes `/rover_state = ORBIT`. The bridge:
   * opens `region start` (s02);
   * reads the rover pose from TF 5 times a second;
   * `SweepTracker` adds up the bearing swept around the casualty;
   * at every 90° it sends `region stop` + `region start` (s03, s04, s05).
5. **Leave.** When `/rover_state` goes back to TRAVEL/IDLE (orbit done, skipped or aborted), the
   bridge sends `region stop` + **`global stop`**. The extractor finishes its last selections in
   the background while the rover drives on. It never waits for the models.

Result: **one extractor run per casualty with 5 segments** (parked view + 4 orbit quarters).
At K = 2 crops per segment that is up to 10 crops for the Spark's injury model, plus the audio
triage on the rover.

---

## 4. Interfaces

### ROS (inside the container)

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/rover_state` | `vortex_interfaces/RoverState`, latched | manager → bridge | `TRAVEL=0 STANDBY=1 ORBIT=2 IDLE=3`, `casualty_id`, `casualty_pose` |
| `/casualties` | `vortex_interfaces/CasualtyArray`, latched | manager → bridge | used for lat/lon in the results |
| `/casualty_manager/start_orbit` | `vortex_interfaces/StartOrbit` service | bridge → manager | rejected unless the rover is in STANDBY at that casualty |
| `/casualty_models/result` | `std_msgs/String` (JSON), latched, last 50 | bridge → anyone | one message per casualty, see below |
| TF `map → base_link` | | → bridge | orbit progress |

`/casualty_models/result` example (from the sim test):
```json
{"casualty_id": 1, "audio_status": "done",
 "triage": {"verbal_alertness": "NORMAL", "injury_regions": ["LegLeft"], "injury_types": ["Closed"]},
 "transcript": "my left leg hurts, I can't move it",
 "out_dir": ".../casualty_audio_runs/cas1_20261014-125943", "question_played": true,
 "audio_seconds": 7.0, "lat": 28.75, "lon": 77.11, "extractor_run": "run-125943",
 "stamp": "2026-10-14 12:59:50"}
```
`audio_status` is one of `done | failed | timeout | cancelled | disabled`.

### HTTP: frame extractor (software team, unchanged), `http://127.0.0.1:8090`

| Request | Meaning |
|---|---|
| `POST /trigger {"kind":"global","state":"start"}` | open a run (one casualty) |
| `POST /trigger {"kind":"region","state":"start"}` | open a segment; frames are buffered 1/s, near-duplicates dropped |
| `POST /trigger {"kind":"region","state":"stop"}` | close the segment → YOLOE (+ Sapiens2) pick K crops → published |
| `POST /trigger {"kind":"global","state":"stop"}` | close the run after its segments are published |
| `GET /status` | `run_id`, open region, queue, camera fps |
| `GET /runs` | what the Spark polls |

It's lenient: `region start` with no run opens one; `region start` while a region is open
closes it first. That is why a manual RC trigger during the audio step is still handled
correctly.

### HTTP: audio service (new), `http://127.0.0.1:8091`

| Request | Answer |
|---|---|
| `POST /run {"casualty_id": N}` | `200 {"ok":true,"out_dir":...}` or `409` if a job is already running |
| `GET /status` | `{"state": idle\|playing\|recording\|done\|failed\|cancelled, "casualty_id", "triage", "transcript", "out_dir", "question_played", "error", ...}` |
| `POST /cancel` | kills the player / `audioPipe.py` (whole process group) |

`recording` covers both the 10 s recording and the processing, because `audioPipe.py` does both.

---

## 5. The new files in depth

### 5.1 `casualty_models_bridge.py`
A ROS 2 node (`MultiThreadedExecutor`, 4 threads) with these parts:

**State machine on `/rover_state`** (`on_state`):

| Message | Bridge `phase` before | Action |
|---|---|---|
| STANDBY(id) | none / other casualty | start `standby_sequence(id)` in its own thread |
| ORBIT(id) | standby / none | `orbit_started`: stop audio if still running, `region start`, new `SweepTracker` |
| TRAVEL / IDLE | standby / orbit | `close_casualty`: `region stop`, `global stop` |

**`standby_sequence`** (one thread per casualty):
* extractor `global start` + `region start`;
* `run_audio`, which polls until done, failed or timed out;
* keeps the parked segment open at least `standby_min_sec`;
* `region stop`, then publish the result, then `request_orbit`.

If the rover leaves STANDBY during the audio (operator skip, RC trigger, abort), the audio job
is cancelled and no `start_orbit` is sent.

**Extractor worker**: every extractor trigger goes into one queue and is sent **in order** by a
single thread, so HTTP never blocks a ROS callback and `stop`/`start` can't swap places. After
`global start` it reads `GET /status` to learn the run id.

**Orbit tracking** (`track_orbit`, 5 Hz timer): TF pose → `SweepTracker.update()` → on each
90° boundary, `region stop` + `region start`.

**Results**: `/casualty_models/result` (latched JSON) plus `~/casualty_logs/models_<time>.csv`
with these columns: time, casualty id, lat, lon, extractor run, audio status, verbal alertness,
injury regions, injury types, transcript, audio folder.

**Parameters** (`config/casualty_params_simple.yaml`, section `casualty_models_bridge`):

| Parameter | Default | Meaning |
|---|---|---|
| `extractor_url` | `http://127.0.0.1:8090` | frame extractor |
| `audio_url` | `http://127.0.0.1:8091` | audio service |
| `enable_extractor`, `enable_audio` | `true` | turn either model off |
| `audio_timeout` | 120 s | then the audio is cancelled and the orbit starts |
| `standby_min_sec` | 3 s | minimum length of the parked extractor segment |
| `orbit_if_audio_failed` | `true` | audio failed/down → orbit anyway; `false` → wait for RC / console |
| `orbit_segment_deg` | 90 | orbit segment size → 4 orbit segments |
| `http_timeout` | 2 s | per HTTP call |
| `log_dir` | `~/casualty_logs` | **in Docker set this inside the mounted workspace** |

### 5.2 `orbit_segments.py`: `SweepTracker`
* Each update computes the rover's bearing around the casualty, `atan2(y − cy, x − cx)`.
* It adds the absolute change since the last update. Steps over 30° are ignored as pose jumps.
* It returns `True` each time the total passes another `segment_deg`.
* It works in either direction and across ±180°.
* A full circle gives 3 boundaries (90°, 180°, 270°) → 4 orbit segments; the 4th is closed by
  `global stop`.

### 5.3 `host/audio_service.py`
* A plain `ThreadingHTTPServer` on `127.0.0.1:8091`, in the same style as the frame extractor's
  server.
* It runs one job at a time, in a worker thread:
  1. `aplay [-D device] question.wav`. A missing wav or a failed player is logged, and it records anyway.
  2. Pause `--pause-after-question` (0.5 s).
  3. `<python> audioPipe.py --sec 10 --out <runs_dir>/cas<N>_<time>`, run with `cwd` = the
     pipeline folder; output goes to `<out>/audioPipe.log`.
  4. Read `triage.json` and `transcript.txt` → `done`. A non-zero exit or no `triage.json` →
     `failed`, with the last log lines as `error`.
* **Why a subprocess:** the audio team's code is used exactly as they ship it. Its models
  (SepFormer, Whisper, Qwen 0.8B) are loaded only while a job runs, and their GPU memory is
  freed afterwards, which leaves room for the extractor on the shared 16 GB.
* **Cancel** kills the whole process group (player or `audioPipe.py` and its children).
* **Options:**
  * `--pipeline-dir`, `--python` (defaults to the pipeline's `.venv` if present);
  * `--question-wav`, `--aplay-device`, `--sec`, `--ignore` (mic indices);
  * `--audio-file` (bench test with a recorded 8-channel wav, no Pico);
  * `--runs-dir`, `--port`.

### 5.4 `host/start_models.sh`
* Creates tmux session `models` with three panes:
  1. `start_camera.sh` (skip with `CAMERA=0` if `fcb_ros.sh` already owns the camera; only one
     program may open it);
  2. the frame extractor (`TRIGGER=http SAPIENS=1 ./launch.sh`);
  3. `audio_service.py`.
* Settings come from environment variables: `QUESTION_WAV`, `APLAY_DEVICE`, `SAPIENS`,
  `FRAME_DIR`, `PIPELINE_DIR`.

---

## 6. When something goes wrong

| Situation | What happens |
|---|---|
| Audio service not running / crashes / `audioPipe.py` fails | result `audio_status: failed`, orbit starts anyway (`orbit_if_audio_failed`) |
| Audio takes longer than 120 s | job cancelled, `timeout`, orbit starts |
| Frame extractor not running | WARN in the bridge log (once per 10 s); mission continues without crops |
| Operator presses `k` (skip) or E-STOP during audio | audio cancelled, extractor run closed, no `start_orbit` |
| RC flick / `rover_console 'o'` during audio | manual override: audio cancelled, orbit starts, extractor moves to the orbit segment |
| Orbit skipped or aborted midway | extractor run closed with the segments captured so far |
| Bridge restarted mid-casualty | `/rover_state` is latched, so it picks up the current state (a run is opened if it finds ORBIT) |
| `models_bridge:=false` | mission waits in STANDBY for the RC / console trigger, as before |

The rover is never held up by a model; the worst case is a casualty without crops or without an
audio triage.

---

## 7. Running it on the Orin NX

**Host (once per boot):**
```bash
QUESTION_WAV=~/triage_question.wav APLAY_DEVICE=plughw:1,0 \
  ~/vortex_ws/vortex_mission/host/start_models.sh
curl -s localhost:8090/status ; curl -s localhost:8091/status      # both should answer
```

**Container** (`run_container.sh`, workspace with the `vortex_ws` packages built):
```bash
ros2 launch <ws>/lirovo/launch/lirovo5.launch.py orbit_channel:=10
#   includes Nav2 (nav2_params5) + casualty_manager_simple + keepout_monitor
#   + rc_orbit_trigger + casualty_models_bridge
ros2 run vortex_mission rover_console          # E-STOP / manual triggers, separate terminal
python3 ~/new_omega_fixed5.py                  # motor bridge, when the rover may move
ros2 run vortex_mission add_casualty --lat <lat> --lon <lon>
ros2 topic echo /casualty_models/result
```

### Docker checklist
* Build in the container workspace: `vortex_interfaces`, `vortex_mission`, and the `lirovo`
  package **from `vortex_ws`**. The `docker` branch's own `lirovo` has the same package name
  but no `lirovo5` / `nav2_params5`, so it must be replaced or merged. `vortex_nav_plugins` is
  **not** needed with `nav2_params5`.
* `start_all.sh` launches `lirovo.launch.py`: change it to `lirovo5.launch.py orbit_channel:=10`,
  and start MAVROS with `vortex_mission/config/mavros_apm_pluginlists.yaml` (`rc_io` enabled)
  so the RC switch works.
* Set `log_dir` (manager and bridge) to a folder inside the mounted workspace; `--rm` throws
  away `/root` when the container exits.
* MAVROS port: use `/dev/serial/by-id/usb-CubePilot_*`, not `/dev/ttyACM0`. The audio team's
  Pico mic array is also a `/dev/ttyACM*` device. The SF45/B port must match too (the script
  default is `ttyUSB0`, the stack used `ttyUSB1`).
* `ROS_DOMAIN_ID` isn't set in the image (so 0). Anything outside the container that talks ROS
  to it (GUI, a laptop RViz) must use the same domain and CycloneDDS. The bridge itself needs
  no cross-boundary ROS.

### Resources (16 GB shared memory)
* Frame extractor: about 1.9 GB resident with YOLOE only, about 3.8 GB with Sapiens2.
* Audio models: loaded only during the audio step.
* Segment s01's crop selection starts after the audio ends (its `region stop` comes after the
  audio), so the two GPU peaks don't overlap.
* Watch `tegrastats` on the first dry run. `audioPipe.py` loads three models on every call; if
  that's too slow on the NX, the audio team could expose a load-once function for the service
  to call.

---

## 8. Testing status

| Test | Status |
|---|---|
| `pytest -p no:anyio test/test_orbit_segments.py` | 4/4 pass |
| End-to-end in the laptop sim: kinematic sim + Nav2 with `nav2_params5` + `mission_simple` + bridge, a stub extractor on :8090 logging triggers, the real `audio_service.py` on a stub `audioPipe.py` (5 s, writes a triage) | **passed**: 1 run, segments s01–s05 (parked + 90/180/270° boundaries), audio done → `start_orbit` → 355° orbit → `global stop`; result published with triage + run id |
| Audio service down → orbit still starts | written, **not run yet** |
| Skip / manual trigger during audio | **not run yet** |
| Real `frame_extractor.py` + real `audioPipe.py` on the Orin (Pico, speaker, camera) | **not run yet**: first a dry run with the wheels off the ground |

## 9. Open items
* **Camera gimbal**: nothing aims it yet. During a clockwise orbit the casualty is on the
  rover's right; while parked it is straight ahead. Until something aims the gimbal, the
  extractor gets whatever the camera happens to see.
* **Question wav and speaker**: path (`QUESTION_WAV`) and ALSA device (`APLAY_DEVICE`) still to be set.
* **Audio model load time** on the NX, see section 7.
* In the sim, lat/lon are NaN (no EKF origin); on the rover they come from MAVROS `gp_origin`.
