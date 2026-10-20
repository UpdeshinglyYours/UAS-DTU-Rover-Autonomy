# Tele-Ops flags

Two front doors, the same flags, in any order (no `--` needed):

```bash
cd ~/podman_rover && ./rover.sh teleops <flags>     # the usual way (podman stack)
cd ~/teleops      && ./Teleops.sh <flags>           # what rover.sh calls
```

Pick **one mode**, then add any **switches** you like. Switches are independent: combine them freely.
A switch that does not apply to the chosen mode is ignored with a `!! ... ignored` warning; only real
conflicts stop (see the end).

## Load the models ahead (`load` / `unload`)

```bash
./rover.sh load --audio-bypass --skip-audio     # the SAME flags as teleops: only the models load, and stay
./rover.sh teleops --audio-bypass --skip-audio  # starts at once: the loaded models are used
./rover.sh loaded                               # what is loaded right now
./rover.sh teleops --dummy --load                # any run + --load: it loads what it needs, KEEPS it after
./rover.sh unload                               # the models leave memory / VRAM
```
`load` starts a model server (`warm.py`) in teleops_humble that keeps the models; every following
`teleops` run with models runs inside it (output, Ctrl-C and the checks as usual). A run that needs a
model that is not loaded yet loads it once - it then stays too. Smoke tests never use it. The models
take their memory until `unload` (motor ~1.6 GB, + trauma ~1.2 GB, + audio ~5 GB). Measured on the NX:
motor run 66-87 s cold -> 33-40 s with `load`; after `unload` the memory came back (0.8 -> 4.6 GB free).

## The table

### Modes (one)

| flag | runs | loads models | needs camera | needs Pico | needs mavros | what happens |
|---|---|---|---|---|---|---|
| *(none)* | Audio + Trauma | yes | yes | yes¹ | yes² | **rover mode**: Travel -> GUIDED (RC or `G`) -> Stand-By: angle 1 clip + audio + motor -> verdict -> no trauma: MANUAL, Travel / trauma: MANUAL, Change -> GUIDED -> angle 2 -> run closed (DGX) -> Travel |
| `--audio-bypass` = `--bypass` | Audio (+ Trauma with `--trauma`) | yes | yes | yes¹ | no | audio + motor **now, once** (Stand-By); with `--trauma` the angles follow: `G` = the next angle |
| `--audio-test` = `--test` | same | yes | yes | yes¹ | no | the same as `--audio-bypass` |
| `--smoke_ros` | Audio + Trauma (2 processes) | no | no | no | yes | **RC check**: GUIDED -> `Listening...` + one magenta angle; 2 angles close the run |
| `--audio-smoke` = `--smoke` | Audio | no | no | no | no | terminal: `G` GUIDED -> `Listening...`, `M` MANUAL, `Q` quit (no Enter) |
| `--audio-smoke-mic` = `--smoke_mic` | Audio | no | no | yes | no | the same, really listening on the Pico (8 levels/s) |
| `--audio-smoke-ros` | Audio | no | no | no | yes | GUIDED -> `Listening...`, MANUAL -> stop |
| `--trauma-smoke` | Trauma | no | no | no | no | magenta crops published once, served |
| `--trauma-smoke-ros` | Trauma | no | no | no | yes | every GUIDED (or `G`) = one magenta angle; 2 angles close the run |
| `--trauma-bypass` | Trauma | yes | yes | no | no | angle 1 **now**, then `G` for each next angle; the run closes after 2 (gallery, DGX) |
| `--trauma-test` | Trauma | yes | yes | no | no | every angle back to back, then keeps serving |
| `--smoke-camfeed` | Trauma | no | yes | no | no | ONE camera frame, served like real crops |
| `--manual` | Trauma | YOLOE only | yes | no | yes² | **frames by hand**: Enter, the web button or any RC mode change captures 2 frames over 2 s -> YOLOE person crops -> the DGX; `n` = next casualty |

¹ not with `--skip-audio`, `--audio-dummy` or `--verbal-wav`.
² for the RC switch; without it (no autopilot / RC) the terminal keys `G` / `M` / `Q` drive the modes.
² only for the RC mode switch captures; without mavros (or with `--no-manual-rc`) the keys / web button work.

### Switches (independent, combine freely)

| flag | group | applies to | what it changes |
|---|---|---|---|
| `--trauma` | trauma | rover, `--audio-bypass`, `--audio-test` | ALWAYS the trauma angles after the tasks, whatever they found |
| `--skip-audio` | audio (one of) | rover, bypass, test | no audio task at all (no Pico, no audio models); old name `--bypass-audio` still works |
| `--audio-dummy` | audio (one of) | rover, bypass, test | a stand-in audio task: 2 x a `Listening 8 -> 1` countdown, no prompt / Pico / models, verbal = `DUMMY` (-> trauma) |
| `--skip-motor` | motor | rover, bypass, test | skip the motor task; old name `--no-motor` still works (YOLO11m-pose still loads for the face check: `YOLO detects face as (x, y)` before every audio take) |
| `--skip-trauma` | trauma | rover mode | **Audio_Thread only**: Trauma_Thread is not loaded; GUIDED -> audio + motor -> the decision is logged, no angles recorded, no DGX port |
| `--skip-gallery` | trauma | rover, bypass/test with `--trauma`, `--trauma-bypass`, `--trauma-test` | no gallery models: every angle publishes its latest RAW camera frame (same wire format, DGX viewer works) |
| `--dummy` | shortcut | rover, bypass, test | = `--audio-dummy --skip-motor` |
| `--no-speaker` | speaker | rover, bypass, test | no spoken prompts: each is printed; audio: then the 8 s `Listening` countdown; motor: `Act in 5 -> 1` (stands in for the speech), then `Acting 5 -> 1` (the time to act) |
| `--verbal-wav <wav> [wav2]` | mics | rover, bypass, test | recorded 8-channel take(s) instead of the Pico |
| *(no camera flag)* = `--camera direct` (rover.sh) | camera (one of) | any mode with a camera | **default** (rover.sh and the launchers): the pipeline opens the FCB itself with `~/fcbbbb` (fcb_jazzy must be stopped: `./rover.sh down fcb`) |
| `--fcbros` = `--camera ros` (rover.sh) | camera (one of) | same | optional: frames from fcb_jazzy's JPEG stream (`~/fcb_orin_ros`; fcb_jazzy must run) |
| `--file <clip>` | camera (one of) | same | a recorded video inside `~/teleops` instead of the camera |
| `--driver fcbbbb\|manipulator` (rover.sh) | camera | `--camera direct` | which non-ROS FCB driver |
| `--port N` | server | anything that serves the DGX | the trauma server's port (default 8090) |
| `--clip-sec S`, `--angles N` | trauma | rover, bypass/test with `--trauma`, `--trauma-*` | seconds of camera per angle (10) and angles per casualty before the run closes (2); `--timer-segments` (the orbit's) is ignored with a warning |
| `--yoloe` | trauma | rover, bypass, test | YOLOE as the person finder (faster on the NX than Sapiens2 FP4) |
| `--no-sapiens` | trauma | `--trauma-*` modes | crops without Sapiens2 regions |
| `--allow-cpu` | GPU | model modes | run even when the GPU did not come up (very slow) |
| `--dry-run` | - | everything | only the checks and the exact command; nothing starts |
| `--load` | models | any run with models | keep the models loaded after the run (the model server); free them with `./rover.sh unload` |
| `--debug` | console | everything | also the details (settings, paths, per-frame / per-window info, the exact command), each marked ` DEBUG `, and EVERY message shows its time after the component (` Audio_Thread   [HH:MM:SS] ...`); without it only the progress (`~/teleops/stdout/debug_<stamp>.txt` always has everything, with times) |
| `--program Trauma_Thread.py` (rover.sh) | - | - | Trauma alone; without a mode = its rover mode (each GUIDED = an angle) |

## Only one thread
- **Audio_Thread only** (with the RC): `./rover.sh --skip-trauma` (+ any audio / motor switch).
  Once, now, without the RC: `./rover.sh --audio-bypass`.
- **Trauma_Thread only** (with the RC): `./rover.sh --program Trauma_Thread.py` (each GUIDED, or `G`,
  records an angle; 2 angles close the run). Now, without the RC: `./rover.sh --trauma-bypass`
  (+ `--skip-gallery`).
  By hand, frame by frame: `./rover.sh --manual` (see Manual capture below).

## Explanation

### How a run is built
The front door reads the **mode** and the **switches**, works out what the run needs (camera, Pico,
GPU, port, mavros), checks all of it, and only then starts the program in the `teleops_humble`
container. The start line shows the result, e.g.

    [Audio_Thread.sh] mode bypass | audio real | motor on | trauma FORCED | camera fcbros | Pico

Add `--dry-run` to see that line and the exact command without starting anything.

### Modes
- **Rover mode** (no mode flag) is the real mission, on three modes:
  - **Travel** (the start). Nothing runs until GUIDED: the RC switch, or `G` in the terminal.
  - **Stand-By** (GUIDED, at the casualty). Trauma_Thread records angle 1, a 10 s clip. Meanwhile
    the audio task asks the casualty, records the Pico, and Whisper + Qwen judge it; the motor task
    follows ("raise one arm"). Then the decision:
    - **no trauma** (verbally NORMAL): the pipeline sets MANUAL -> **Travel**;
    - **trauma** (or always, with `--trauma`): angle 1 goes to the gallery in the background, its crops
      are served, and the pipeline sets MANUAL -> **Change**.
  - **Change**: the pilot drives to the next angle; nothing records. The next GUIDED is Stand-By
    again: a 10 s clip of angle 2 -> the gallery. After the last angle (2) the run closes (the DGX
    runs the trauma inference) and the pipeline sets MANUAL -> **Travel**.
- **The terminal is the master RC**, with or without a real RC. Single keys, no Enter, at any time:
  `G` = GUIDED, `M` = MANUAL, `Q` = quit (= Ctrl-C). Each key shows under the **RC** badge. With
  MAVROS up, `G` / `M` also set the rover's mode. It works with loaded models too (`--load`).
- **`--audio-bypass` / `--audio-test`** start at Stand-By immediately, once, without mavros. With
  `--trauma` the angles follow: `G` records each next angle, and the run closes after the last.
  Q / Ctrl-C once closes the casualty and keeps serving; again exits.
- **Smoke modes** load no models. `--smoke_ros` is the integration check for the RC switch: both
  threads listen to the real `/mavros/state` and show that every switch arrives. Smoke modes never
  command the rover.
- **`--trauma-*` and `--smoke-camfeed`** run only the trauma side.
- **`--manual`** runs only the trauma side, with the frames chosen by a person (below).

### Manual capture (`--manual`)
Frame selection by hand instead of the automatic angle clips. Trauma_Thread alone; no Pico, no
Sapiens2 on the NX. mavros only for the RC mode switch (below); without it the keys / web button work.

```bash
./rover.sh --manual --fcbros        # camera via fcb_jazzy (it keeps running)
./rover.sh --manual                 # camera opened directly (fcb_jazzy stopped)
./rover.sh --manual --file data/test_clip.mp4   # bench test
```

| button | what happens |
|---|---|
| **Enter** in the terminal, **Capture** on `http://<NX-IP>:8090/manual` (`POST /capture`), or **any RC mode change** (the mode switch: whichever mode -> whichever mode, from `/mavros/state`) | a capture: 2 camera frames over 2 s (the newest frame at the press, then one 2 s later) -> YOLOE person box on each -> the padded person crops (the same crop as the DGX galleries) -> published together as one segment; the DGX segments them (Sapiens2) and runs the injury model at once |
| **n** + Enter, or **Next casualty** (`POST /next`) | the casualty (run) closes -> the DGX pools all its captures into the final verdict; the next capture starts a new casualty |
| **q** + Enter, or Ctrl-C | stops capturing, closes the open casualty, keeps serving; again = exit |

- A capture with no person in the frame sends nothing (yellow warning): aim and capture again. A frame
  without a person is dropped; the other frame of the capture still goes.
- A press / mode change while a capture is still taking its frames (within the 2 s) is ignored: flipping
  the switch there and straight back = one capture. `n` / Next casualty / Ctrl-C wait for it to finish,
  so it still belongs to the casualty being closed.
- Every mode change counts, also one made by the ground station or the autopilot (failsafe), not only
  the RC switch. The console shows each one (`[mode] MANUAL -> GUIDED`). RC needs the rover container
  (rover_humble, mavros) up; `--no-manual-rc` = keys / web button only.
- `--manual-frames N` (default 2; 1 = the press only), `--manual-span S` (default 2 s: first to last frame).
- A frame older than 2 s (`MANUAL_MAX_AGE`) is captured with a warning (camera stalled?).
- The web page also takes Space / Enter = Capture, `n` = Next casualty, and shows the last result.
- The first capture of a casualty opens it; no capture = no run on the DGX.
- `--manual` never runs in the model server (`--load`): its keys need the terminal. YOLOE loads in a
  few seconds at start.
- On the DGX: the usual receiver (`live_triage.sh` / `run_trauma.py --obc http://<NX-IP>:8090`); each
  capture is a segment with up to two crops.

### Switches
- **Audio** is one choice: real (default), `--skip-audio` (none) or `--audio-dummy` (stand-in).
- **Motor**, **speaker**, **Pico** (`--verbal-wav`) and **trauma** (`--trauma`) are independent on/off.
- **Camera** is one choice: fcbbbb (direct, the default), `--fcbros` (via fcb_jazzy) or `--file`.
- Typical combinations:
  - `--dummy --trauma`: GUIDED -> `Listening...` -> the real angles. The full flow with the audio faked.
  - `--audio-bypass --trauma --skip-motor`: real audio now, then the angles, no motor task.
  - `--skip-audio`: **motor + trauma, no audio** in rover mode (GUIDED -> motor -> angles).
  - `--audio-bypass --skip-audio --trauma`: motor task now, then the angles.
  - `--audio-bypass --verbal-wav data/test_take.wav --no-speaker --file data/test_clip.mp4`: the whole
    task with no hardware at all.

### What stops (`STOP: ... / fix: ...`, exit 2)
A mistyped flag stops at once, before anything starts, with the closest flag and - when every
typo has a confident match - the whole corrected command to copy (`did you mean:  ./rover.sh ...`).
It also fixes a missing or single dash (`skip-motor`, `-skip-motor` -> `--skip-motor`) and `_` for `-`.
Two modes at once; two audio choices (`--skip-audio` + `--audio-dummy`); two camera sources;
`--source/--device/--fallback-source/--fcb-driver` (the front door chooses them); typos of flags
(`--fcbross`); the camera held by another program, or fcb_jazzy not running for `--fcbros`; the port
in use; the Pico / GPU missing when needed; another teleops run holding the same camera, Pico or port.

### mavros
The navigation stack runs mavros on the rover's Wi-Fi/Ethernet network, while fcb_jazzy and
teleops_humble use the loopback network. **`mavros_bridge.py`** carries `/mavros/state`,
`/mavros/rc/in` and `/frame_extraction/trigger` to the pipeline, and `/mavros/statustext/send`,
`/teleops/operator` and the `/mavros/set_mode` service back. The front door starts it automatically
whenever a mode needs mavros (one per container; log: `~/teleops/data/mavros_bridge.log`). It does
nothing when the networks are the same.

### Console look
One line per message: the component as a badge (white on its colour), then the message in plain words.
A sub-component's badge starts with 2 characters in its parent's colour (Motor in Audio_Thread's;
Occular will be in Trauma_Thread's). Red = error, yellow = warning / fallback, green = a model's result.
`--debug` adds the technical details (marked DEBUG).

### Output
Each line starts with its component as a badge - white text on the component's colour
(Launcher, Audio_Thread, Trauma_Thread, Motor, Triage, Speaker, Camera, ...) - then the message:
**red** = an error, **yellow** = a fallback / warning, **green** = a model's prediction (verbal and
motor results, the decision, crops published), plain = progress. Only the PROGRESS is shown; add
`--debug` for the specifics, printed as ` DEBUG ` (black on white) + the component's name in its
colour + the message. Two harmless blocks are folded away (PyTorch's "compute capability 8.7"
warning, the ROS `rcutils ... serdata` notice). `NO_COLOR=1` = plain text.
Bench angles (`--trauma-bypass`, `--audio-bypass --trauma`): `G` records each next angle; `--trauma-test`
records them back to back. Ctrl-C / Q once closes the run; again finishes the queued segments and
exits (code 0); a third time exits at once (code 130) and says what was not published.

### Loader (which models, and is there memory?)
Before anything loads, the **Loader** lists the models this run needs - worked out from the flags -
with their size, estimates the memory and compares it with what is free:

    Loader  models for this run (1, 0.0 GB on disk):
    Loader  1. motor  YOLO11m-pose - posture, arm raise, movement (42 MB)
    Loader  memory: needs ~1.6 GB, 5.4 GB free - ok
    Loader  loading motor (YOLO11m-pose) ...
    Loader  motor (YOLO11m-pose) ready in 12.8s

| model | group | loaded when | on disk |
|---|---|---|---|
| SepFormer, Whisper base.en, Qwen3.5 0.8B triage | audio | real audio task (not with `--skip-audio` / `--audio-dummy`) | ~2.0 GB |
| YOLO11m-pose | motor | the motor task (not with `--skip-motor`) | 42 MB |
| Sapiens2 0.8B FP4 (regions + person finder) | trauma | the trauma angles (not with `--skip-gallery`) | 0.5 GB |
| YOLOE-11L person | trauma | `--yoloe`, `--manual` (then the only model) | 68 MB |

The NX has 16 GB shared by CPU and GPU and **no swap**: when the run does not fit, the kernel kills it
(`Killed`, exit 137). The launcher warns first when less than 8 GB is free and names the biggest
programs (e.g. a `llama-server` of another team, 5 GB). The full audio + motor + trauma run needs
~8 GB; motor alone ~1.6 GB (`--audio-bypass --skip-audio`), trauma alone ~2.8 GB.

Every run gets a **new folder, `~/teleops/data/recordings/<YYYY-MM-DD_HH-MM-SS>/`** (never overwritten;
the console ends with `Saved in ...`):
- `01_camera_feed.mp4` - the whole camera feed (mp4, 10 fps, ~2.6 MB/s at 1080p; `checks.py` `MEDIA_FPS`)
- `02_<casualty>_audio_take1.wav`, ... - the Pico takes; `NN_<casualty>_motor_<step>.mp4` - what the pose model saw
- `images/audio_feed/`, `images/trauma_clips/`, `images/camfeed/` - the raw frames each part used
- `images/sent_to_dgx/` - every image sent to the DGX (crops, raw frames, the camfeed frame)
- `run.json` - the command typed, the mode, every file, each casualty's results, the exit code
- the run's full log is `~/teleops/stdout/debug_<same stamp>.txt` - every console line as `<component> [HH:MM:SS] <message>`, INCLUDING the debug details (also without `--debug`)
It belongs to your user (no sudo to delete) and is kept out of the Docker image. Old folders add up:
delete them by hand.

### On the DGX
Copy the `DGX viewer:` line printed at start:
`python3 ~/NX_rover1_pipeline/show_streamed_image.py http://<NX-IP>:<port>`.
Every served image is also saved on the NX, in the run folder's `images/sent_to_dgx/`.

### Before a real run
- GPU: `ls /dev/nvgpu/igpu0/ctrl` (missing -> the `/etc/firmware` fix in `ABOUT.md` 10.9).
- Camera: `sudo podman logs --tail 2 fcb_jazzy` shows about 60 fps (0 fps -> cycle the camera's 12 V).
- mavros: the start line shows `mavros bridge: ... /mavros/state: mode ..., connected True`.
- The rover is often ARMED: rover mode, `--dummy` and `--audio-dummy` switch it to MANUAL themselves.
