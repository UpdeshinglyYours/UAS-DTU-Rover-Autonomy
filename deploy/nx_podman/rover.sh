#!/usr/bin/env bash
# The rover's containers on rootful podman (replaces docker's run_container.sh).
#
#   rover_humble  uas_rover image, ROS 2 Humble: MAVROS, Blickfeld, SF45/B,
#                 Nav2, lirovo ... and the rosbag (record_sensors.sh)
#   fcb_jazzy     fcb_jazzy image, ROS 2 Jazzy: the FCB camera driver, with
#                 the Jetson's hardware encoder for its AVI recorder
#
# Both sit on the host network with one CycloneDDS profile, so their topics
# meet as if they were one machine (they are), on one clock.
#
#   ./rover.sh up [rover|fcb]      start both containers (or one)
#   ./rover.sh down [rover|fcb]    stop (asks before stopping the camera driver)
#   ./rover.sh shell [rover|fcb]   a shell inside (default rover); ROS sourced
#   ./rover.sh rec [args]          sensors + one rosbag in the Humble container:
#                                  record_sensors.sh --no-camera [args]
#   ./rover.sh keys                fcb_keys: zoom / RGB-IR / record by key
#   ./rover.sh status              containers, ROS nodes, camera status line
#   ./rover.sh logs [rover|fcb]    follow a container's output
#   ./rover.sh doctor              Humble <-> Jazzy DDS check (talker/listener)
#
#   ./rover.sh teleops [flags]   the gate-3 casualty pipeline (~/teleops) in its own Humble
#                                ("teleops" may be left out: ./rover.sh --smoke-camfeed works too)
#                                container (teleops_humble), through ~/teleops/Teleops.sh (which
#                                checks camera, Pico, port, GPU and locks first and cleans up after).
#                                Every flag below works directly, in any order (no "--" needed):
#     rover mode (both threads; Travel -> GUIDED -> Stand-By: angle 1 clip + audio + motor -> trauma?
#                 MANUAL, Change -> GUIDED -> angle 2 -> run closed (DGX) -> Travel; terminal G / M / Q):
#       (none)                   the pipeline opens the FCB with ~/fcbbbb (= --camera direct; fcb_jazzy must be down)
#       --fcbros                 camera from fcb_jazzy's JPEG stream instead (= --camera ros)
#       --driver fcbbbb|manipulator   for the direct camera (non-ros = fcbbbb)
#       --file <clip>            a video inside ~/teleops instead of the camera
#       --trauma                 every casualty gets the trauma angles (also with --audio-bypass)
#       --skip-audio             no verbal task (no Pico): GUIDED -> motor -> trauma angles
#       --skip-motor             no motor task
#       --skip-gallery           every angle sends its RAW camera frame, no gallery models
#       --skip-trauma            Audio_Thread only: GUIDED -> audio + motor -> decision logged, no angles
#       --audio-dummy            the audio task is a DUMMY ('Listening...' 10 s, no Pico / models)
#       --dummy                  = --audio-dummy --skip-motor: GUIDED -> 'Listening...' -> real angles
#       --clip-sec S, --angles N the clip per angle (10 s) and the angles per casualty (2)
#       --port N                 the trauma server's port for the DGX (default 8090)
#       --no-speaker, --allow-cpu, --dry-run, ...   -> the launcher / program
#       --debug                  also the details (marked DEBUG); without it only the progress
#       --load                   keep the models loaded after the run (./rover.sh unload frees them)
#     recordings: a NEW folder per run, ~/teleops/data/recordings/<date_time>/ (never overwritten): every
#            video / audio / image of the run (01_camera_feed.mp4, 02_..._audio_take1.wav, images/...) + run.json
#     switches combine freely (one that does not apply is ignored with a warning); FLAGS.md
#     tests (one; no models unless noted):
#       --smoke_ros              BOTH threads on the real /mavros/state (RC check): GUIDED ->
#                                "Listening...", MANUAL -> run + magenta crops every 5 s, GUIDED -> closed
#       --audio-smoke | --smoke          Audio, keyboard g/m/q
#       --audio-smoke-mic | --smoke_mic  Audio, really listening on the Pico
#       --audio-smoke-ros                Audio only, real /mavros/state
#       --audio-bypass | --bypass        Audio + motor task NOW, once (models)
#       --audio-test | --test            = --audio-bypass
#       --trauma-smoke                   Trauma, magenta crops published once
#       --trauma-smoke-ros               Trauma only, real /mavros/state
#       --trauma-bypass                  Trauma angles NOW: angle 1, then G per angle (models, camera)
#       --trauma-test                    every Trauma angle back to back (models, camera)
#       --smoke-camfeed                  ONE camera frame, served like real crops (--camfeed = old name)
#       --manual                         frames BY HAND: Enter in this terminal, the Capture button on
#                                        http://<NX-IP>:8090/manual, or ANY RC mode change = a capture of
#                                        2 frames over 2 s -> YOLOE person crops -> the DGX (Sapiens2 +
#                                        injury model there); n + Enter = next casualty (DGX verdict);
#                                        q + Enter / Ctrl-C = stop. RC needs rover_humble (mavros) up;
#                                        --no-manual-rc = keys / web only
#       --program Trauma_Thread.py [--smoke|--bypass|--test|--smoke-camfeed|--smoke_ros]
#                                        Trauma_Thread alone (rover mode: each GUIDED = an angle)
#   ./rover.sh down teleops        stop the pipeline container
#   ./rover.sh load [flags]        the teleops flags, but ONLY the models load - and stay loaded (a
#                                  warm process in teleops_humble): every following ./rover.sh teleops ...
#                                  runs in it and starts at once (models it lacks load once and stay)
#   ./rover.sh loaded              is a warm process up, which models does it hold?
#   ./rover.sh unload              end it: the models leave memory / VRAM
#
# Environment:
#   ROS_NET=net       autodetected network discovery (rviz from a laptop)
#                     instead of loopback only. Must match across containers:
#                     restart both after changing it.
#   ROS_DOMAIN_ID     default 0
#   FCB_ARGS          extra launch args for the driver, e.g. "icr:=ir"
#   FCB_START_ZOOM    zoom the camera goes to when fcb_jazzy gets camera control
#                     (default 1.0 = widest; 0 = leave it as found). The ch7 knob
#                     takes over as soon as it reports.
#   FCB_JPEG_WIDTH    width of fcb_jazzy's JPEG stream ~/image_raw/compressed
#                     (default 0 = full 1920; the pipeline's trauma crops want it)
#   FCB_JPEG_HZ       its rate (default 15; the pipeline's motor task needs >= 10, tuned at 15).
#                     Both are fixed when fcb_jazzy starts; the bag records this stream
#                     (1920 @ 15 Hz is ~4 MB/s).
#
#   ./rover.sh autostart on|off|status   start rover_humble + fcb_jazzy at every
#                                  boot (a @reboot line in rover1's crontab that
#                                  runs ./rover.sh boot; log: boot.log here)
#   ./rover.sh boot                what the boot runs: wait for the GPU, then up.
#                                  If the GPU did not come up (seen: "nvgpu ...
#                                  Bootstrap HS ACR failed" after a power cut),
#                                  the containers start WITHOUT it rather than
#                                  not at all; reboot to get the GPU back.
#                                  MAVROS/Blickfeld/bag (rec) and the pipeline
#                                  (teleops) are not started at boot.
#
# Do not stop fcb_jazzy between flights: on the Oppila board, closing the
# video stream needs a 12 V power cycle to recover. Start and stop recordings
# with RC ch8 or ./rover.sh keys, and new bags with ./rover.sh rec --bag.
set -u

DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
P="sudo podman"
ROVER=rover_humble
FCB=fcb_jazzy
ROVER_IMAGE="${ROVER_IMAGE:-docker.io/library/uas_rover:latest}"
FCB_IMAGE="${FCB_IMAGE:-localhost/fcb_jazzy:latest}"
U_HOME=/home/rover1
GST=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
RECORD_SENSORS=/workspace/rover_ws/src/UAS-DTU-navigation-simulation/lirovo/lirovo/record_sensors.sh
TELEOPS_DIR="${TELEOPS_DIR:-$U_HOME/teleops}"
TELEOPS=teleops_humble                 # humble.sh's container name
FCB_JPEG_WIDTH="${FCB_JPEG_WIDTH:-0}"
FCB_JPEG_HZ="${FCB_JPEG_HZ:-15}"
FCB_START_ZOOM="${FCB_START_ZOOM:-1.0}"

# -- shared options ---------------------------------------------------------
dds_args() {
    echo "-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    if [[ "${ROS_NET:-local}" != net ]]; then
        echo "-v $DIR/cyclonedds_local.xml:/etc/cyclonedds_local.xml:ro"
        echo "-e CYCLONEDDS_URI=file:///etc/cyclonedds_local.xml"
    fi
}

# --device nvidia.com/gpu=all needs a CDI spec podman 4.9 can parse.
#  - /var/run/cdi is a tmpfs: nvidia-cdi-refresh regenerates the spec at boot,
#    but not always (seen after a 1970-clock boot), so it is started if absent.
#  - NVIDIA Container Toolkit 1.19 writes CDI 0.7.0 with `additionalGids`,
#    which podman 4.9's CDI library rejects ("unknown field"), dropping the
#    whole spec. A copy without those blocks, as CDI 0.6.0, goes to /etc/cdi
#    (podman skips the unparsable one). The groups they added (video, render)
#    come from --privileged as root, or FCB_GROUPS in the camera container.
#    Written through podman, the one root command this user has.
CDI_SRC=/var/run/cdi/nvidia.yaml
CDI_FIX=/etc/cdi/nvidia-podman.yaml

# The Orin's GPU is up: its device nodes exist. After some power cuts it is not
# ("nvgpu: Bootstrap HS ACR failed", only /dev/nvgpu/igpu0/power): a container
# asking for the CDI device would then not start at all.
gpu_ok() { [[ -e /dev/nvgpu/igpu0/ctrl && -e /dev/nvgpu/igpu0/as ]]; }
gpu_args() {
    if gpu_ok && [[ -f $CDI_FIX ]]; then echo "--device nvidia.com/gpu=all"; fi
}

ensure_cdi() {
    if ! gpu_ok; then
        echo "!! GPU not available on this boot (no /dev/nvgpu/igpu0/ctrl; dmesg | grep nvgpu):" >&2
        echo "!! containers start WITHOUT the GPU (no CUDA, CPU video encoder). Reboot to retry." >&2
        return 0
    fi
    if [[ ! -f $CDI_SRC ]]; then
        echo "NVIDIA CDI spec missing -- regenerating (nvidia-cdi-refresh)"
        sudo systemctl start nvidia-cdi-refresh.service
        for _ in 1 2 3 4 5 6 7 8 9 10; do [[ -f $CDI_SRC ]] && break; sleep 1; done
        if [[ ! -f $CDI_SRC ]]; then
            [[ -f $CDI_FIX ]] && { echo "($CDI_SRC not regenerated; using the saved $CDI_FIX)"; return 0; }
            echo "error: $CDI_SRC still missing and no $CDI_FIX" >&2; return 1
        fi
    fi
    [[ -f $CDI_FIX && $CDI_FIX -nt $CDI_SRC ]] && return 0
    local tmp; tmp=$(mktemp -d)
    python3 - "$CDI_SRC" "$tmp/nvidia-podman.yaml" <<'EOF' || { rm -rf "$tmp"; return 1; }
import re, sys
out, skip = [], None
for line in open(sys.argv[1]):
    indent = len(line) - len(line.lstrip())
    if skip is not None:
        if line.strip() and (indent > skip or (indent == skip and line.lstrip().startswith("- "))):
            continue
        skip = None
    if line.lstrip().startswith("additionalGids:"):
        skip = indent
        continue
    out.append(re.sub(r"^cdiVersion: .*", "cdiVersion: 0.6.0", line))
open(sys.argv[2], "w").writelines(["# generated by rover.sh from %s for podman 4.9\n" % sys.argv[1]] + out)
EOF
    chmod 644 "$tmp/nvidia-podman.yaml"
    $P run --rm --network none --entrypoint cp -v "$tmp":/in:ro -v /etc/cdi:/out "$ROVER_IMAGE" \
        /in/nvidia-podman.yaml /out/nvidia-podman.yaml
    rm -rf "$tmp"
    [[ -f $CDI_FIX ]] && echo "wrote $CDI_FIX (podman-compatible NVIDIA CDI spec)"
}

running() { [[ "$($P inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" == true ]]; }

start_rover() {
    running $ROVER && { echo "$ROVER already running"; return 0; }
    xhost +local:root >/dev/null 2>&1
    # Same mounts as docker's run_container.sh, plus the home directory at its
    # own path so the DL models and venvs scattered under it resolve unchanged.
    $P run -d --replace --name $ROVER \
        --network host --ipc host --privileged \
        $(gpu_args) -e NVIDIA_DISABLE_REQUIRE=1 \
        $(dds_args) \
        -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v /etc/localtime:/etc/localtime:ro \
        -v $U_HOME:$U_HOME -v $U_HOME/rover_ws:/workspace/rover_ws \
        "$ROVER_IMAGE" sleep infinity >/dev/null \
    && echo "$ROVER started (Humble)"
}

# The camera container's options (also used by doctor's encoder test).
# Runs as rover1 (entrypoint's setpriv) so recordings are not root-owned; its
# groups (video, dialout, render ...) open the camera and the encoder. The
# host's NVIDIA GStreamer plugins match this Ubuntu 24.04 image's GStreamer.
fcb_opts() {
    echo "--network host --ipc host --privileged $(gpu_args)"
    echo "-v $GST/libgstnvvideo4linux2.so:$GST/libgstnvvideo4linux2.so:ro"
    echo "-v $GST/libgstnvvidconv.so:$GST/libgstnvvidconv.so:ro"
    dds_args
    echo "-e FCB_UID=$(id -u rover1) -e FCB_GID=$(id -g rover1)"
    echo "-e FCB_GROUPS=$(id -G rover1 | tr ' ' ',') -e HOME=$U_HOME"
    echo "-v /dev:/dev -v /run/udev:/run/udev:ro -v /etc/localtime:/etc/localtime:ro"
    echo "-v $U_HOME:$U_HOME"
}

start_fcb() {
    running $FCB && { echo "$FCB already running"; return 0; }
    # One program at a time may hold the camera.
    if pgrep -f "fcb_orin_driver/fcb_driver|fcb_record.py" >/dev/null; then
        echo "error: a camera driver is already running on the host: $(camera_pid_names "$(pgrep -f "fcb_orin_driver/fcb_driver|fcb_record.py")") (~/fcb_orin_ros/fcb_ros.sh --stop)" >&2
        return 1
    fi
    if running $ROVER && $P exec $ROVER pgrep -f "fcb_orin_driver/fcb_driver" >/dev/null 2>&1; then
        echo "error: the Humble container runs its own fcb_driver (record_sensors.sh without --no-camera)" >&2
        return 1
    fi
    mkdir -p $U_HOME/flight_recordings
    # --init + SIGINT: ros2 launch as PID 1 ignored podman's SIGTERM and was
    # SIGKILLed after the timeout, cutting the AVI off unfinalised. With an
    # init forwarding SIGINT, the launch stops the driver as Ctrl-C would.
    $P run -d --replace --name $FCB --restart on-failure \
        --init --stop-signal SIGINT --stop-timeout 45 $(fcb_opts) \
        "$FCB_IMAGE" \
        ros2 launch fcb_orin_driver fcb_driver.launch.py \
            fc_source:=mavros publish_compressed:=true \
            compressed_width:=$FCB_JPEG_WIDTH compressed_rate_hz:=$FCB_JPEG_HZ \
            start_zoom:=$FCB_START_ZOOM \
            record_dir:=$U_HOME/flight_recordings ${FCB_ARGS:-} >/dev/null \
    && echo "$FCB started (Jazzy camera driver) -- ./rover.sh logs fcb"
}

pick() { case "${1:-}" in fcb) echo $FCB ;; rover|"") echo $ROVER ;; teleops) echo $TELEOPS ;; *) echo "$1" ;; esac; }

# fcb_jazzy's JPEG stream as started (width, rate), from its command line
fcb_jpeg() {
    local cmd w r
    cmd=$($P inspect -f '{{join .Config.Cmd " "}}' $FCB 2>/dev/null)
    w=$(sed -n 's/.*compressed_width:=\([0-9]*\).*/\1/p' <<<"$cmd"); r=$(sed -n 's/.*compressed_rate_hz:=\([0-9.]*\).*/\1/p' <<<"$cmd")
    echo "${w:-960} ${r:-10}"
}

# 'PID 12 "fcb_view.py"' for each PID: ~/teleops/ui.sh's pid_names (names the program), plain text
camera_pid_names() {
    if [[ -f $TELEOPS_DIR/ui.sh ]]; then (source "$TELEOPS_DIR/ui.sh"; _ui_plain "$(pid_names $1)")
    else echo "PID(s) $(echo $1)"; fi
}

# Anything else holding the FCB (only one program may: video + VISCA port)
other_camera_users() {
    pgrep -af "fcb_orin_driver/fcb_driver|fcb_record.py|fcb_track.py|camera_node|fcb_view.py" \
        | grep -v -E "pgrep|conmon" || true
}

# "./rover.sh --smoke-camfeed" = "./rover.sh teleops --smoke-camfeed": a first argument that is a flag means the pipeline
export TELEOPS_CMDLINE="./rover.sh $(printf '%q ' "$@")"   # the launchers' typo checker echoes it, corrected
export TELEOPS_ROVER_SH="$(readlink -f "${BASH_SOURCE[0]}")"
[[ "${1:-}" == --* && "${1:-}" != --help ]] && set -- teleops "$@"
# "./rover.sh load <teleops flags>": the same checks and flags, but only the models load (and stay)
[[ "${1:-}" == load ]] && { export TELEOPS_WARM=load; set -- teleops "${@:2}"; }
case "${1:-}" in
unload)                                         # the warm process ends: models out of memory / VRAM
    TELEOPS_COLOR=1 TELEOPS_FORCE_COLOR=$([ -t 1 ] && echo 1) python3 "$TELEOPS_DIR/warm_client.py" unload --engine "sudo -n podman" --container "$TELEOPS"
    exit $? ;;
loaded)                                         # is a warm process up, which models does it hold?
    TELEOPS_COLOR=1 TELEOPS_FORCE_COLOR=$([ -t 1 ] && echo 1) python3 "$TELEOPS_DIR/warm_client.py" status --engine "sudo -n podman" --container "$TELEOPS"
    exit $? ;;
up)
    ensure_cdi || exit 1
    case "${2:-all}" in
        rover) start_rover ;;
        fcb)   start_fcb ;;
        *)     start_rover; start_fcb ;;
    esac ;;
down)
    for c in $( [[ -n "${2:-}" ]] && pick "$2" || echo "$FCB $ROVER" ); do
        running "$c" || { echo "$c not running"; continue; }
        if [[ $c == "$TELEOPS" ]]; then
            TELEOPS_ENGINE=podman "$TELEOPS_DIR/humble.sh" off
            continue
        fi
        if [[ $c == "$FCB" ]]; then
            echo "Stopping the camera driver closes the camera stream; on the Oppila board"
            echo "the video usually does not come back until its 12 V is power-cycled."
            read -r -p "Stop $FCB anyway? [y/N] " a; [[ "$a" == [yY]* ]] || { echo "left running"; continue; }
            # SIGTERM -> the launch finalises the AVI/CSV (sigterm_timeout 15 s)
            $P stop -t 40 $FCB >/dev/null && echo "$FCB stopped"
        else
            $P exec $ROVER bash -c "tmux has-session -t rover_rec 2>/dev/null && echo 'stop the recording first: ./rover.sh rec --stop'" | grep . && continue
            $P stop -t 20 $ROVER >/dev/null && echo "$ROVER stopped"
        fi
    done ;;
shell)
    c=$(pick "${2:-}")
    if [[ $c == "$FCB" ]]; then
        exec $P exec -it --user "$(id -u rover1):$(id -g rover1)" -e HOME=$U_HOME -w $U_HOME $FCB \
            /usr/local/bin/fcb_entrypoint.sh bash
    fi
    exec $P exec -it $c bash ;;
rec)
    shift
    running $ROVER || { echo "$ROVER is not running: ./rover.sh up" >&2; exit 1; }
    exec $P exec -it $ROVER bash -ic "$RECORD_SENSORS --no-camera $*" ;;
keys)
    exec $P exec -it --user "$(id -u rover1):$(id -g rover1)" -e HOME=$U_HOME $FCB \
        /usr/local/bin/fcb_entrypoint.sh ros2 run fcb_orin_driver fcb_keys ;;
logs)
    exec $P logs -f --tail 100 "$(pick "${2:-}")" ;;
status)
    $P ps -a --filter name=$ROVER --filter name=$FCB --filter name=$TELEOPS --format '  {{.Names}}\t{{.Status}}\t{{.Image}}'
    if running $FCB; then
        $P exec $FCB /usr/local/bin/fcb_entrypoint.sh bash -c \
            "timeout 5 ros2 topic echo --once /fcb_driver/status_text 2>/dev/null" | sed -n 's/^data: /  camera: /p'
    fi
    if running $ROVER; then
        echo "  nodes (seen from Humble):"
        $P exec $ROVER bash -ic "timeout 10 ros2 node list --no-daemon --spin-time 3 2>/dev/null" | sed 's/^/    /'
        $P exec $ROVER bash -c "tmux has-session -t rover_rec 2>/dev/null && tmux list-windows -t rover_rec -F '  rec: #{window_name} #{pane_current_command}'"
    fi ;;
doctor)
    ensure_cdi || exit 1
    running $ROVER || start_rover
    running $FCB || { echo "(fcb_jazzy not running: using a throwaway Jazzy container)"; }
    J="$P run --rm --network host --ipc host $(dds_args) $FCB_IMAGE"
    echo "== Jazzy talker -> Humble listener"
    $J timeout 12 ros2 topic pub -r 2 /chatter std_msgs/msg/String '{data: jazzy}' >/dev/null 2>&1 &
    $P exec $ROVER bash -ic "timeout 10 ros2 topic echo --once /chatter std_msgs/msg/String" 2>/dev/null | grep -q data \
        && echo "   OK" || echo "   FAILED"
    wait
    echo "== Humble talker -> Jazzy listener"
    $P exec -d $ROVER bash -ic "timeout 12 ros2 topic pub -r 2 /chatter_h std_msgs/msg/String '{data: humble}'" >/dev/null
    $J timeout 10 ros2 topic echo --once /chatter_h std_msgs/msg/String 2>/dev/null | grep -q humble \
        && echo "   OK" || echo "   FAILED"
    echo "== hardware H.264 encoder (nvv4l2h264enc) in fcb_jazzy, as rover1"
    $P run --rm $(fcb_opts) "$FCB_IMAGE" bash -c \
        "gst-launch-1.0 -q videotestsrc num-buffers=120 ! video/x-raw,width=1920,height=1080,framerate=60/1 \
         ! videoconvert ! video/x-raw,format=NV12 ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
         ! nvv4l2h264enc bitrate=25000000 insert-sps-pps=1 maxperf-enable=1 ! h264parse ! avimux \
         ! filesink location=/tmp/nvenc_test.avi >/dev/null 2>&1 && test -s /tmp/nvenc_test.avi" \
        && echo "   OK" || echo "   FAILED (the recorder will fall back to the CPU encoder)" ;;
boot)
    # run by cron at boot (./rover.sh autostart on). Give the GPU driver time to come up.
    echo "==== $(date '+%F %T') boot"
    for _ in $(seq 1 60); do gpu_ok && break; sleep 2; done
    gpu_ok && echo "GPU up" || echo "!! GPU did not come up within 120 s"
    ensure_cdi || exit 1
    start_rover; start_fcb
    $P ps -a --filter name=$ROVER --filter name=$FCB --format '  {{.Names}} {{.Status}}' ;;
autostart)
    LINE="@reboot $DIR/rover.sh boot >> $DIR/boot.log 2>&1"
    case "${2:-status}" in
        on)     { crontab -l 2>/dev/null | grep -vF "$DIR/rover.sh boot"; echo "$LINE"; } | crontab - \
                && echo "autostart ON: rover_humble + fcb_jazzy start at boot (log: $DIR/boot.log)" ;;
        off)    { crontab -l 2>/dev/null | grep -vF "$DIR/rover.sh boot"; } | crontab - \
                && echo "autostart OFF" ;;
        status) if crontab -l 2>/dev/null | grep -qF "$DIR/rover.sh boot"; then echo "autostart ON"
                else echo "autostart OFF"; fi
                [[ -f $DIR/boot.log ]] && { echo "last boot run:"; tail -6 "$DIR/boot.log" | sed 's/^/  /'; } ;;
        *)      echo "usage: ./rover.sh autostart on|off|status" >&2; exit 1 ;;
    esac ;;
teleops)
    shift
    # ~/teleops's own launchers do the checks (camera / driver, Pico, port, GPU, locks) and the
    # clean-up after an ssh drop: Teleops.sh (both threads), Audio_Thread.sh, Trauma_Thread.sh.
    # This only adds the podman side: CDI, fcb_jazzy started for --camera ros, the JPEG check.
    CAMERA=direct; DRIVER=fcbbbb; PROG=Audio_Thread.py; FILE=""; EXTRA=(); PASS=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --skip-audio|--bypass-audio) EXTRA+=(--skip-audio); shift ;;
            --trauma)            EXTRA+=(--trauma); shift ;;
            --timer-segments)    echo "warning: --timer-segments: the orbit is gone (casualty angles now) - ignored" >&2; shift ;;
            --camera)            CAMERA="$2"; shift 2 ;;
            --fcbros)            CAMERA=ros; shift ;;
            --driver|--fcb-driver) DRIVER="$2"; [[ $DRIVER == non-ros ]] && DRIVER=fcbbbb; shift 2 ;;
            --file)              FILE="$2"; shift 2 ;;
            --program)           PROG="$2"; shift 2 ;;
            --)                  shift; break ;;
            *)                   PASS+=("$1"); shift ;;          # the launcher's flags (any order)
        esac
    done
    PASS+=("$@")
    case "$PROG" in
        Audio_Thread.py|Audio_Thread.sh|audio) PROG=Audio_Thread.py ;;
        Trauma_Thread.py|Trauma_Thread.sh|trauma) PROG=Trauma_Thread.py ;;
        *) echo "error: --program Audio_Thread.py|Trauma_Thread.py" >&2; exit 1 ;;
    esac
    if [[ $PROG == Trauma_Thread.py && ${#EXTRA[@]} -gt 0 ]]; then   # switches combine; one that does not apply is ignored
        echo "warning: ${EXTRA[*]} are Audio_Thread switches - ignored with --program Trauma_Thread.py" >&2; EXTRA=()
    fi
    case "$DRIVER" in fcbbbb|manipulator) ;; *) echo "error: --driver fcbbbb|manipulator (non-ros = fcbbbb)" >&2; exit 1 ;; esac
    [[ -x "$TELEOPS_DIR/humble.sh" ]] || { echo "error: $TELEOPS_DIR/humble.sh not found" >&2; exit 1; }
    ensure_cdi || exit 1
    running $ROVER || echo "warning: $ROVER is not running -> no MAVROS, GUIDED/MANUAL are never seen (./rover.sh up rover && ./rover.sh rec)" >&2
    CAM=(); SMOKE=0
    for x in "${PASS[@]}"; do case "$x" in
        --smoke|--smoke_ros|--smoke-ros|--smoke_mic|--smoke-mic|--trauma-smoke*) SMOKE=1 ;;
        --audio-smoke*) [[ $PROG == Trauma_Thread.py ]] || SMOKE=1 ;;     # ignored with --program Trauma_Thread.py
    esac; done
    if [[ $SMOKE == 1 ]]; then
        CAMERA=none                             # smoke tests use no camera (Teleops.sh --smoke_ros refuses other flags)
        [[ " ${PASS[*]} " == *" --smoke-ros "* ]] && PASS=("${PASS[@]/#--smoke-ros/--smoke_ros}")
    elif [[ -n $FILE ]]; then
        # relative to here, else to ~/teleops (where the launchers want it)
        if [[ -f $FILE ]]; then FILE="$(realpath "$FILE")"; else FILE="$TELEOPS_DIR/${FILE#./}"; fi
        CAM=(--file "$FILE"); CAMERA="file $FILE"
    else case "$CAMERA" in
        ros)
            if ! running $FCB; then
                start_fcb || exit 1
            fi
            read -r w r <<<"$(fcb_jpeg)"
            if [[ "$w" != 0 && "$w" -lt 1280 ]] || awk "BEGIN{exit !($r < 10)}"; then
                echo "warning: fcb_jazzy streams JPEG at width ${w} / ${r} Hz (started with older settings);" >&2
                echo "         the pipeline wants full width at >= 10 Hz (motor tuned at 15). Restarting the" >&2
                echo "         driver fixes it but the Oppila board then needs a 12 V power cycle." >&2
            fi
            CAM=(--fcbros) ;;
        direct|fcb)
            if running $FCB; then
                echo "error: fcb_jazzy holds the camera. The default camera (direct) opens it from the pipeline itself:" >&2
                echo "       ./rover.sh down fcb first. On the Oppila board, closing the stream needs a" >&2
                echo "       12 V power cycle of the camera before the pipeline gets video." >&2
                echo "       Or keep fcb_jazzy and read its stream: add --fcbros." >&2
                exit 1
            fi
            others=$(other_camera_users)
            if [[ -n "$others" ]]; then
                echo "error: another program holds the camera: $(camera_pid_names "$(cut -d' ' -f1 <<<"$others")")" >&2; exit 1
            fi ;;
        *)  echo "error: --camera ros|direct" >&2; exit 1 ;;
    esac; fi
    export TELEOPS_ENGINE=podman ROS_NET="${ROS_NET:-local}" ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    if [[ -z $FILE && $SMOKE == 0 && $CAMERA != ros && $DRIVER == manipulator ]]; then
        # the wrappers only open the FCB with fcbbbb: fcb_manipulator's driver goes straight in
        echo "teleops: $PROG, camera direct, driver manipulator (without the wrappers' checks)"
        exec "$TELEOPS_DIR/humble.sh" run "$PROG" --source fcb --fallback-source "" \
            --fcb-driver manipulator "${EXTRA[@]}" "${PASS[@]}"
    fi
    if [[ $PROG == Trauma_Thread.py ]]; then
        RUN=Trauma_Thread.sh                    # the --trauma-* names mean its own modes here
        for k in "${!PASS[@]}"; do
            case "${PASS[$k]}" in
                --trauma-smoke) PASS[$k]=--smoke ;;   --trauma-smoke-ros) PASS[$k]=--smoke_ros ;;
                --trauma-bypass) PASS[$k]=--bypass ;; --trauma-test) PASS[$k]=--test ;;
                --audio-*) echo "warning: ${PASS[$k]} is an Audio_Thread test - ignored with --program Trauma_Thread.py" >&2; PASS[$k]="" ;;
            esac
        done
    else
        RUN=Teleops.sh                          # rover mode, --smoke_ros, and routes every single-thread test
    fi
    P2=(); for x in "${PASS[@]}"; do [[ -n $x ]] && P2+=("$x"); done; PASS=("${P2[@]}")   # dropped (ignored) flags out
    [[ -x "$TELEOPS_DIR/$RUN" ]] || { echo "error: $TELEOPS_DIR/$RUN not found" >&2; exit 1; }
    [[ " ${PASS[*]} " == *" --debug "* ]] && echo "teleops: $RUN, camera $CAMERA$([[ $CAMERA == ros ]] && echo " (fcb_jazzy topic)")$([[ $CAMERA == direct || $CAMERA == fcb ]] && echo " (fcbbbb, no ROS)")$([[ " ${EXTRA[*]} " == *" --skip-audio "* ]] && echo ", audio task SKIPPED")$([[ " ${EXTRA[*]} " == *" --trauma "* ]] && echo ", trauma FORCED")"
    exec "$TELEOPS_DIR/$RUN" "${CAM[@]}" "${EXTRA[@]}" "${PASS[@]}" ;;
*)
    f="$(readlink -f "${BASH_SOURCE[0]}")"         # the header: line 2 up to the line before "set -u"
    sed -n "2,$(( $(grep -n '^set -u$' "$f" | head -1 | cut -d: -f1) - 1 ))p" "$f" | sed 's/^# \?//'
    exit 1 ;;
esac
