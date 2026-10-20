#!/usr/bin/env bash
# Casualty models on the Orin HOST (outside the rover container), in tmux session "models":
#   pane 0  FCB camera driver        ~/frame_extraction_final/start_camera.sh   (CAMERA=0 to skip,
#                                     e.g. when ~/fcb_orin_ros/fcb_ros.sh already owns the camera)
#   pane 1  frame extractor          ~/frame_extraction_final/launch.sh, TRIGGER=http (:8090)
#   pane 2  audio triage service     vortex_mission/host/audio_service.py (:8091)
# The rover container (lirovo5 -> casualty_models_bridge) drives both over localhost.
#
#   QUESTION_WAV=~/triage_question.wav APLAY_DEVICE=plughw:1,0 ./start_models.sh
#   SAPIENS=0 ./start_models.sh        extractor with YOLOE only
#   tmux attach -t models              watch;  tmux kill-session -t models   stop
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SESSION="${SESSION:-models}"
FRAME_DIR="${FRAME_DIR:-$HOME/frame_extraction_final}"
PIPELINE_DIR="${PIPELINE_DIR:-$HOME/pipeline_v1_standalone}"
QUESTION_WAV="${QUESTION_WAV:-$HOME/triage_question.wav}"
APLAY_DEVICE="${APLAY_DEVICE:-}"
CAMERA="${CAMERA:-1}"
SAPIENS="${SAPIENS:-1}"

for f in "$FRAME_DIR/launch.sh" "$PIPELINE_DIR/audioPipe.py"; do
  [ -f "$f" ] || { echo "!! missing $f" >&2; exit 1; }
done
[ -f "$QUESTION_WAV" ] || echo "!! question wav $QUESTION_WAV not found: the casualty will not be asked"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n main
tmux set-option -t "$SESSION" remain-on-exit on
tmux split-window -h -t "$SESSION:main"
tmux split-window -v -t "$SESSION:main.1"

if [ "$CAMERA" = 1 ]; then
  tmux send-keys -t "$SESSION:main.0" "cd '$FRAME_DIR' && ./start_camera.sh" Enter
else
  tmux send-keys -t "$SESSION:main.0" "echo '[camera pane: CAMERA=0, using an already running driver]'" Enter
fi
# give the camera a moment; launch.sh waits up to 15 s for the first image anyway
tmux send-keys -t "$SESSION:main.1" "sleep 3; cd '$FRAME_DIR' && TRIGGER=http SAPIENS=$SAPIENS ./launch.sh" Enter
tmux send-keys -t "$SESSION:main.2" "python3 '$HERE/audio_service.py' --pipeline-dir '$PIPELINE_DIR' \
--question-wav '$QUESTION_WAV' ${APLAY_DEVICE:+--aplay-device '$APLAY_DEVICE'}" Enter

echo "models started in tmux session '$SESSION' (tmux attach -t $SESSION)"
echo "check:  curl -s localhost:8090/status ; curl -s localhost:8091/status"
