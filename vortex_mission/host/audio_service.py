#!/usr/bin/env python3
"""Audio triage service for the rover: plays the triage question, then runs the audio team's
pipeline_v1_standalone/audioPipe.py (records 10 s from the Pico array, WPE -> GSC -> SepFormer
-> MVDR -> Whisper -> Qwen) and serves the result. Runs on the Orin HOST; stdlib only, no ROS.
The audio team's folder is used as-is (audioPipe.py is run as a subprocess, so its models are
loaded only while a job runs and the GPU memory is freed afterwards).

  python3 audio_service.py --question-wav ~/triage_question.wav
  python3 audio_service.py --question-wav q.wav --audio-file take.wav   # bench: no Pico

HTTP (localhost:8091; casualty_models_bridge in the rover container calls it):
  POST /run     {"casualty_id": 3}   start a job (409 if one is running)
  GET  /status                       {"state": idle|playing|recording|done|failed|cancelled,
                                      "casualty_id", "triage", "transcript", "out_dir", ...}
  POST /cancel                       stop the running job (kills the player / audioPipe)
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class AudioJobs:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.proc = None
        self.cancelled = False
        self.status = {'state': 'idle'}

    def busy(self):
        return self.status.get('state') in ('playing', 'recording')

    def start(self, casualty_id):
        with self.lock:
            if self.busy():
                return False, f'busy with casualty {self.status.get("casualty_id")}'
            out_dir = Path(self.args.runs_dir).expanduser() / \
                f'cas{casualty_id}_{time.strftime("%Y%m%d-%H%M%S")}'
            self.cancelled = False
            self.status = {'state': 'playing', 'casualty_id': casualty_id, 'out_dir': str(out_dir),
                           't_start': time.time(), 'question_played': False}
        threading.Thread(target=self._run, args=(casualty_id, out_dir), daemon=True).start()
        return True, str(out_dir)

    def cancel(self):
        with self.lock:
            self.cancelled = True
            p = self.proc
        if p and p.poll() is None:
            try:                                  # whole process group: audioPipe + children
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        return True

    def _set(self, **kw):
        with self.lock:
            self.status.update(kw)

    def _call(self, cmd, cwd=None, log=None):
        """Run one subprocess (cancellable); returns its exit code."""
        with self.lock:
            if self.cancelled:
                return -1
            self.proc = subprocess.Popen(cmd, cwd=cwd, stdout=log or subprocess.DEVNULL,
                                         stderr=subprocess.STDOUT, start_new_session=True)
            p = self.proc
        rc = p.wait()
        with self.lock:
            self.proc = None
        return rc

    def _run(self, cid, out_dir):
        a = self.args
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f'[audio] casualty {cid}: job -> {out_dir}', flush=True)
        # 1. ask the question
        wav = Path(a.question_wav).expanduser() if a.question_wav else None
        if wav and wav.is_file():
            cmd = [a.player] + (['-D', a.aplay_device] if a.aplay_device and a.player == 'aplay'
                                else []) + [str(wav)]
            rc = self._call(cmd)
            self._set(question_played=(rc == 0))
            if rc != 0 and not self.cancelled:
                print(f'[audio] !! playing {wav} failed (exit {rc}); recording anyway', flush=True)
        else:
            print(f'[audio] !! no question wav ({a.question_wav!r}); recording without asking',
                  flush=True)
        if a.pause_after_question > 0 and not self.cancelled:
            time.sleep(a.pause_after_question)
        # 2. record + process (the audio team's pipeline, unchanged)
        self._set(state='recording', t_record=time.time())
        cmd = [a.python, 'audioPipe.py', '--out', str(out_dir)]
        cmd += ['--audio', str(Path(a.audio_file).expanduser())] if a.audio_file \
            else ['--sec', str(a.sec)]
        if a.ignore:
            cmd += ['--ignore'] + [str(i) for i in a.ignore]
        with open(out_dir / 'audioPipe.log', 'w') as log:
            rc = self._call(cmd, cwd=a.pipeline_dir, log=log)
        # 3. collect
        res = {'t_end': time.time(), 'exit_code': rc}
        tri, txt = out_dir / 'triage.json', out_dir / 'transcript.txt'
        if self.cancelled:
            res.update(state='cancelled', error='cancelled')
        elif rc == 0 and tri.is_file():
            try:
                res.update(state='done', triage=json.loads(tri.read_text()),
                           transcript=txt.read_text().strip() if txt.is_file() else '')
            except ValueError as e:
                res.update(state='failed', error=f'bad triage.json: {e}')
        else:
            tail = (out_dir / 'audioPipe.log').read_text(errors='replace').strip().splitlines()[-5:]
            res.update(state='failed', error=f'audioPipe exit {rc}: ' + ' | '.join(tail))
        self._set(**res)
        print(f'[audio] casualty {cid}: {res["state"]} {json.dumps(res.get("triage"))}'
              + (f' ({res.get("error")})' if res.get('error') else ''), flush=True)


class Handler(BaseHTTPRequestHandler):
    jobs: AudioJobs = None

    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.rstrip('/') == '/status':
            with self.jobs.lock:
                return self._json(200, dict(self.jobs.status))
        self._json(404, {'error': 'GET /status only'})

    def do_POST(self):
        path = self.path.rstrip('/')
        try:
            body = json.loads(self.rfile.read(int(self.headers.get('Content-Length') or 0)) or b'{}')
        except ValueError:
            return self._json(400, {'error': 'body must be JSON'})
        if path == '/run':
            ok, info = self.jobs.start(body.get('casualty_id', 0))
            return self._json(200 if ok else 409, {'ok': ok, ('out_dir' if ok else 'error'): info})
        if path == '/cancel':
            return self._json(200, {'ok': self.jobs.cancel()})
        self._json(404, {'error': 'POST /run or /cancel'})


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pipeline-dir', default='~/pipeline_v1_standalone')
    ap.add_argument('--python', default='',
                    help="python for audioPipe.py (default: <pipeline-dir>/.venv/bin/python, else python3)")
    ap.add_argument('--question-wav', default='~/triage_question.wav')
    ap.add_argument('--player', default='aplay', help='program that plays the question wav')
    ap.add_argument('--aplay-device', default='', help="ALSA device, e.g. plughw:1,0 ('' = default)")
    ap.add_argument('--pause-after-question', type=float, default=0.5)
    ap.add_argument('--sec', type=int, default=10, help='seconds to record')
    ap.add_argument('--ignore', type=int, nargs='*', default=[], help='mic indices to leave out')
    ap.add_argument('--audio-file', default='', help='bench test: use this 8-ch wav, no Pico')
    ap.add_argument('--runs-dir', default='~/casualty_audio_runs')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8091)
    a = ap.parse_args()
    a.pipeline_dir = str(Path(a.pipeline_dir).expanduser())
    if not a.python:
        venv = Path(a.pipeline_dir) / '.venv' / 'bin' / 'python'
        a.python = str(venv) if venv.exists() else 'python3'
    if not (Path(a.pipeline_dir) / 'audioPipe.py').is_file():
        raise SystemExit(f'no audioPipe.py in {a.pipeline_dir}')
    if a.player and not shutil.which(a.player):
        print(f'[audio] !! player {a.player!r} not found; the question will not be played',
              flush=True)
    Handler.jobs = AudioJobs(a)
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f'[audio] serving http://{a.host}:{a.port}  (POST /run /cancel, GET /status); '
          f'pipeline {a.pipeline_dir} with {a.python}; question {a.question_wav}', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        Handler.jobs.cancel()
        print('\n[audio] exit', flush=True)


if __name__ == '__main__':
    main()
