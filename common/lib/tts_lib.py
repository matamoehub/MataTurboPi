# tts_lib.py
# Piper TTS helper for Matamoe robots (Python 3.10)
#
# Features:
# - Synth text -> wav (cached by hash)
# - Play wav via aplay
# - Simple say() helper (blocking or async)
# - Optional queued TTS scheduler (get_tts_queue())

import os
import time
import queue
import threading
import subprocess
import shutil
import hashlib
import tempfile
import wave
import contextlib
from typing import Optional

# Allow override so it works no matter which user runs Jupyter/services
VOICE_DIR = os.environ.get("PIPER_VOICE_DIR", os.path.expanduser("/opt/robot/piper/voices"))

VOICE_MAP = {
    "ryan": ("en_US-ryan-high.onnx", "en_US-ryan-high.onnx.json"),
    "amy":  ("en_GB-amy-medium.onnx", "en_GB-amy-medium.onnx.json"),
    "aru":  ("en_GB-aru-medium.onnx", "en_GB-aru-medium.onnx.json"),
    # Add more voices you download into VOICE_DIR
}

DEFAULT_VOICE = "ryan"
PIPER_VOICE_ENV = "PIPER_VOICE"


def _resolve_voice(voice: Optional[str]) -> str:
    """
    Voice precedence:
    1) explicit function arg
    2) PIPER_VOICE env
    3) DEFAULT_VOICE
    """
    requested = (voice or os.environ.get(PIPER_VOICE_ENV) or DEFAULT_VOICE).strip().lower()
    if requested in VOICE_MAP:
        return requested
    return DEFAULT_VOICE


def _safe_workdir() -> str:
    """
    Piper CLI calls Path.cwd() internally; if notebook cwd is gone it crashes.
    Always run subprocesses from a directory that exists.
    """
    try:
        cwd = os.getcwd()
    except Exception:
        cwd = None

    for cand in (cwd, os.environ.get("HOME"), "/tmp"):
        if cand and os.path.isdir(cand):
            return cand
    return "/tmp"


def _voice_paths(name: Optional[str]):
    name = _resolve_voice(name)
    model_file, config_file = VOICE_MAP[name]
    model = os.path.join(VOICE_DIR, model_file)
    config = os.path.join(VOICE_DIR, config_file)
    if not (os.path.isfile(model) and os.path.isfile(config)):
        raise FileNotFoundError(
            f"Voice files missing for '{name}':\n  {model}\n  {config}\n"
            f"Tip: set PIPER_VOICE_DIR to the folder containing your .onnx + .json"
        )
    return model, config


def _require_aplay():
    if not shutil.which("aplay"):
        raise RuntimeError(
            "aplay not found. Install: sudo apt update && sudo apt install -y alsa-utils"
        )


def synth_to_wav(
    text: str,
    voice: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.12",
    out_path: str = "/tmp/piper_line.wav",
) -> str:
    """
    Synthesize speech to a wav file using piper.
    """
    model, config = _voice_paths(voice)

    proc = subprocess.run(
        [
            "piper",
            "-m",
            model,
            "-c",
            config,
            "--length-scale",
            str(length_scale),
            "--sentence-silence",
            str(sentence_silence),
            "-f",
            out_path,
        ],
        input=(text.strip() + "\n").encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=_safe_workdir(),
    )

    if proc.returncode != 0:
        raise RuntimeError("Piper failed:\n" + proc.stderr.decode("utf-8", "ignore"))

    return out_path


def pre_synth(
    text: str,
    voice: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.10",
) -> str:
    """
    Cached synth: writes a deterministic wav in /tmp based on content+settings.
    """
    voice_name = _resolve_voice(voice)
    key = f"{voice_name}|{length_scale}|{sentence_silence}|{text}".encode("utf-8")
    h = hashlib.sha1(key).hexdigest()[:16]
    out_path = os.path.join(tempfile.gettempdir(), f"piper-{h}.wav")
    if not os.path.exists(out_path):
        synth_to_wav(
            text,
            voice=voice_name,
            length_scale=length_scale,
            sentence_silence=sentence_silence,
            out_path=out_path,
        )
    return out_path


def wav_duration_seconds(path: str) -> float:
    with contextlib.closing(wave.open(path, "rb")) as w:
        return w.getnframes() / float(w.getframerate())


def play_wav_async(path: str, device: Optional[str] = None):
    """
    Play wav using aplay (returns subprocess.Popen).
    """
    _require_aplay()
    cmd = ["aplay"]
    if device:
        cmd += ["-D", device]
    cmd += [path]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def play_path_async(path: str, device: Optional[str] = None):
    return play_wav_async(path, device=device)


def warm_piper(voice: Optional[str] = None):
    """
    Warms piper model cache / initial startup cost.
    """
    try:
        _ = pre_synth("...", voice=voice, length_scale="1.00", sentence_silence="0.10")
    except Exception as e:
        print("[warm_piper] warn:", e)


def say(
    text: str,
    voice: Optional[str] = None,
    device: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.12",
    block: bool = True,
) -> str:
    """
    One-liner for lessons/tests.
    Returns the wav path.
    """
    path = pre_synth(
        text,
        voice=voice,
        length_scale=length_scale,
        sentence_silence=sentence_silence,
    )
    p = play_path_async(path, device=device)
    if block:
        p.wait()
    return path


# -------- Async TTS Queue --------

class TTSJob:
    def __init__(
        self,
        text: str,
        when: float,
        voice: str,
        length_scale: str,
        sentence_silence: str,
        device: Optional[str],
        block: bool,
    ):
        self.text = text
        self.when = float(when)
        self.voice = voice
        self.length_scale = length_scale
        self.sentence_silence = sentence_silence
        self.device = device
        self.block = block


class TTSQueue:
    def __init__(
        self,
        default_voice: Optional[str] = None,
        default_length: str = "1.00",
        default_sil: str = "0.12",
        device: Optional[str] = None,
    ):
        self.default_voice = _resolve_voice(default_voice)
        self.default_length = default_length
        self.default_sil = default_sil
        self.device = device

        self.q = queue.PriorityQueue()
        self._idx = 0
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def schedule(
        self,
        text: str,
        delay_s: float = 0.0,
        voice: Optional[str] = None,
        length_scale: Optional[str] = None,
        sentence_silence: Optional[str] = None,
        block: bool = True,
    ):
        t = time.time() + float(delay_s)
        job = TTSJob(
            text=text,
            when=t,
            voice=voice or self.default_voice,
            length_scale=length_scale or self.default_length,
            sentence_silence=sentence_silence or self.default_sil,
            device=self.device,
            block=block,
        )
        self.q.put((t, self._idx, job))
        self._idx += 1

    def _run(self):
        while not self._stop.is_set():
            try:
                when, _, job = self.q.get(timeout=0.1)
            except queue.Empty:
                continue

            dt = when - time.time()
            if dt > 0:
                time.sleep(dt)

            try:
                path = pre_synth(
                    job.text,
                    voice=job.voice,
                    length_scale=job.length_scale,
                    sentence_silence=job.sentence_silence,
                )
                p = play_path_async(path, device=job.device)
                if job.block:
                    p.wait()
            except Exception as e:
                print("[TTSQueue] error:", e)

    def stop(self):
        self._stop.set()
        self._thr.join(timeout=1.0)


# Singleton queue (created on demand)
_ttsq: Optional[TTSQueue] = None

def get_tts_queue() -> TTSQueue:
    global _ttsq
    if _ttsq is None:
        _ttsq = TTSQueue(default_voice=None, default_length="0.98", default_sil="0.08", device=None)
    return _ttsq
