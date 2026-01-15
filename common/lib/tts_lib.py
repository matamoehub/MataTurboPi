# tts_lib.py
import os, time, queue, threading, subprocess, shutil, hashlib, tempfile, wave, contextlib
from typing import Optional

VOICE_DIR = os.path.expanduser("~/.local/share/piper/voices")
VOICE_MAP = {
    "ryan":   ("en_US-ryan-high.onnx",  "en_US-ryan-high.onnx.json"),
    "amy":    ("en_GB-amy-medium.onnx", "en_GB-amy-medium.onnx.json"),
    # add others as you download them
}
DEFAULT_VOICE = "ryan"
WAV_PATH      = "/tmp/piper_line.wav"

def _voice_paths(name: str):
    if name not in VOICE_MAP:
        name = DEFAULT_VOICE
    m, c = VOICE_MAP[name]
    model  = os.path.join(VOICE_DIR, m)
    config = os.path.join(VOICE_DIR, c)
    if not (os.path.isfile(model) and os.path.isfile(config)):
        raise FileNotFoundError(f"Voice files missing for '{name}':\n  {model}\n  {config}")
    return model, config

def synth_to_wav(text: str, voice: str = DEFAULT_VOICE,
                 length_scale: str = "1.00", sentence_silence: str = "0.12",
                 out_path: str = WAV_PATH) -> str:
    model, config = _voice_paths(voice)
    proc = subprocess.run(
        ["piper", "-m", model, "-c", config,
         "--length-scale", str(length_scale),
         "--sentence-silence", str(sentence_silence),
         "-f", out_path],
        input=(text.strip() + "\n").encode("utf-8"),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if proc.returncode != 0:
        raise RuntimeError("Piper failed:\n" + proc.stderr.decode("utf-8", "ignore"))
    return out_path

def play_wav_async(path: str, device: Optional[str] = None):
    if not shutil.which("aplay"):
        raise RuntimeError("aplay not found. Install: sudo apt update && sudo apt install -y alsa-utils")
    cmd = ["aplay"]
    if device:
        cmd += ["-D", device]
    cmd += [path]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ---- NEW: pre-synth cache + duration + play-by-path (what your notebook calls) ----
def pre_synth(text: str, voice: str = DEFAULT_VOICE,
              length_scale: str = "1.00", sentence_silence: str = "0.10") -> str:
    key = f"{voice}|{length_scale}|{sentence_silence}|{text}".encode("utf-8")
    h = hashlib.sha1(key).hexdigest()[:16]
    out_path = os.path.join(tempfile.gettempdir(), f"piper-{h}.wav")
    if not os.path.exists(out_path):
        synth_to_wav(text, voice=voice, length_scale=length_scale,
                     sentence_silence=sentence_silence, out_path=out_path)
    return out_path

def wav_duration_seconds(path: str) -> float:
    with contextlib.closing(wave.open(path, 'rb')) as w:
        return w.getnframes() / float(w.getframerate())

def play_path_async(path: str, device: Optional[str] = None):
    return play_wav_async(path, device=device)

def warm_piper(voice: str = DEFAULT_VOICE):
    try:
        _ = pre_synth("...", voice=voice, length_scale="1.00", sentence_silence="0.10")
    except Exception as e:
        print("[warm_piper] warn:", e)

# -------- Async TTS Queue --------
class TTSJob:
    def __init__(self, text: str, when: float, voice: str, length_scale: str, sentence_silence: str, device: Optional[str]):
        self.text = text; self.when = float(when)
        self.voice = voice; self.length_scale = length_scale
        self.sentence_silence = sentence_silence; self.device = device

class TTSQueue:
    def __init__(self, default_voice=DEFAULT_VOICE, default_length="1.00", default_sil="0.12", device: Optional[str]=None):
        self.default_voice = default_voice; self.default_length = default_length
        self.default_sil = default_sil; self.device = device
        self.q = queue.PriorityQueue()
        self._idx = 0; self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True); self._thr.start()

    def schedule(self, text: str, delay_s: float = 0.0, voice: Optional[str]=None,
                 length_scale: Optional[str]=None, sentence_silence: Optional[str]=None):
        t = time.time() + float(delay_s)
        job = TTSJob(
            text=text, when=t,
            voice=voice or self.default_voice,
            length_scale=length_scale or self.default_length,
            sentence_silence=sentence_silence or self.default_sil,
            device=self.device
        )
        self.q.put((t, self._idx, job)); self._idx += 1

    def _run(self):
        while not self._stop.is_set():
            try:
                when, _, job = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            dt = when - time.time()
            if dt > 0: time.sleep(dt)
            try:
                path = pre_synth(job.text, voice=job.voice,
                                 length_scale=job.length_scale,
                                 sentence_silence=job.sentence_silence)
                p = play_path_async(path, device=job.device)
                p.wait()
            except Exception as e:
                print("[TTSQueue] error:", e)

    def stop(self):
        self._stop.set(); self._thr.join(timeout=1.0)

# Notebook singleton (optional)
try:
    ttsq
except NameError:
    ttsq = TTSQueue(default_voice="ryan", default_length="0.98", default_sil="0.08", device=None)
