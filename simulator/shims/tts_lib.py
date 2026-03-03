"""Simulator TTS shim: speak on the local computer instead of robot hardware.

Implements the same public API shape as common/lib/tts_lib.py used by lessons.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import queue
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Optional

DEFAULT_VOICE = "default"
PIPER_VOICE_ENV = "PIPER_VOICE"


def _safe_text(text: str) -> str:
    return str(text or "").strip()


def _cache_path(text: str, voice: str, length_scale: str, sentence_silence: str) -> str:
    key = f"{voice}|{length_scale}|{sentence_silence}|{text}".encode("utf-8")
    h = hashlib.sha1(key).hexdigest()[:16]
    p = Path(tempfile.gettempdir()) / f"sim-tts-{h}.json"
    return str(p)


def _speak_cmd(text: str, voice: Optional[str] = None):
    sysname = platform.system().lower()
    chosen = (voice or os.environ.get(PIPER_VOICE_ENV) or DEFAULT_VOICE).strip()

    # macOS: built-in 'say'
    if sysname == "darwin" and shutil.which("say"):
        cmd = ["say"]
        if chosen and chosen != "default":
            cmd += ["-v", chosen]
        cmd += [text]
        return cmd

    # Linux: prefer spd-say, then espeak
    if sysname == "linux":
        if shutil.which("spd-say"):
            return ["spd-say", text]
        if shutil.which("espeak"):
            return ["espeak", text]

    # Windows: PowerShell speech synth (usually available)
    if sysname == "windows" and shutil.which("powershell"):
        ps = (
            "Add-Type -AssemblyName System.Speech; "
            "$s=New-Object System.Speech.Synthesis.SpeechSynthesizer; "
            f"$s.Speak('{text.replace("'", "''")}')"
        )
        return ["powershell", "-NoProfile", "-Command", ps]

    # Fallback: no TTS engine
    return None


def _play_text_async(text: str, voice: Optional[str] = None):
    cmd = _speak_cmd(text, voice=voice)
    if not cmd:
        raise RuntimeError(
            "No local TTS engine found. macOS: 'say', Linux: 'spd-say' or 'espeak', Windows: PowerShell speech"
        )
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def synth_to_wav(
    text: str,
    voice: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.12",
    out_path: str = "/tmp/piper_line.wav",
) -> str:
    # Simulator mode does not generate real wav; store metadata at requested path.
    payload = {
        "text": _safe_text(text),
        "voice": voice or DEFAULT_VOICE,
        "length_scale": str(length_scale),
        "sentence_silence": str(sentence_silence),
    }
    path = Path(out_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    return str(path)


def pre_synth(
    text: str,
    voice: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.10",
) -> str:
    text = _safe_text(text)
    voice_name = (voice or os.environ.get(PIPER_VOICE_ENV) or DEFAULT_VOICE).strip() or DEFAULT_VOICE
    out = _cache_path(text, voice_name, str(length_scale), str(sentence_silence))
    if not os.path.exists(out):
        synth_to_wav(
            text,
            voice=voice_name,
            length_scale=length_scale,
            sentence_silence=sentence_silence,
            out_path=out,
        )
    return out


def wav_duration_seconds(path: str) -> float:
    # Approximate from text length if metadata file exists.
    try:
        with open(path, "r", encoding="utf-8") as f:
            obj = json.load(f)
        text = _safe_text(obj.get("text", ""))
        return max(0.3, len(text) / 14.0)
    except Exception:
        return 0.8


def play_path_async(path: str, device: Optional[str] = None):
    _ = device
    with open(path, "r", encoding="utf-8") as f:
        obj = json.load(f)
    text = _safe_text(obj.get("text", ""))
    voice = obj.get("voice", DEFAULT_VOICE)
    return _play_text_async(text, voice=voice)


def play_wav_async(path: str, device: Optional[str] = None):
    return play_path_async(path, device=device)


def warm_piper(voice: Optional[str] = None):
    _ = voice


def say(
    text: str,
    voice: Optional[str] = None,
    device: Optional[str] = None,
    length_scale: str = "1.00",
    sentence_silence: str = "0.12",
    block: bool = True,
) -> str:
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
        self.default_voice = (default_voice or DEFAULT_VOICE).strip() or DEFAULT_VOICE
        self.default_length = str(default_length)
        self.default_sil = str(default_sil)
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


_ttsq: Optional[TTSQueue] = None


def get_tts_queue() -> TTSQueue:
    global _ttsq
    if _ttsq is None:
        _ttsq = TTSQueue(default_voice=None, default_length="0.98", default_sil="0.08", device=None)
    return _ttsq
