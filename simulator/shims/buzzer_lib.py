#!/usr/bin/env python3
"""Simulator buzzer shim.

Provides a compatible API without ROS/rclpy and plays short local beeps where possible.
"""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import time
import wave
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

_SEMITONES: Dict[str, int] = {
    "C": 0,
    "C#": 1,
    "DB": 1,
    "D": 2,
    "D#": 3,
    "EB": 3,
    "E": 4,
    "F": 5,
    "F#": 6,
    "GB": 6,
    "G": 7,
    "G#": 8,
    "AB": 8,
    "A": 9,
    "A#": 10,
    "BB": 10,
    "B": 11,
}

DEFAULT_BPM = int(os.getenv("BUZZER_DEFAULT_BPM", "120"))
POST_NOTE_GAP_S = float(os.getenv("BUZZER_POST_NOTE_GAP_S", "0.01"))
MUSIC_TIME_SCALE = float(os.getenv("SIM_MUSIC_TIME_SCALE", "0.6667"))  # 50% faster (1.5x speed)


def note_to_freq(note: str) -> int:
    token = str(note).strip().upper()
    if token in ("R", "REST", "PAUSE", "SILENCE"):
        return 0
    if len(token) < 2:
        raise ValueError(f"Invalid note: {note}")
    if token[1] in ("#", "B"):
        key = token[:2]
        octave_txt = token[2:]
    else:
        key = token[:1]
        octave_txt = token[1:]
    if key not in _SEMITONES or not octave_txt:
        raise ValueError(f"Invalid note: {note}")
    octave = int(octave_txt)
    midi = (octave + 1) * 12 + _SEMITONES[key]
    hz = 440.0 * (2.0 ** ((midi - 69) / 12.0))
    return int(round(max(1.0, hz)))


def _write_beep_wav(freq: int, duration_s: float) -> str:
    sr = 22050
    n = max(1, int(sr * max(0.02, float(duration_s))))
    path = os.path.join(tempfile.gettempdir(), f"sim-beep-{freq}-{int(duration_s*1000)}.wav")
    amp = 12000
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        frames = bytearray()
        for i in range(n):
            s = int(amp * math.sin(2 * math.pi * float(freq) * (i / sr)))
            frames += int(s).to_bytes(2, byteorder="little", signed=True)
        w.writeframes(bytes(frames))
    return path


def _play_wav(path: str):
    sysname = os.uname().sysname.lower() if hasattr(os, "uname") else ""
    if sysname == "darwin":
        return subprocess.Popen(["afplay", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    elif sysname == "linux":
        return subprocess.Popen(["aplay", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        # best-effort fallback
        return subprocess.Popen(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


@dataclass
class Buzzer:
    topic: str = "/sim/buzzer"

    def off(self) -> None:
        pass

    def beep(self, freq: int = 2000, duration_s: float = 0.2, gap_s: float = POST_NOTE_GAP_S) -> None:
        if int(freq) <= 0:
            return
        dur = max(0.02, float(duration_s))
        t0 = time.time()
        try:
            p = _write_beep_wav(int(freq), dur)
            proc = _play_wav(p)
            if proc is not None:
                proc.wait(timeout=max(0.1, dur + 0.5))
        except Exception:
            # Never crash a lesson due to missing local audio tool.
            print(f"[sim buzzer] beep {freq}Hz {dur:.2f}s")
        elapsed = time.time() - t0
        remain = dur - elapsed
        if remain > 0:
            time.sleep(remain)
        if float(gap_s) > 0:
            time.sleep(float(gap_s))

    def play_note(self, note: str, beats: float = 1.0, bpm: int = DEFAULT_BPM) -> None:
        beat_s = 60.0 / float(max(1, int(bpm)))
        duration_s = float(beats) * beat_s * MUSIC_TIME_SCALE
        freq = note_to_freq(note)
        if freq > 0:
            self.beep(freq=freq, duration_s=duration_s, gap_s=POST_NOTE_GAP_S * MUSIC_TIME_SCALE)
        else:
            time.sleep(max(0.0, duration_s))

    def play_notes(self, score: str, bpm: int = DEFAULT_BPM) -> None:
        tokens = [t for t in str(score).split() if t.strip()]
        for token in tokens:
            if ":" in token:
                n, beats_txt = token.split(":", 1)
                beats = float(beats_txt)
            else:
                n, beats = token, 1.0
            self.play_note(n, beats=beats, bpm=bpm)

    def play_melody(self, notes: List[Tuple[str, float]], bpm: int = DEFAULT_BPM) -> None:
        for note, beats in notes:
            self.play_note(note, beats=float(beats), bpm=bpm)

    def mario_fragment(self) -> None:
        self.play_melody(
            [
                ("E5", 0.5), ("E5", 0.5), ("R", 0.5), ("E5", 0.5),
                ("R", 0.5), ("C5", 0.5), ("E5", 0.5), ("R", 0.5),
                ("G5", 1.0), ("R", 1.0), ("G4", 1.0),
            ],
            bpm=180,
        )


_BUZZER_SINGLETON: Optional[Buzzer] = None


def get_buzzer(topic: str = "/sim/buzzer") -> Buzzer:
    global _BUZZER_SINGLETON
    if _BUZZER_SINGLETON is None:
        _BUZZER_SINGLETON = Buzzer(topic=topic)
    return _BUZZER_SINGLETON
