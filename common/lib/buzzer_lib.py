#!/usr/bin/env python3
"""
buzzer_lib.py
Play tones and simple melodies on TurboPi buzzer via ROS2.

Examples:
    import buzzer_lib
    bz = buzzer_lib.get_buzzer()
    bz.beep(2000, 0.2)
    bz.play_notes("C4:1 D4:1 E4:2 R:1 C5:2", bpm=120)
    bz.play_notes_music_mode("C4:1 D4:1 E4:2", bpm=120)
"""
__version__ = "1.1.0"


import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from ros_robot_controller_msgs.msg import BuzzerState

DEFAULT_TOPIC = os.getenv("BUZZER_TOPIC", "/ros_robot_controller/set_buzzer")
FLUSH_SPIN_S = float(os.getenv("BUZZER_FLUSH_SPIN_S", "0.08"))
POST_NOTE_GAP_S = float(os.getenv("BUZZER_POST_NOTE_GAP_S", "0.01"))
DEFAULT_BPM = int(os.getenv("BUZZER_DEFAULT_BPM", "120"))
BUZZER_SUSTAIN_CHUNK_S = float(os.getenv("BUZZER_SUSTAIN_CHUNK_S", "0.08"))
BUZZER_MIN_FREQ = int(os.getenv("BUZZER_MIN_FREQ", "2000"))
BUZZER_MAX_FREQ = int(os.getenv("BUZZER_MAX_FREQ", "4000"))

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


def _qos_rel(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _rclpy_init_once() -> None:
    if not rclpy.ok():
        rclpy.init(args=None)


def note_to_freq(note: str) -> int:
    """
    Convert note name to frequency (Hz).
    Supported: C4, C#4, Db4, A4 ... B8, plus R/REST for silence (0).
    """
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

    if key not in _SEMITONES:
        raise ValueError(f"Invalid note key: {note}")
    if octave_txt == "":
        raise ValueError(f"Missing octave in note: {note}")

    octave = int(octave_txt)
    midi = (octave + 1) * 12 + _SEMITONES[key]
    hz = 440.0 * (2.0 ** ((midi - 69) / 12.0))
    return int(round(max(1.0, hz)))


def note_to_buzzer_freq(note: str, min_freq: int = BUZZER_MIN_FREQ, max_freq: int = BUZZER_MAX_FREQ) -> int:
    """
    Convert a musical note to a buzzer-safe frequency.
    The TurboPi buzzer behaves more reliably in a high-frequency band, so notes
    are transposed upward by octaves until they fit inside the supported range.
    """
    hz = note_to_freq(note)
    if hz <= 0:
        return 0

    lo = max(1, int(min_freq))
    hi = max(lo, int(max_freq))

    while hz < lo:
        hz *= 2
    while hz > hi and hz > 1:
        hz /= 2.0

    # Final guard in case the configured band cannot be reached exactly.
    hz = max(lo, min(hi, hz))
    return int(round(hz))


class Buzzer(Node):
    """
    ROS2 buzzer helper.
    The message fields vary between some images, so assignment is tolerant.
    """

    def __init__(self, topic: str = DEFAULT_TOPIC, node_name: str = "buzzer_client"):
        _rclpy_init_once()
        uniq = f"{node_name}_{os.getpid()}_{int(time.time() * 1000) % 100000}"
        super().__init__(uniq)
        self.topic = topic
        self.pub = self.create_publisher(BuzzerState, self.topic, _qos_rel())
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        print(f"buzzer_lib ready -> topic={self.topic}")

    def _flush(self, seconds: float = FLUSH_SPIN_S) -> None:
        end = time.time() + float(seconds)
        while time.time() < end:
            self._exec.spin_once(timeout_sec=0.02)

    @staticmethod
    def _set_if_exists(msg: BuzzerState, names: Tuple[str, ...], value) -> bool:
        for name in names:
            if hasattr(msg, name):
                setattr(msg, name, value)
                return True
        return False

    def _publish_buzzer(
        self,
        freq: int,
        on_time_s: float,
        off_time_s: float,
        repeat: int = 1,
    ) -> None:
        msg = BuzzerState()
        set_f = self._set_if_exists(msg, ("freq", "frequency", "hz"), int(freq))
        set_on = self._set_if_exists(msg, ("on_time", "on", "duration"), float(on_time_s))
        set_off = self._set_if_exists(msg, ("off_time", "off"), float(off_time_s))
        set_r = self._set_if_exists(msg, ("repeat", "times", "count"), int(repeat))

        if not (set_f and set_on and set_off and set_r):
            fields = list(getattr(msg, "get_fields_and_field_types", lambda: {})().keys())
            raise RuntimeError(
                "Unsupported BuzzerState fields on this image. "
                f"Found fields: {fields}"
            )

        self.pub.publish(msg)
        self._flush()

    def off(self) -> None:
        # Some controller images are picky about zero-duration messages.
        # Try a short explicit silent command, then a zeroed command.
        try:
            self._publish_buzzer(freq=0, on_time_s=0.02, off_time_s=0.0, repeat=1)
        except Exception:
            pass
        try:
            self._publish_buzzer(freq=0, on_time_s=0.0, off_time_s=0.0, repeat=1)
        except Exception:
            pass

    def beep(
        self,
        freq: int = 2000,
        duration_s: float = 0.2,
        gap_s: float = POST_NOTE_GAP_S,
        note: str | None = None,
    ) -> None:
        if note is not None:
            freq = note_to_freq(note)
        duration_s = max(0.0, float(duration_s))
        if duration_s <= 0:
            return
        if int(freq) <= 0:
            time.sleep(duration_s)
            if gap_s > 0:
                time.sleep(float(gap_s))
            return
        # Some controller images ignore long on_time values. Sustain the tone by
        # reissuing short chunks until the requested duration has elapsed.
        remain_s = duration_s
        chunk_s = max(0.02, float(BUZZER_SUSTAIN_CHUNK_S))
        while remain_s > 0:
            on_s = min(chunk_s, remain_s)
            self._publish_buzzer(freq=int(freq), on_time_s=on_s, off_time_s=0.0, repeat=1)
            time.sleep(on_s)
            remain_s -= on_s
        self.off()
        if gap_s > 0:
            time.sleep(float(gap_s))

    def play_note(self, note: str, beats: float = 1.0, bpm: int = DEFAULT_BPM) -> None:
        beat_s = 60.0 / float(max(1, int(bpm)))
        total_s = max(0.0, float(beats) * beat_s)
        freq = note_to_freq(note)
        if freq <= 0:
            time.sleep(total_s)
            return
        # Make the total note time match beats * beat_s.
        # The release gap is carved out of the beat instead of added on top.
        gap_s = min(float(POST_NOTE_GAP_S), total_s * 0.25)
        on_s = max(0.0, total_s - gap_s)
        self.beep(freq=freq, duration_s=on_s, gap_s=gap_s)

    def play_notes(self, score: str, bpm: int = DEFAULT_BPM) -> None:
        """
        score format:
            "C4:1 D4:1 E4:2 R:1 G4:0.5"
        Each token is NOTE:BEATS.
        """
        tokens = [t for t in str(score).split() if t.strip()]
        try:
            for token in tokens:
                if ":" in token:
                    n, beats_txt = token.split(":", 1)
                    beats = float(beats_txt)
                else:
                    n, beats = token, 1.0
                self.play_note(n, beats=beats, bpm=bpm)
        finally:
            self.off()

    def play_note_music_mode(self, note: str, beats: float = 1.0, bpm: int = DEFAULT_BPM) -> None:
        """
        Music-mode playback that transposes notes into the buzzer-friendly band.
        Use this when the physical buzzer cannot reproduce lower octaves reliably.
        """
        beat_s = 60.0 / float(max(1, int(bpm)))
        total_s = max(0.0, float(beats) * beat_s)
        freq = note_to_buzzer_freq(note)
        if freq <= 0:
            time.sleep(total_s)
            return
        gap_s = min(float(POST_NOTE_GAP_S), total_s * 0.25)
        on_s = max(0.0, total_s - gap_s)
        self.beep(freq=freq, duration_s=on_s, gap_s=gap_s)

    def play_notes_music_mode(self, score: str, bpm: int = DEFAULT_BPM) -> None:
        tokens = [t for t in str(score).split() if t.strip()]
        try:
            for token in tokens:
                if ":" in token:
                    n, beats_txt = token.split(":", 1)
                    beats = float(beats_txt)
                else:
                    n, beats = token, 1.0
                self.play_note_music_mode(n, beats=beats, bpm=bpm)
        finally:
            self.off()

    def play_melody(self, notes: List[Tuple[str, float]], bpm: int = DEFAULT_BPM) -> None:
        try:
            for note, beats in notes:
                self.play_note(note, beats=float(beats), bpm=bpm)
        finally:
            self.off()

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


def get_buzzer(topic: str = DEFAULT_TOPIC) -> Buzzer:
    global _BUZZER_SINGLETON
    if _BUZZER_SINGLETON is None:
        _BUZZER_SINGLETON = Buzzer(topic=topic)
    return _BUZZER_SINGLETON
