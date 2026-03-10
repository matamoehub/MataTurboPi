#!/usr/bin/env python3
"""Student animation helper library for Lesson 2 character work.

Design goals:
- Use robot_moves as the sole movement backend.
- Be safe to import/reload from any notebook cell.
- Keep eyes/camera/tts optional when hardware is unavailable.
"""

from __future__ import annotations

import builtins
import random
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

RGB = Tuple[int, int, int]
DEFAULT_EYE_COLOR: RGB = (0, 255, 120)
DEFAULT_FIDGET_MOVES: Tuple[str, ...] = ("forward", "backward", "left", "right")
_SINGLETON_KEY = "_mataturbopi_student_animation_singleton"
_LOCK_KEY = "_mataturbopi_student_animation_lock"


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def build_fidget_plan(
    duration_s: float,
    step_s: float = 0.05,
    seed: Optional[int] = None,
    moves: Sequence[str] = DEFAULT_FIDGET_MOVES,
) -> List[str]:
    """Deterministic random move plan for testing/replay."""
    step = max(0.01, float(step_s))
    total = max(step, float(duration_s))
    count = max(1, int(round(total / step)))
    pool = tuple(moves) if moves else DEFAULT_FIDGET_MOVES
    rng = random.Random(seed)
    return [rng.choice(pool) for _ in range(count)]


class AnimationLibrary:
    def __init__(
        self,
        robot=None,
        eyes=None,
        camera=None,
        tts=None,
        base_speed: float = 300.0,
        eye_color: RGB = DEFAULT_EYE_COLOR,
        verbose: bool = True,
        sleep_fn: Callable[[float], None] = time.sleep,
    ):
        self.base_speed = float(base_speed)
        self.default_eye_color: RGB = tuple(int(v) for v in eye_color)
        self.voice = "ryan"
        self.phrases: Dict[str, Dict[str, str]] = {}
        self._sleep = sleep_fn
        self.verbose = bool(verbose)

        self.robot = robot
        self.eyes = eyes
        self.camera = camera
        self.tts = tts
        self._robot_error: Optional[Exception] = None

        self._blink_stop = threading.Event()
        self._blink_thread: Optional[threading.Thread] = None
        self._fidget_stop = threading.Event()
        self._fidget_thread: Optional[threading.Thread] = None

        self._ensure_backends()
        self._status("ready")

    def _status(self, *parts):
        if self.verbose:
            print("[anim]", *parts)

    def _ensure_backends(self) -> None:
        if self.robot is None:
            try:
                import robot_moves as rm

                if hasattr(rm, "RobotMoves"):
                    self.robot = rm.RobotMoves(base_speed=self.base_speed)
                else:
                    self.robot = rm
                self._robot_error = None
                self._status("robot backend:", type(self.robot).__name__)
            except Exception as e:
                self.robot = None
                self._robot_error = e
                self._status("robot backend unavailable:", e)

        if self.eyes is None:
            try:
                import eyes_lib

                self.eyes = eyes_lib.get_eyes()
                self._status("eyes backend: ready")
            except Exception:
                self.eyes = None
                self._status("eyes backend: unavailable")

        if self.camera is None:
            try:
                import camera_lib

                self.camera = camera_lib.get_camera()
                self._status("camera backend: ready")
            except Exception:
                self.camera = None
                self._status("camera backend: unavailable")

        if self.tts is None:
            try:
                import tts_lib

                self.tts = tts_lib
                self._status("tts backend: ready")
            except Exception:
                self.tts = None
                self._status("tts backend: unavailable")

    def run_async(self, fn: Callable, *args, **kwargs) -> threading.Thread:
        t = threading.Thread(target=fn, args=args, kwargs=kwargs, daemon=True)
        t.start()
        self._status("async start:", getattr(fn, "__name__", str(fn)))
        return t

    # ---------------- movement ----------------
    def _resolve_move(self, move_name: str):
        aliases = {
            "forward": ("forward", "move_forward"),
            "backward": ("backward", "move_backward"),
            "left": ("left", "move_left"),
            "right": ("right", "move_right"),
            "turn_left": ("turn_left", "spin_left"),
            "turn_right": ("turn_right", "spin_right"),
            "drift_left": ("drift_left",),
            "drift_right": ("drift_right",),
        }
        names = aliases.get(move_name, (move_name,))
        for name in names:
            fn = getattr(self.robot, name, None) if self.robot is not None else None
            if callable(fn):
                return fn
        return None

    def move(self, move_name: str, seconds: float = 0.5, speed: Optional[float] = None, **kwargs):
        self._ensure_backends()
        if self.robot is None:
            detail = f" ({self._robot_error})" if self._robot_error else ""
            raise RuntimeError(f"robot_moves not available{detail}")

        fn = self._resolve_move(move_name)
        if fn is None:
            raise AttributeError(f"robot move '{move_name}' not found on robot_moves backend")
        self._status(f"move: {move_name}", f"seconds={seconds}", f"speed={speed}", kwargs)
        return fn(seconds=seconds, speed=speed, **kwargs)

    def move_async(self, move_name: str, seconds: float = 0.5, speed: Optional[float] = None, **kwargs):
        return self.run_async(self.move, move_name, seconds, speed, **kwargs)

    # ---------------- eyes ----------------
    def set_eye_color(self, color: RGB):
        self.default_eye_color = tuple(int(v) for v in color)
        self._status("eyes color:", self.default_eye_color)
        if self.eyes:
            self.eyes.set_both(*self.default_eye_color)

    def blink_once(self, color: Optional[RGB] = None, blank_s: float = 0.5):
        if not self.eyes:
            self._status("blink_once skipped: eyes unavailable")
            return
        c = tuple(int(v) for v in (color or self.default_eye_color))
        self._status("blink_once", f"color={c}", f"blank_s={blank_s}")
        self.eyes.set_both(*c)
        self.eyes.off()
        self._sleep(max(0.0, float(blank_s)))
        self.eyes.set_both(*c)

    def wink(self, side: str = "left", color: Optional[RGB] = None, blank_s: float = 0.5):
        if not self.eyes:
            self._status("wink skipped: eyes unavailable")
            return
        c = tuple(int(v) for v in (color or self.default_eye_color))
        self._status("wink", f"side={side}", f"color={c}", f"blank_s={blank_s}")
        self.eyes.set_both(*c)
        if str(side).strip().lower() in ("left", "l"):
            self.eyes.set_left(0, 0, 0)
            self._sleep(max(0.0, float(blank_s)))
            self.eyes.set_left(*c)
        else:
            self.eyes.set_right(0, 0, 0)
            self._sleep(max(0.0, float(blank_s)))
            self.eyes.set_right(*c)

    def _blink_loop(self, every_s: float, color: RGB, blank_s: float):
        interval = max(0.05, float(every_s))
        self._status("blink loop start", f"every_s={interval}", f"blank_s={blank_s}")
        while not self._blink_stop.is_set():
            if self._blink_stop.wait(interval):
                break
            if self.eyes:
                self._status("blink")
                self.eyes.off()
                if self._blink_stop.wait(max(0.0, float(blank_s))):
                    break
                self.eyes.set_both(*color)
        self._status("blink loop stop")

    def start_blinking(self, every_s: float = 3.0, color: Optional[RGB] = None, blank_s: float = 0.5):
        if not self.eyes:
            self._status("start_blinking skipped: eyes unavailable")
            return None
        c = tuple(int(v) for v in (color or self.default_eye_color))
        self._status("start_blinking", f"every_s={every_s}", f"color={c}", f"blank_s={blank_s}")
        self.eyes.set_both(*c)
        self.stop_blinking()
        self._blink_stop.clear()
        self._blink_thread = self.run_async(self._blink_loop, every_s, c, blank_s)
        return self._blink_thread

    def stop_blinking(self):
        self._status("stop_blinking")
        if self._blink_thread and self._blink_thread.is_alive():
            self._blink_stop.set()
            self._blink_thread.join(timeout=1.0)
        self._blink_thread = None
        self._blink_stop.clear()

    # ---------------- camera ----------------
    def nod(self, depth: int = 250, speed_s: Optional[float] = None):
        if self.camera:
            self._status("camera nod", f"depth={depth}", f"speed_s={speed_s}")
            self.camera.nod(depth=depth, speed_s=speed_s)
        else:
            self._status("camera nod skipped: camera unavailable")

    def shake(self, width: int = 250, speed_s: Optional[float] = None):
        if self.camera:
            self._status("camera shake", f"width={width}", f"speed_s={speed_s}")
            self.camera.shake(width=width, speed_s=speed_s)
        else:
            self._status("camera shake skipped: camera unavailable")

    def look_left(self, amplitude: int = 250, hold_s: float = 0.15):
        if self.camera:
            self._status("camera look_left", f"amplitude={amplitude}", f"hold_s={hold_s}")
            self.camera.glance_left(amplitude=amplitude, hold_s=hold_s)
        else:
            self._status("camera look_left skipped: camera unavailable")

    def look_right(self, amplitude: int = 250, hold_s: float = 0.15):
        if self.camera:
            self._status("camera look_right", f"amplitude={amplitude}", f"hold_s={hold_s}")
            self.camera.glance_right(amplitude=amplitude, hold_s=hold_s)
        else:
            self._status("camera look_right skipped: camera unavailable")

    def look_up(self, amplitude: int = 250, hold_s: float = 0.15):
        if self.camera:
            self._status("camera look_up", f"amplitude={amplitude}", f"hold_s={hold_s}")
            self.camera.look_up(amplitude=amplitude, hold_s=hold_s)
        else:
            self._status("camera look_up skipped: camera unavailable")

    def look_down(self, amplitude: int = 250, hold_s: float = 0.15):
        if self.camera:
            self._status("camera look_down", f"amplitude={amplitude}", f"hold_s={hold_s}")
            self.camera.look_down(amplitude=amplitude, hold_s=hold_s)
        else:
            self._status("camera look_down skipped: camera unavailable")

    def center_camera(self):
        if self.camera:
            self._status("camera center")
            self.camera.center_all()
        else:
            self._status("camera center skipped: camera unavailable")

    # ---------------- tts ----------------
    def get_available_voices(self) -> List[str]:
        if not self.tts:
            return []
        vm = getattr(self.tts, "VOICE_MAP", None)
        if isinstance(vm, dict):
            return sorted([str(k).strip().lower() for k in vm.keys()])
        return []

    def show_voices(self) -> List[str]:
        voices = self.get_available_voices()
        self._status(f"voices available: {len(voices)}")
        for i, v in enumerate(voices, start=1):
            print(f"[anim]   {i}. {v}")
        if not voices:
            self._status("voice map unavailable; using current voice:", self.voice)
        return voices

    def select_voice(self, voice: Optional[str] = None, number: Optional[int] = None) -> str:
        """
        Choose voice by name or 1-based number.
        Examples:
            anim.show_voices()
            anim.select_voice(number=1)
            anim.select_voice("ryan")
            anim.select_voice("2")   # also works
        """
        voices = self.get_available_voices()

        if number is not None:
            idx = int(number)
            if idx < 1 or idx > len(voices):
                raise ValueError(f"Voice number out of range: {idx} (1..{len(voices)})")
            self.voice = voices[idx - 1]
            self._status("voice selected by number:", idx, "->", self.voice)
            return self.voice

        if voice is None:
            self.show_voices()
            self._status("voice unchanged:", self.voice)
            return self.voice

        token = str(voice).strip().lower()
        if token.isdigit():
            return self.select_voice(number=int(token))

        if voices and token not in voices:
            raise ValueError(f"Unknown voice '{token}'. Available: {voices}")

        self.voice = token
        self._status("voice selected:", self.voice)
        return self.voice

    def select_voice_number(self, number: int) -> str:
        return self.select_voice(number=number)

    def generate_phrase(
        self,
        key: str,
        text: str,
        voice: Optional[str] = None,
        length_scale: str = "0.98",
        sentence_silence: str = "0.08",
    ) -> str:
        if not self.tts:
            raise RuntimeError("tts_lib not available")
        v = (voice or self.voice).strip().lower()
        self._status("generate_phrase", f"key={key}", f"voice={v}", f"text={text}")
        path = self.tts.pre_synth(
            text,
            voice=v,
            length_scale=length_scale,
            sentence_silence=sentence_silence,
        )
        self.phrases[key] = {
            "path": path,
            "voice": v,
            "text": text,
            "length_scale": str(length_scale),
            "sentence_silence": str(sentence_silence),
        }
        return path

    def play_phrase(self, key: str, block: bool = True, device: Optional[str] = None) -> str:
        if not self.tts:
            raise RuntimeError("tts_lib not available")
        if key not in self.phrases:
            raise KeyError(f"Unknown phrase: {key}")
        path = self.phrases[key]["path"]
        self._status("play_phrase", f"key={key}", f"block={block}", f"device={device}")
        p = self.tts.play_path_async(path, device=device)
        if block:
            p.wait()
        return path

    def speak(self, text: str, block: bool = True, voice: Optional[str] = None) -> str:
        if not self.tts:
            raise RuntimeError("tts_lib not available")
        self._status("speak", f"voice={voice or self.voice}", f"block={block}", f"text={text}")
        return self.tts.say(text, voice=voice or self.voice, block=block)

    # ---------------- fidget ----------------
    def do_fidget(
        self,
        duration_s: float,
        step_s: float = 0.05,
        speed_scale: float = 0.12,
        seed: Optional[int] = None,
    ) -> List[str]:
        step = max(0.01, float(step_s))
        speed_ratio = _clamp(speed_scale, 0.05, 1.0)
        speed = self.base_speed * speed_ratio
        actions = build_fidget_plan(duration_s=duration_s, step_s=step, seed=seed)
        self._status("do_fidget", f"duration_s={duration_s}", f"step_s={step}", f"speed_scale={speed_scale}", f"seed={seed}")
        for action in actions:
            self._status("fidget action:", action)
            self.move(action, seconds=step, speed=speed)
        return actions

    def _fidget_loop(self, step_s: float, speed_scale: float, seed: Optional[int]):
        rng = random.Random(seed)
        step = max(0.01, float(step_s))
        speed_ratio = _clamp(speed_scale, 0.05, 1.0)
        speed = self.base_speed * speed_ratio
        self._status("fidget loop start", f"step_s={step}", f"speed_scale={speed_scale}", f"seed={seed}")
        while not self._fidget_stop.is_set():
            action = rng.choice(DEFAULT_FIDGET_MOVES)
            self._status("fidget action:", action)
            self.move(action, seconds=step, speed=speed)
            # Add a tiny gap so fidget feels nervous/subtle rather than frantic.
            self._sleep(step * 0.6)
        self._status("fidget loop stop")

    def start_fidget(self, step_s: float = 0.05, speed_scale: float = 0.12, seed: Optional[int] = None):
        self._status("start_fidget", f"step_s={step_s}", f"speed_scale={speed_scale}", f"seed={seed}")
        self.stop_fidget()
        self._fidget_stop.clear()
        self._fidget_thread = self.run_async(self._fidget_loop, step_s, speed_scale, seed)
        return self._fidget_thread

    def stop_fidget(self):
        self._status("stop_fidget")
        if self._fidget_thread and self._fidget_thread.is_alive():
            self._fidget_stop.set()
            self._fidget_thread.join(timeout=1.0)
        self._fidget_thread = None
        self._fidget_stop.clear()

    def stop(self):
        """
        Stop background animation threads and stop robot motors.
        Equivalent intent to bot.stop().
        """
        self._status("stop")
        self.stop_fidget()
        self.stop_blinking()
        self._ensure_backends()
        if self.robot is None:
            return
        called = False

        fn = getattr(self.robot, "stop", None)
        if callable(fn):
            fn()
            called = True

        fn = getattr(self.robot, "emergency_stop", None)
        if callable(fn):
            fn()
            called = True

        fn = getattr(self.robot, "all_stop", None)
        if callable(fn):
            try:
                fn()
            except TypeError:
                pass
            else:
                called = True

        if not called:
            self._status("stop: no robot stop API found")

    def stop_all(self):
        self.stop()

    # ---------------- extras ----------------
    def horn_normal(self, block: bool = True):
        self._ensure_backends()
        if self.robot is None:
            detail = f" ({self._robot_error})" if self._robot_error else ""
            raise RuntimeError(f"robot_moves not available{detail}")
        horn = getattr(self.robot, "horn", None)
        if callable(horn):
            self._status("horn_normal", f"block={block}")
            return horn(block=block)
        horn = getattr(self.robot, "Horn", None)
        if callable(horn):
            self._status("Horn()")
            return horn()
        self._status("horn skipped: horn API unavailable")
        return False

    def demo_turbo_sequence(self):
        self._status("demo_turbo_sequence start")
        intro_line = (
            "Hi, I'm Turbo. I've been asked to talk to you. "
            "When all I want to do is drift around corners."
        )
        sigh_line = "Siiigh."
        self.set_eye_color((60, 180, 255))
        self.start_blinking(every_s=2.8, blank_s=0.5)
        self.start_fidget(step_s=0.05, speed_scale=0.12)

        try:
            if self.tts:
                path_intro = self.generate_phrase("turbo_intro", intro_line, length_scale="1.05")
                path_sigh = self.generate_phrase("turbo_sigh", sigh_line, length_scale="1.35", sentence_silence="0.18")
                dur = self.tts.wav_duration_seconds(path_intro)
                p = self.tts.play_path_async(path_intro)
            else:
                dur = 4.0
                p = None
                path_sigh = None

            self._sleep(min(0.7, max(0.2, dur * 0.25)))
            self.look_left(amplitude=230, hold_s=0.18)
            self._sleep(0.22)
            self.look_right(amplitude=230, hold_s=0.18)
            self._sleep(0.30)

            if p is not None:
                p.wait()

            # Pause before the sigh beat so it lands clearly.
            self._sleep(0.35)
            self.shake(width=260, speed_s=0.16)
            self._sleep(0.20)
            if self.tts and path_sigh is not None:
                self.tts.play_path_async(path_sigh).wait()
        finally:
            self.stop_fidget()
            self.stop_blinking()

        self._sleep(0.25)
        self.move("drift_left", seconds=1.0, turn_blend=0.95)
        self._sleep(0.35)
        self.move("drift_right", seconds=1.0, turn_blend=0.95)
        self._status("demo_turbo_sequence end")



def get_animation_lib(**kwargs) -> AnimationLibrary:
    lock = getattr(builtins, _LOCK_KEY, None)
    if lock is None:
        lock = threading.Lock()
        setattr(builtins, _LOCK_KEY, lock)

    with lock:
        inst = getattr(builtins, _SINGLETON_KEY, None)
        if inst is None:
            inst = AnimationLibrary(**kwargs)
            setattr(builtins, _SINGLETON_KEY, inst)
            return inst

        if "robot" in kwargs and kwargs["robot"] is not None:
            inst.robot = kwargs["robot"]
            inst._robot_error = None
        if "eyes" in kwargs:
            inst.eyes = kwargs["eyes"]
        if "camera" in kwargs:
            inst.camera = kwargs["camera"]
        if "tts" in kwargs:
            inst.tts = kwargs["tts"]
        if "base_speed" in kwargs and kwargs["base_speed"] is not None:
            inst.base_speed = float(kwargs["base_speed"])
        if "eye_color" in kwargs and kwargs["eye_color"] is not None:
            inst.set_eye_color(kwargs["eye_color"])
        if "verbose" in kwargs and kwargs["verbose"] is not None:
            inst.verbose = bool(kwargs["verbose"])

        inst._ensure_backends()
        inst._status("reused singleton")
        return inst



def reset_animation_lib() -> None:
    lock = getattr(builtins, _LOCK_KEY, None)
    if lock is None:
        lock = threading.Lock()
        setattr(builtins, _LOCK_KEY, lock)

    with lock:
        inst = getattr(builtins, _SINGLETON_KEY, None)
        if inst is not None:
            if getattr(inst, "verbose", False):
                print("[anim] reset_animation_lib")
            try:
                inst.stop_fidget()
            except Exception:
                pass
            try:
                inst.stop_blinking()
            except Exception:
                pass
        setattr(builtins, _SINGLETON_KEY, None)
