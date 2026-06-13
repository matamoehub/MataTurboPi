__version__ = "1.3.0"

# tts_lib.py
# Piper TTS helper for Matamoe robots (Python 3.10)
#
# Features:
# - Synth text -> wav (cached by hash)
# - Play wav via aplay
# - Simple say() helper (blocking or async)

import os
import time
import subprocess
import shutil
import hashlib
import tempfile
import wave
import contextlib
import re
from typing import Optional

# Allow override so it works no matter which user runs Jupyter/services
VOICE_DIR = os.environ.get("PIPER_VOICE_DIR", os.path.expanduser("/opt/robot/piper/voices"))

VOICE_MAP = {
    "ryan": ("en_US-ryan-high.onnx", "en_US-ryan-high.onnx.json"),
    "amy":  ("en_US-amy-medium.onnx", "en_US-amy-medium.onnx.json"),
    "aru":  ("en_GB-aru-medium.onnx", "en_GB-aru-medium.onnx.json"),
    # More voices are auto-discovered from VOICE_DIR (see below).
}


def _discover_installed_voices() -> None:
    """Register any *.onnx voice present in VOICE_DIR that isn't already mapped.

    A voice file dropped into VOICE_DIR is then usable by its short name —
    the middle token of the Piper filename, e.g.
        en_GB-alba-medium.onnx  ->  "alba"
    — with no code change. Requires the matching <name>.onnx.json to be present.

    Curated names in VOICE_MAP above win if a key collides.
    """
    try:
        entries = set(os.listdir(VOICE_DIR))
    except Exception:
        return
    mapped_files = {model for model, _cfg in VOICE_MAP.values()}
    for fname in sorted(entries):
        if not fname.endswith(".onnx") or fname in mapped_files:
            continue
        cfg = fname + ".json"
        if cfg not in entries:
            continue  # need both the model and its config
        parts = fname[:-len(".onnx")].split("-")
        key = (parts[1] if len(parts) >= 2 else parts[0]).strip().lower()
        if key and key not in VOICE_MAP:
            VOICE_MAP[key] = (fname, cfg)


_discover_installed_voices()

DEFAULT_VOICE = "ryan"
PIPER_BIN_ENV = "PIPER_BIN"
PIPER_VOICE_ENV = "PIPER_VOICE"
VOLUME_CONTROLS = tuple(
    name.strip()
    for name in os.environ.get("ROBOT_VOLUME_CONTROLS", "Master,PCM,Speaker").split(",")
    if name.strip()
)
DEFAULT_AUDIO_VOLUME = max(0, min(100, int(os.environ.get("ROBOT_DEFAULT_AUDIO_VOLUME", "40"))))
_DEFAULT_VOLUME_READY = False
_DEFAULT_VOLUME_FAILED = False   # set after first failure so we warn once only
PREFERRED_VOLUME_CONTROLS = (
    "Master",
    "PCM",
    "Speaker",
    "Headphone",
    "Playback",
    "Line Out",
    "Digital",
    "Audio",
)


def available_voices(installed_only: bool = True) -> list[str]:
    """Return voices in classroom order, optionally filtered to installed voices only."""
    voices = list(VOICE_MAP.keys())
    if not installed_only:
        return voices
    out: list[str] = []
    for name in voices:
        try:
            _voice_paths(name)
            out.append(name)
        except Exception:
            pass
    return out


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




def _resolve_piper_bin() -> str:
    configured = (os.environ.get(PIPER_BIN_ENV) or "").strip()
    candidates = []
    if configured:
        candidates.append(configured)
    discovered = shutil.which("piper")
    if discovered:
        candidates.append(discovered)
    candidates.append("/opt/robot/piper/piper")

    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate

    tried = ", ".join(candidates) if candidates else "(none)"
    raise FileNotFoundError(
        "Piper binary not found. Tried: " + tried +
        ". Set PIPER_BIN to the correct executable path."
    )


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


def _require_amixer():
    if not shutil.which("amixer"):
        raise RuntimeError(
            "amixer not found. Install: sudo apt update && sudo apt install -y alsa-utils"
        )


def _clamp_volume_percent(percent: int) -> int:
    return max(0, min(100, int(percent)))


def _amixer_control_exists(control: str) -> bool:
    proc = subprocess.run(
        ["amixer", "sget", str(control)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        cwd=_safe_workdir(),
        check=False,
    )
    return proc.returncode == 0


def _list_amixer_controls() -> list[str]:
    proc = subprocess.run(
        ["amixer", "scontrols"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=_safe_workdir(),
        check=False,
    )
    if proc.returncode != 0:
        return []
    found = re.findall(r"Simple mixer control '([^']+)'", proc.stdout or "")
    seen = set()
    ordered = []
    for name in found:
        if name not in seen:
            seen.add(name)
            ordered.append(name)
    return ordered


def _amixer_control_has_playback_volume(control: str) -> bool:
    proc = subprocess.run(
        ["amixer", "sget", str(control)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        cwd=_safe_workdir(),
        check=False,
    )
    if proc.returncode != 0:
        return False
    output = proc.stdout or ""
    return bool(re.search(r"\[\d{1,3}%\]", output))


def _pick_volume_control(control: Optional[str] = None) -> str:
    _require_amixer()
    if control:
        return str(control)
    configured = [candidate for candidate in VOLUME_CONTROLS if candidate]
    available = _list_amixer_controls()

    for candidate in configured:
        if _amixer_control_exists(candidate) and _amixer_control_has_playback_volume(candidate):
            return candidate

    preferred = list(PREFERRED_VOLUME_CONTROLS)
    for candidate in preferred:
        for available_name in available:
            if available_name.lower() == candidate.lower() and _amixer_control_has_playback_volume(available_name):
                return available_name

    for available_name in available:
        if _amixer_control_has_playback_volume(available_name):
            return available_name

    if available:
        raise RuntimeError(
            "No supported ALSA playback volume control found. "
            f"Configured: {', '.join(configured) or '(none)'}; "
            f"available mixer controls: {', '.join(available)}"
        )

    raise RuntimeError(
        "No ALSA mixer controls were found. "
        f"Configured controls: {', '.join(configured) or '(none)'}"
    )


def _set_volume_pipewire(percent: int) -> bool:
    """Try setting volume via wpctl (PipeWire/WirePlumber). Returns True on success."""
    if not shutil.which("wpctl"):
        return False
    try:
        # wpctl uses 0.0–1.0 scale, not percent
        vol = f"{percent / 100:.2f}"
        proc = subprocess.run(
            ["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", vol],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, cwd=_safe_workdir(), check=False,
        )
        return proc.returncode == 0
    except Exception:
        return False


def _set_volume_pulseaudio(percent: int) -> bool:
    """Try setting volume via pactl (PulseAudio). Returns True on success."""
    if not shutil.which("pactl"):
        return False
    try:
        proc = subprocess.run(
            ["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{percent}%"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, cwd=_safe_workdir(), check=False,
        )
        return proc.returncode == 0
    except Exception:
        return False


def _get_volume_pipewire() -> Optional[int]:
    """Read volume via wpctl (PipeWire/WirePlumber). Returns 0-100 or None."""
    if not shutil.which("wpctl"):
        return None
    try:
        proc = subprocess.run(
            ["wpctl", "get-volume", "@DEFAULT_AUDIO_SINK@"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, cwd=_safe_workdir(), check=False,
        )
        if proc.returncode != 0:
            return None
        # Output like: "Volume: 0.40" or "Volume: 0.40 [MUTED]"
        m = re.search(r"Volume:\s*([0-9.]+)", proc.stdout or "")
        if not m:
            return None
        return _clamp_volume_percent(int(round(float(m.group(1)) * 100)))
    except Exception:
        return None


def _get_volume_pulseaudio() -> Optional[int]:
    """Read volume via pactl (PulseAudio). Returns 0-100 or None."""
    if not shutil.which("pactl"):
        return None
    try:
        proc = subprocess.run(
            ["pactl", "get-sink-volume", "@DEFAULT_SINK@"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, cwd=_safe_workdir(), check=False,
        )
        if proc.returncode != 0:
            return None
        m = re.search(r"(\d{1,3})%", proc.stdout or "")
        if not m:
            return None
        return _clamp_volume_percent(int(m.group(1)))
    except Exception:
        return None


def set_volume(percent: int, control: Optional[str] = None) -> dict:
    """
    Set robot audio output volume.

    Tries backends in order: PipeWire (wpctl) → PulseAudio (pactl) → ALSA (amixer).
    Raspberry Pi 5 with modern Raspberry Pi OS uses PipeWire, so amixer finds no
    controls — the wpctl path handles it cleanly.
    """
    value = _clamp_volume_percent(percent)

    # 1. PipeWire (Pi 5 / modern Raspberry Pi OS)
    if not control and _set_volume_pipewire(value):
        return {"ok": True, "control": "wpctl:@DEFAULT_AUDIO_SINK@", "volume": value}

    # 2. PulseAudio
    if not control and _set_volume_pulseaudio(value):
        return {"ok": True, "control": "pactl:@DEFAULT_SINK@", "volume": value}

    # 3. ALSA amixer fallback
    chosen = _pick_volume_control(control)
    proc = subprocess.run(
        ["amixer", "set", chosen, f"{value}%"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=_safe_workdir(),
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout or "amixer volume set failed").strip())
    return {"ok": True, "control": chosen, "volume": value}


def get_volume(control: Optional[str] = None) -> dict:
    """
    Return the current audio output volume percentage.

    Tries the same backends as set_volume: PipeWire (wpctl) → PulseAudio
    (pactl) → ALSA (amixer). On a Pi 5 / modern Raspberry Pi OS amixer finds
    no controls, so the wpctl path is what actually answers there.
    """
    if not control:
        vol = _get_volume_pipewire()
        if vol is not None:
            return {"ok": True, "control": "wpctl:@DEFAULT_AUDIO_SINK@", "volume": vol}
        vol = _get_volume_pulseaudio()
        if vol is not None:
            return {"ok": True, "control": "pactl:@DEFAULT_SINK@", "volume": vol}

    chosen = _pick_volume_control(control)
    proc = subprocess.run(
        ["amixer", "sget", chosen],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=_safe_workdir(),
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout or "amixer volume read failed").strip())

    matches = re.findall(r"\[(\d{1,3})%\]", proc.stdout or "")
    volume = int(matches[-1]) if matches else None
    return {"ok": True, "control": chosen, "volume": volume, "raw": proc.stdout}


def ensure_default_volume(percent: int = DEFAULT_AUDIO_VOLUME, control: Optional[str] = None) -> dict:
    """
    Set the default audio playback volume once per process.
    Tries PipeWire → PulseAudio → ALSA in order.
    After the first attempt (success or failure) this becomes a no-op so the
    warning never repeats on every say() call.
    """
    global _DEFAULT_VOLUME_READY, _DEFAULT_VOLUME_FAILED
    if _DEFAULT_VOLUME_READY or _DEFAULT_VOLUME_FAILED:
        return {"ok": _DEFAULT_VOLUME_READY, "cached": True, "volume": _clamp_volume_percent(percent)}
    result = set_volume(percent, control=control)
    _DEFAULT_VOLUME_READY = True
    return result


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
    piper_bin = _resolve_piper_bin()

    proc = subprocess.run(
        [
            piper_bin,
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


def _is_valid_wav(path: str) -> bool:
    try:
        if not os.path.isfile(path) or os.path.getsize(path) < 44:
            return False
        with contextlib.closing(wave.open(path, "rb")) as w:
            return w.getnframes() > 0 and w.getframerate() > 0
    except Exception:
        return False


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
    if not _is_valid_wav(out_path):
        try:
            if os.path.exists(out_path):
                os.remove(out_path)
        except Exception:
            pass
        synth_to_wav(
            text,
            voice=voice_name,
            length_scale=length_scale,
            sentence_silence=sentence_silence,
            out_path=out_path,
        )
        if not _is_valid_wav(out_path):
            raise RuntimeError(f"Generated invalid wav: {out_path}")
    return out_path


def wav_duration_seconds(path: str) -> float:
    if not _is_valid_wav(path):
        raise RuntimeError(f"Invalid wav file: {path}")
    with contextlib.closing(wave.open(path, "rb")) as w:
        return w.getnframes() / float(w.getframerate())


def play_wav_async(path: str, device: Optional[str] = None):
    """
    Play wav using aplay (returns subprocess.Popen).
    """
    _require_aplay()
    try:
        ensure_default_volume()
    except Exception as e:
        global _DEFAULT_VOLUME_FAILED
        if not _DEFAULT_VOLUME_FAILED:
            print("[tts_lib] warn: could not set volume (will not retry):", e)
            _DEFAULT_VOLUME_FAILED = True
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

