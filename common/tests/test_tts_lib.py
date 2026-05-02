import importlib.util
import os
import stat
from pathlib import Path


def _load_tts_lib():
    path = Path(__file__).resolve().parents[1] / "lib" / "tts_lib.py"
    spec = importlib.util.spec_from_file_location("tts_lib_for_test", str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_prefers_configured_piper_bin(monkeypatch, tmp_path):
    tts_lib = _load_tts_lib()
    fake = tmp_path / "piper"
    fake.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)

    monkeypatch.setenv("PIPER_BIN", str(fake))

    assert tts_lib._resolve_piper_bin() == str(fake)


def test_raises_clear_error_when_no_piper_bin(monkeypatch):
    tts_lib = _load_tts_lib()
    monkeypatch.delenv("PIPER_BIN", raising=False)
    monkeypatch.setattr(tts_lib.shutil, "which", lambda _name: None)
    monkeypatch.setattr(tts_lib.os.path, "isfile", lambda _path: False)

    try:
        tts_lib._resolve_piper_bin()
    except FileNotFoundError as exc:
        assert "PIPER_BIN" in str(exc)
    else:
        raise AssertionError("expected FileNotFoundError")
