"""
ROBOT 2 — RYAN (Turbo) — OUTTAKES
Run this on Robot 2. Each scene is self-contained — you can re-run
individual scenes by calling slate() then the scene block.

SYNC: Robot 2 is the FOLLOWER (same handshake as the main video).
  1. Run Robot 1's outtakes first — it prints its IP.
  2. Paste that IP below as ROBOT1_IP, then run this.
  3. Wait — Robot 2 connects and both start when Robot 1 presses Enter.
"""
from lesson_header import *
import time
import socket
import subprocess
import shutil
from pathlib import Path

# ── PASTE ROBOT 1'S IP HERE ───────────────────────────────────────────────────
ROBOT1_IP = "192.168.1.100"   # ← change this to Robot 1's IP each session

# ── SOUND FILES ───────────────────────────────────────────────────────────────
# Put your mp3s on THIS robot (Robot 2). Recommended location:
#     /opt/robot/sounds/snore.mp3
#     /opt/robot/sounds/hypno.mp3
# (any of the SOUND_DIRS below also works). Change the names here if yours differ.
SNORE_MP3 = "snore.mp3"
HYPNO_MP3 = "hypno.mp3"
SOUND_DIRS = [
    "/opt/robot/sounds",
    str(Path.home() / "sounds"),
    str(Path(__file__).resolve().parent / "sounds"),
    ".",
]


_last_sound = None   # most recent non-blocking sound process (so slate can stop it)


def play_sound(filename, block=False):
    """Play an mp3 via mpg123. Returns the Popen (truthy) or None if not found.

    Non-blocking by default so the sound plays UNDER the action (e.g. snoring
    while Amy talks). Falls back to None so callers can use a buzzer instead.
    """
    global _last_sound
    if not shutil.which("mpg123"):
        print("  [sound] mpg123 not installed (sudo apt install -y mpg123)")
        return None
    for d in SOUND_DIRS:
        p = Path(d) / filename
        if p.is_file():
            cmd = ["mpg123", "-q", str(p)]
            try:
                if block:
                    subprocess.run(cmd)
                    return True
                proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                _last_sound = proc
                return proc
            except Exception as e:
                print(f"  [sound] failed to play {filename}: {e}")
                return None
    print(f"  [sound] not found: {filename} — put it in /opt/robot/sounds/")
    return None


def stop_sound():
    """Stop any still-playing sound so it can't bleed into the next take."""
    global _last_sound
    if _last_sound is not None:
        try:
            _last_sound.terminate()
        except Exception:
            pass
        _last_sound = None


# ── SYNC — connect to Robot 1 and wait for the GO signal ─────────────────────
def _sync_follower(ip, port=9877):
    print(f"  Connecting to Robot 1 at {ip}...")
    s = socket.socket()
    s.connect((ip, port))
    print(f"  Connected. Waiting for Robot 1 to press Enter...")
    s.recv(16)   # 'GO'
    print("  GO!\n")
    return s

_sock = _sync_follower(ROBOT1_IP)


def _wait_scene(timeout_s=60.0):
    """Block until Robot 1's scene cue arrives — keeps the two reels in lockstep.

    Robot 1 leads each take: it does the (audible) slate, then releases us here
    so both robots start every scene together. Times out so a dropped link
    can't hang the reel.
    """
    try:
        _sock.settimeout(timeout_s)
        _sock.recv(16)   # 'SCENE'
    except Exception:
        pass

myRobot = bot(base_speed=300)
myRobot.voice.select("ryan")
myRobot.voice.set_volume(90)

# ── MOTOR WARM-UP — prime the motors off-camera before any take ──────────────
# The mecanum motors draw a stall/inrush spike on the first cold move. The
# library auto-primes on first move, but do it here during setup so it never
# happens mid-take.
myRobot.move.forward(seconds=0.1, speed=120)
myRobot.move.backward(seconds=0.1, speed=120)
myRobot.move.stop()


def cut():
    """Visual 'cut' on Robot 2 — red eyes, NO beep (the cut/slate sound is
    Robot 1's job, so the clapper isn't doubled across both robots)."""
    myRobot.move.stop()
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(255, 40, 0)                   # red = cut (visual only)
    time.sleep(0.3)


_first_slate = True

def slate():
    """Silent reset + wait for Robot 1's scene cue.

    The slate/clapper SOUND plays on Robot 1 only. Here we just do the visual
    reset, then block until Robot 1 releases us so both start the scene together.
    """
    global _first_slate
    stop_sound()                    # kill any lingering snore/hypno from last take
    if not _first_slate:
        cut()                       # visual cut of the take we just flubbed
    _first_slate = False
    myRobot.move.stop()
    myRobot.camera.center()
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(255, 255, 255)   # visual clapper flash (no beep)
    time.sleep(0.3)
    myRobot.eyes.color(0, 255, 200)
    _wait_scene()                   # ← lockstep: start the scene on Robot 1's cue

# ── SCENE 1 — LINE FLUB ──────────────────────────────────────────────────────
print("Scene 1 — Line Flub")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
myRobot.camera.wiggle(cycles=2, amplitude=180)
time.sleep(2.5)
myRobot.eyes.color(255, 255, 0)
myRobot.camera.wiggle(cycles=4, amplitude=220)
myRobot.move.left(seconds=0.2, speed=200)
myRobot.move.right(seconds=0.2, speed=200)
myRobot.buzzer.beep(freq=800, duration_s=0.1)
myRobot.buzzer.beep(freq=900, duration_s=0.1)
myRobot.buzzer.beep(freq=800, duration_s=0.1)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 2 — EARLY DRIFT ────────────────────────────────────────────────────
print("Scene 2 — Early Drift")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
time.sleep(1.5)
myRobot.move.forward(seconds=0.12)
myRobot.move.backward(seconds=0.12)
time.sleep(0.4)
myRobot.eyes.color(255, 165, 0)
myRobot.move.forward(seconds=0.12)
myRobot.move.backward(seconds=0.12)
myRobot.voice.say("I thought it was drift time!", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 165, 0)
myRobot.move.drift_right(seconds=1.0, speed=320, turn_blend=0.5)
myRobot.move.drift_left(seconds=1.0, speed=320, turn_blend=0.5)
myRobot.move.stop()
myRobot.camera.wiggle(cycles=2, amplitude=150)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 3 — SNORING ────────────────────────────────────────────────────────
print("Scene 3 — Snoring")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.4)
time.sleep(2.0)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 30, 60)
myRobot.camera.down(amplitude=300)
time.sleep(0.5)
myRobot.camera.down(amplitude=450)
# Snore: play the mp3 under the gag (Amy's "Are you snoring?" lands over it).
# Falls back to the buzzer if the file isn't on the robot yet.
snore = play_sound(SNORE_MP3)
if snore:
    time.sleep(3.0)                       # let it snore while Amy talks
else:
    myRobot.buzzer.beep(freq=280, duration_s=0.7)
    time.sleep(0.3)
    myRobot.buzzer.beep(freq=240, duration_s=0.9)
    time.sleep(0.4)
    myRobot.buzzer.beep(freq=280, duration_s=0.7)
    time.sleep(0.3)
myRobot.eyes.color(255, 255, 0)
myRobot.camera.center()
myRobot.move.forward(seconds=0.12)
myRobot.move.backward(seconds=0.12)
myRobot.camera.wiggle(cycles=4, amplitude=220)
myRobot.anim.start_blinking(every_s=0.6, blank_s=0.15)
time.sleep(1.5)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 4 — HYPNO EYES ─────────────────────────────────────────────────────
print("Scene 4 — Hypno Eyes")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
time.sleep(1.0)
myRobot.anim.stop_blinking()
# Hypno toad: play the mp3 while the eyes swirl red-heavy and hold on RED.
# Falls back to the buzzer sweep if the file isn't on the robot yet.
hypno = play_sound(HYPNO_MP3)
colours = [
    (255,0,0),(0,0,255),(128,0,128),
    (255,0,0),(255,165,0),(255,0,0),      # red-dominant swirl
    (255,0,255),(0,255,255),(255,0,0),
    (0,0,255),(128,0,128),(255,0,0),
]
for colour in colours:
    myRobot.eyes.color(*colour)
    time.sleep(0.18 if hypno else 0.12)
if not hypno:
    for f in [400, 600, 900, 1200, 1600, 2000]:
        myRobot.buzzer.beep(freq=f, duration_s=0.12)
myRobot.eyes.color(255, 0, 0)             # hold on red — hypno eyes
time.sleep(0.8 if hypno else 0.5)
myRobot.eyes.color(0, 255, 200)
myRobot.camera.wiggle(cycles=1, amplitude=100)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 5 — TOO BRIGHT ─────────────────────────────────────────────────────
print("Scene 5 — Too Bright")
slate()
myRobot.eyes.color(0, 255, 200)
time.sleep(1.0)
for _ in range(6):
    myRobot.eyes.color(255, 255, 255)
    myRobot.buzzer.beep(freq=2800, duration_s=0.07)
    time.sleep(0.08)
    myRobot.eyes.off()
    time.sleep(0.08)
myRobot.eyes.color(0, 255, 200)
time.sleep(0.8)
myRobot.voice.say("Who, me?", block=True)
myRobot.camera.wiggle(cycles=2, amplitude=120)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()
time.sleep(2.0)

# ── SCENE 6 — BALL DISTRACTION ───────────────────────────────────────────────
# vision.find_color returns a dict {"objects": [...], "color": ...}
# Check result["objects"] — not result.found
print("Scene 6 — Ball Distraction")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.camera.nod(depth=150)
time.sleep(0.8)
myRobot.camera.nod(depth=150)
time.sleep(0.8)
result = myRobot.vision.find_color("red", show=False)
if result["objects"]:
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(0, 255, 0)
    myRobot.camera.center()
    time.sleep(0.2)
    myRobot.move.forward(seconds=1.5, speed=250)
    myRobot.move.left(seconds=0.4, speed=200)
    myRobot.move.forward(seconds=0.8, speed=200)
    myRobot.camera.nod(depth=300)
    time.sleep(0.4)
    myRobot.move.forward(seconds=0.3, speed=150)
    myRobot.camera.nod(depth=300)
    time.sleep(1.0)
    myRobot.move.backward(seconds=2.0, speed=200)
    myRobot.eyes.color(0, 255, 200)
    myRobot.camera.wiggle(cycles=1, amplitude=100)
    myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
else:
    # No ball — chase anyway for the gag
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(0, 255, 0)
    myRobot.move.forward(seconds=1.5, speed=250)
    myRobot.move.left(seconds=0.4, speed=200)
    myRobot.move.forward(seconds=0.8, speed=200)
    myRobot.camera.nod(depth=300)
    time.sleep(0.4)
    myRobot.move.forward(seconds=0.3, speed=150)
    myRobot.camera.nod(depth=300)
    time.sleep(1.5)
    myRobot.move.backward(seconds=2.0, speed=200)
    myRobot.eyes.color(0, 255, 200)
    myRobot.camera.wiggle(cycles=1, amplitude=100)
    myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 7 — TALKING OVER EACH OTHER ───────────────────────────────────────
print("Scene 7 — Talking Over Each Other")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
myRobot.voice.say("And I am Tur—", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 165, 0)
myRobot.camera.glance_left(amplitude=200, hold_s=0.5)
time.sleep(0.8)
myRobot.voice.say("And I am—", block=True)
myRobot.eyes.color(255, 165, 0)
time.sleep(0.5)
time.sleep(0.8)
myRobot.eyes.color(0, 255, 200)
myRobot.voice.say("And I am Turbo!", block=True)
myRobot.camera.wiggle(cycles=2, amplitude=180)
time.sleep(0.3)
myRobot.voice.say("And I—", block=True)
myRobot.eyes.color(255, 0, 0)
myRobot.camera.shake(width=200)
time.sleep(0.5)

# ── WRAP ─────────────────────────────────────────────────────────────────────
cut()                       # cut the final take too
myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
try:
    _sock.close()
except Exception:
    pass
print("Robot 2 outtakes done.")
