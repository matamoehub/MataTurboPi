"""
ROBOT 1 — AMY (Toni) — OUTTAKES
Run this on Robot 1. Each scene is self-contained — you can re-run
individual scenes by calling slate() then the scene block.

SYNC: Robot 1 is the LEADER (same handshake as the main video).
  1. Run this on Robot 1 — it prints its IP and waits.
  2. Paste that IP into Robot 2's outtakes notebook, then run it.
  3. Press Enter ONCE here — both robots start the reel together.
"""
from lesson_header import *
import time
import socket
import subprocess


def _detect_lan_ips():
    """Return this robot's real non-loopback IPv4 addresses, best route first.

    Do NOT use socket.gethostbyname(socket.gethostname()) — on the Pi's OS the
    hostname maps to 127.0.1.1 in /etc/hosts, so it returns loopback and Robot 2
    ends up dialling its own machine.
    """
    ips = []
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2)
        ips = [x for x in out.stdout.split() if x.count(".") == 3 and not x.startswith("127.")]
    except Exception:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        primary = probe.getsockname()[0]
        probe.close()
        if primary.count(".") == 3 and not primary.startswith("127."):
            ips = [primary] + [x for x in ips if x != primary]
    except Exception:
        pass
    return ips


# ── SYNC — Robot 1 waits for Robot 2 to connect, then starts BOTH together ────
def _sync_leader(port=9877):
    ips = _detect_lan_ips()
    if ips:
        print(f"\n  Robot 1 IP: {ips[0]}")
        if len(ips) > 1:
            print(f"  (other addresses: {', '.join(ips[1:])} — use the one on the same Wi-Fi as Robot 2)")
    else:
        fallback = socket.gethostbyname(socket.gethostname())
        print(f"\n  Robot 1 IP: {fallback}   ⚠ could not detect a LAN IP — if this is 127.x run `hostname -I` in a terminal")
    print(f"  Paste that IP into Robot 2's outtakes notebook now, then run it.")
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', port))
    s.listen(1)
    print(f"  Waiting for Robot 2 to connect...")
    conn, addr = s.accept()
    print(f"  Robot 2 connected from {addr[0]}. Press Enter to start BOTH robots...")
    input()
    conn.sendall(b'GO')
    print("  GO!\n")
    # Connection kept open — re-use it for scene cues if you ever want them
    # (same pattern as the video's DRIFT/DONE signals).
    return conn, s

_conn, _listen_sock = _sync_leader()

myRobot = bot(base_speed=300)
myRobot.voice.select("amy")
myRobot.voice.set_volume(90)

def cut():
    """The 'that's a cut!' beep that ends a flubbed take — Toy Story style.

    A descending two-tone buzz with red eyes: the clear 'nope, cut it' signal.
    """
    myRobot.move.stop()
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(255, 40, 0)                   # red = cut
    myRobot.buzzer.beep(freq=1500, duration_s=0.10)
    myRobot.buzzer.beep(freq=750,  duration_s=0.28)  # drop down: "ehhh, cut"
    time.sleep(0.3)


_first_slate = True

def slate():
    """Clapper-board between takes: cut the previous flub, then slate the next."""
    global _first_slate
    if not _first_slate:
        cut()                       # end the take we just flubbed
    _first_slate = False
    myRobot.move.stop()
    myRobot.camera.center()
    myRobot.anim.stop_blinking()
    myRobot.eyes.color(255, 255, 255)
    myRobot.buzzer.beep(freq=2000, duration_s=0.15)  # clapper snap
    time.sleep(0.3)
    myRobot.eyes.color(0, 255, 200)
    time.sleep(2.0)

# ── SCENE 1 — LINE FLUB ──────────────────────────────────────────────────────
print("Scene 1 — Line Flub")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.camera.nod(depth=150)
myRobot.voice.say("Key ora. I am... I am...", block=True)
myRobot.camera.glance_left(amplitude=200, hold_s=0.5)
myRobot.camera.glance_right(amplitude=200, hold_s=0.5)
myRobot.camera.center()
time.sleep(1.5)
myRobot.eyes.color(0, 100, 255)
myRobot.camera.shake(width=150)
myRobot.voice.say("Not. A. Word.", block=True)
time.sleep(0.5)

# ── SCENE 2 — RYAN DRIFTS EARLY ─────────────────────────────────────────────
print("Scene 2 — Ryan Drifts Early")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("Welcome to the Po Knee key AI Robot League, where students can—", block=True)
myRobot.camera.glance_right(amplitude=250, hold_s=0.4)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 100, 255)
myRobot.voice.say("Not now.", block=True)
myRobot.camera.glance_right(amplitude=300, hold_s=0.5)
time.sleep(2.0)
myRobot.camera.center()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.shake(width=150)
myRobot.voice.say("Every single time.", block=True)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 3 — RYAN SNORING ───────────────────────────────────────────────────
print("Scene 3 — Ryan Snoring")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("Our robots can move, recognise objects, and detect col—", block=True)
myRobot.camera.glance_right(amplitude=250, hold_s=0.8)
time.sleep(1.0)
myRobot.voice.say("Are you snoring?", block=True)
time.sleep(0.8)
myRobot.camera.center()
myRobot.camera.shake(width=150)
myRobot.voice.say("Unbelievable.", block=True)
time.sleep(0.5)

# ── SCENE 4 — HYPNO EYES ─────────────────────────────────────────────────────
print("Scene 4 — Hypno Eyes")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("No experience needed, all students are welc—", block=True)
myRobot.camera.glance_right(amplitude=250, hold_s=0.3)
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 165, 0)
myRobot.voice.say("Not the hypno eyes!", block=True)
myRobot.camera.left(amplitude=350)
time.sleep(1.5)
myRobot.camera.glance_right(amplitude=200, hold_s=0.5)
myRobot.camera.center()
myRobot.eyes.color(0, 255, 200)
myRobot.voice.say("Thank you.", block=True)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── SCENE 5 — TOO BRIGHT ─────────────────────────────────────────────────────
print("Scene 5 — Too Bright")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("Join us and discover what you can build with—", block=True)
myRobot.camera.glance_right(amplitude=250, hold_s=0.3)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 50, 100)
myRobot.move.backward(seconds=0.3, speed=150)
myRobot.voice.say("Too bright.", block=True)
time.sleep(1.0)
myRobot.camera.center()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.shake(width=150)
myRobot.voice.say("Yes. You.", block=True)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(1.5)

# ── SCENE 6 — BALL DISTRACTION ───────────────────────────────────────────────
print("Scene 6 — Ball Distraction")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("Our robots can recognise objects and respond to what they see in real time,", block=True)
myRobot.voice.say("which means students can build programmes that—", block=True)
time.sleep(0.8)
myRobot.camera.glance_right(amplitude=300, hold_s=1.0)
myRobot.voice.say("Turbo.", block=True)
time.sleep(1.5)
myRobot.camera.glance_right(amplitude=300, hold_s=0.5)
myRobot.voice.say("Turbo.", block=True)
time.sleep(2.0)
myRobot.camera.center()
myRobot.camera.shake(width=150)
myRobot.voice.say("It saw a ball.", block=True)
time.sleep(0.5)

# ── SCENE 7 — TALKING OVER EACH OTHER ───────────────────────────────────────
print("Scene 7 — Talking Over Each Other")
slate()
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.voice.say("Key ora I am—", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 100, 255)
myRobot.camera.glance_right(amplitude=200, hold_s=0.5)
time.sleep(0.8)
myRobot.voice.say("Key ora I—", block=True)
myRobot.eyes.color(255, 165, 0)
time.sleep(0.5)
myRobot.voice.say("You go.", block=True)
time.sleep(1.0)
myRobot.voice.say("Key ora I am—", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 50, 150)
myRobot.camera.shake(width=200)
myRobot.voice.say("Start again.", block=True)
time.sleep(0.5)

# ── WRAP ─────────────────────────────────────────────────────────────────────
cut()                       # cut the final take too
myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
try:
    _conn.close()
    _listen_sock.close()
except Exception:
    pass
print("Robot 1 outtakes done.")
