"""
ROBOT 1 — AMY (Toni) — MAIN PROMO
Robots face each other as starting position.
  camera.center()       = looking at Ryan (straight ahead)
  camera.glance_left()  = looking toward filming camera / audience
  camera.glance_right() = looking away from both

SYNC: Robot 1 is the LEADER.
  1. Run this notebook on Robot 1 — it prints its IP and waits.
  2. Paste that IP into Robot 2's notebook, then run it.
  3. Press Enter ONCE here — both robots start together.
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
    # `hostname -I` lists every assigned address; works with no network route.
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2)
        ips = [x for x in out.stdout.split() if x.count(".") == 3 and not x.startswith("127.")]
    except Exception:
        pass
    # UDP-connect trick: picks the IP of the default-route (internet-facing) NIC.
    # No packets are actually sent for a UDP connect.
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


# ── SYNC — Robot 1 waits for Robot 2 to connect, then fires both ─────────────
def _sync_leader(port=9877):
    ips = _detect_lan_ips()
    if ips:
        print(f"\n  Robot 1 IP: {ips[0]}")
        if len(ips) > 1:
            print(f"  (other addresses: {', '.join(ips[1:])} — use the one on the same Wi-Fi as Robot 2)")
    else:
        fallback = socket.gethostbyname(socket.gethostname())
        print(f"\n  Robot 1 IP: {fallback}   ⚠ could not detect a LAN IP — if this is 127.x run `hostname -I` in a terminal")
    print(f"  Paste that IP into Robot 2's notebook now, then run it.")
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
    # Keep the connection OPEN — we send a second cue ('DRIFT') the instant Amy
    # says "You may drift", so Robot 2 launches exactly on her word instead of
    # guessing with sleep timers that drift apart over the ~40s intro.
    return conn, s

_conn, _listen_sock = _sync_leader()


def _cue_drift():
    """Signal Robot 2 to drift NOW. Sent as Amy speaks 'You may drift'."""
    try:
        _conn.sendall(b'DRIFT')
    except Exception as e:
        print(f"  (drift cue not sent — Robot 2 will fall back to its timer: {e})")

myRobot = bot(base_speed=300)
myRobot.voice.select("amy")
myRobot.voice.set_volume(90)

# ── WAKE UP — both robots light up together ───────────────────────────────────
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 100, 80)
myRobot.camera.center()
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── INTRO — Amy speaks to audience, nods hello ────────────────────────────────
myRobot.camera.glance_left(amplitude=200, hold_s=0.3)
myRobot.camera.nod(depth=180)
myRobot.voice.say("Key ora. I am Toni.", block=True)
myRobot.camera.wiggle(cycles=1, amplitude=100)
time.sleep(0.2)

# ── WAIT — Ryan introduces himself ───────────────────────────────────────────
myRobot.camera.center()
myRobot.eyes.color(0, 200, 255)
myRobot.camera.nod(depth=120)
time.sleep(3.5)

# ── LEAGUE ────────────────────────────────────────────────────────────────────
myRobot.eyes.color(0, 150, 255)
myRobot.camera.glance_left(amplitude=180, hold_s=0.2)
myRobot.camera.nod(depth=150)
myRobot.voice.say("Welcome to the Po Knee key AI Robot League.", block=True)
myRobot.eyes.color(0, 255, 200)
time.sleep(0.15)

myRobot.eyes.color(100, 200, 255)
myRobot.camera.glance_left(amplitude=150, hold_s=0.2)
myRobot.voice.say("A free after-school programme for Wellington secondary school students,", block=True)
myRobot.camera.glance_left(amplitude=120, hold_s=0.2)
myRobot.voice.say("run by Mata moe E, and Victoria University", block=True)
myRobot.camera.center()
time.sleep(0.2)

# ── EXPLORE ───────────────────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(80, 0, 200)
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.camera.glance_left(amplitude=160, hold_s=0.2)
myRobot.camera.nod(depth=150)
myRobot.voice.say("In this programme, you will explore robotics, coding, and artificial intelligence.", block=True)
myRobot.camera.nod(depth=120)
time.sleep(0.15)

# ── CAPABILITIES ──────────────────────────────────────────────────────────────
myRobot.eyes.color(0, 255, 100)
myRobot.voice.say("Our robots can move,", block=False)
myRobot.move.forward(seconds=0.2, speed=150)
myRobot.move.backward(seconds=0.2, speed=150)
time.sleep(0.2)

myRobot.eyes.color(255, 140, 0)
myRobot.voice.say("recognise objects,", block=False)
myRobot.camera.glance_left(amplitude=200, hold_s=0.15)
myRobot.camera.glance_right(amplitude=150, hold_s=0.15)
myRobot.camera.center()
time.sleep(0.2)

myRobot.eyes.color(255, 50, 150)
myRobot.voice.say("and detect colours.", block=False)
for c in [(255,0,0),(0,200,0),(0,80,255),(255,50,150)]:
    myRobot.eyes.color(*c)
    time.sleep(0.18)
time.sleep(0.2)

# ── WELCOME ───────────────────────────────────────────────────────────────────
myRobot.camera.center()
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 210, 0)
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.3)
myRobot.camera.glance_left(amplitude=160, hold_s=0.2)
myRobot.camera.nod(depth=160)
myRobot.voice.say("No experience needed. Everyone is welcome.", block=True)
myRobot.camera.wiggle(cycles=1, amplitude=120)
time.sleep(0.2)

# ── GLANCE AT RYAN ────────────────────────────────────────────────────────────
myRobot.camera.center()
myRobot.eyes.color(0, 200, 255)
myRobot.camera.nod(depth=120)
time.sleep(0.3)

# ── CUE RYAN TO DRIFT ─────────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 255, 255)
myRobot.anim.start_blinking(every_s=1.0, blank_s=0.12)
myRobot.camera.center()
time.sleep(0.4)

myRobot.voice.say("Now.", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.off()
time.sleep(0.35)
myRobot.eyes.color(255, 255, 255)
myRobot.camera.nod(depth=150)
time.sleep(0.25)
_cue_drift()                          # ← Ryan starts drifting on this word
myRobot.voice.say("You may drift.", block=True)
time.sleep(0.4)

# ── AMY'S DANCE — while Turbo drifts (~14 seconds) ───────────────────────────
myRobot.anim.start_blinking(every_s=1.5, blank_s=0.18)

myRobot.eyes.color(255, 0, 100)
myRobot.move.left(seconds=0.3, speed=260)
myRobot.eyes.color(0, 255, 100)
myRobot.move.right(seconds=0.3, speed=260)
myRobot.camera.wiggle(cycles=2, amplitude=180)

myRobot.eyes.color(255, 180, 0)
myRobot.move.turn_left(seconds=0.5, speed=300)
myRobot.eyes.color(0, 180, 255)
myRobot.move.turn_right(seconds=0.5, speed=300)
myRobot.camera.nod(depth=200)

myRobot.eyes.color(255, 0, 255)
myRobot.move.forward(seconds=0.3, speed=260)
myRobot.move.backward(seconds=0.3, speed=260)
myRobot.camera.wiggle(cycles=1, amplitude=220)

for c in [(255,0,0),(255,120,0),(255,255,0),(0,255,0),(0,80,255),(160,0,255)]:
    myRobot.eyes.color(*c)
    time.sleep(0.28)
myRobot.move.turn_left(seconds=0.7, speed=340)

myRobot.eyes.color(0, 255, 200)
myRobot.move.left(seconds=0.35, speed=280)
myRobot.camera.wiggle(cycles=2, amplitude=200)
myRobot.move.right(seconds=0.35, speed=280)

myRobot.eyes.color(255, 255, 255)
myRobot.camera.glance_left(amplitude=200, hold_s=0.5)
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=0.5, blank_s=0.1)
time.sleep(1.2)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()

# ── SIGN OFF ──────────────────────────────────────────────────────────────────
time.sleep(1.5)
myRobot.eyes.color(0, 200, 150)
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)
myRobot.camera.nod(depth=160)
time.sleep(0.8)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
try:
    _conn.close()
    _listen_sock.close()
except Exception:
    pass
print("Robot 1 done.")
