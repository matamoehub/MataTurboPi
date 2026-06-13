"""
ROBOT 2 — RYAN (Turbo) — MAIN PROMO
Robots face each other as starting position.
  camera.center()        = looking at Amy (straight ahead)
  camera.glance_right()  = looking toward filming camera / audience
  camera.glance_left()   = looking away from both

SYNC: Robot 2 is the FOLLOWER.
  1. Run Robot 1's notebook first — it will print its IP.
  2. Paste that IP below as ROBOT1_IP, then run this notebook.
  3. Wait — Robot 2 connects and waits for Robot 1 to press Enter.

TIMING NOTES:
  Amy's full speech takes ~40s before she cues Ryan to drift.
  If Ryan's lines overlap Amy's, increase the sleep values in the
  waiting sections below. If there are long gaps, decrease them.
"""
from lesson_header import *
import time
import socket

# ── PASTE ROBOT 1'S IP HERE ───────────────────────────────────────────────────
ROBOT1_IP = "192.168.1.100"   # ← change this to Robot 1's IP each session

# ── SYNC — connect to Robot 1 and wait for the GO signal ─────────────────────
def _sync_follower(ip, port=9877):
    print(f"  Connecting to Robot 1 at {ip}...")
    s = socket.socket()
    s.connect((ip, port))
    print(f"  Connected. Waiting for Robot 1 to press Enter...")
    s.recv(10)
    s.close()
    print("  GO!\n")

_sync_follower(ROBOT1_IP)

myRobot = bot(base_speed=300)
myRobot.voice.select("ryan")
#myRobot.voice.set_volume(90)

# ── WAKE UP ───────────────────────────────────────────────────────────────────
# Motor prime runs automatically on first move (built into robot_moves v1.2.0)
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 100, 80)
myRobot.camera.center()
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
time.sleep(0.5)

# ── WAIT — Amy speaks first (~4s for her intro) ───────────────────────────────
myRobot.camera.center()
myRobot.camera.nod(depth=100)             # small greeting nod toward Amy
time.sleep(2.5)

# ── INTRO — Ryan responds with energy ────────────────────────────────────────
myRobot.eyes.color(0, 220, 255)
myRobot.camera.wiggle(cycles=2, amplitude=180)
myRobot.voice.say("And I am Turbo!", block=True)
myRobot.camera.glance_right(amplitude=150, hold_s=0.2)  # flash at audience
myRobot.camera.center()
time.sleep(0.3)

# ── EXCITED WIGGLES ───────────────────────────────────────────────────────────
myRobot.eyes.color(0, 255, 150)
myRobot.move.forward(seconds=0.15)
myRobot.move.backward(seconds=0.15)
time.sleep(0.2)
myRobot.eyes.color(255, 220, 0)
myRobot.move.left(seconds=0.12)
myRobot.move.right(seconds=0.12)
myRobot.camera.wiggle(cycles=1, amplitude=140)
time.sleep(1.0)

# ── WAIT — Amy does her full speech (~28s) ────────────────────────────────────
# Amy covers: league name → programme → company → explore → capabilities → welcome
# Ryan listens and reacts throughout. Adjust sleep values if timing is off.

myRobot.camera.center()
myRobot.eyes.color(0, 200, 255)
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)

# Amy: "Welcome to Po Knee key AI Robot League." (~4s)
myRobot.camera.nod(depth=100)
time.sleep(3.5)

# Amy: "A free after-school programme..." (~4.5s)
myRobot.camera.glance_right(amplitude=120, hold_s=0.4)  # glance at audience
myRobot.camera.center()
myRobot.camera.nod(depth=80)
time.sleep(3.5)

# Amy: "run by Mata moe E, based at Scots College." (~3s)
myRobot.camera.wiggle(cycles=1, amplitude=100)
time.sleep(2.5)
myRobot.camera.center()

# Amy: "In this programme, you will explore..." (~6s)
myRobot.eyes.color(0, 150, 255)
myRobot.camera.nod(depth=120)
time.sleep(3.5)
myRobot.camera.glance_right(amplitude=100, hold_s=0.3)
myRobot.camera.center()
time.sleep(2.0)

# Amy: capabilities — "can move, recognise objects, detect colours" (~4s)
myRobot.eyes.color(0, 255, 150)
myRobot.move.forward(seconds=0.1)        # react to "can move"
myRobot.move.backward(seconds=0.1)
time.sleep(0.5)
myRobot.camera.wiggle(cycles=1, amplitude=120)  # react to colour flash
time.sleep(2.0)

# Amy: "No experience needed. Everyone is welcome." (~5s)
myRobot.eyes.color(255, 210, 0)          # match Amy's warm gold welcome colour
myRobot.camera.center()
myRobot.camera.nod(depth=100)
time.sleep(2.0)
myRobot.camera.wiggle(cycles=1, amplitude=100)
time.sleep(1.5)

# ── RESPOND TO WELCOME — Ryan speaks to audience ─────────────────────────────
# Amy is now glancing at Ryan + doing her dramatic cue setup
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 150, 255)
myRobot.camera.tiny_wiggle(seconds=0.4, amplitude=90)   # was 0.8 — tighter fidget
myRobot.voice.say("You just bring the curiosity!", block=True)

# ── ASKING TO DRIFT — go straight into the question, no dead air ──────────────
# Amy is now saying "Now." and about to say "You may drift."
myRobot.eyes.color(255, 165, 0)
myRobot.camera.center()                  # look straight at Amy
myRobot.camera.wiggle(cycles=1, amplitude=200)   # was cycles=2 — ask sooner
myRobot.voice.say("Can I drift yet?", block=True)
myRobot.camera.nod(depth=150)            # pleading nod at Amy

# ── WAIT FOR AMY'S "You may drift." then go ──────────────────────────────────
# This is the call-and-response buffer: Turbo holds, eager, until Amy gives
# permission. TUNING KNOB → if Turbo drifts BEFORE Amy says "You may drift",
# raise DRIFT_CUE_WAIT; if there's an awkward gap AFTER she says it, lower it.
DRIFT_CUE_WAIT = 0.8
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.eyes.color(0, 255, 0)
myRobot.camera.nod(depth=220)
time.sleep(DRIFT_CUE_WAIT)
myRobot.eyes.color(255, 255, 0)
myRobot.camera.wiggle(cycles=1, amplitude=180)   # was cycles=2; dropped a nod + 2 sleeps

# ── PRE-DRIFT FANFARE ─────────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 255, 255)
myRobot.camera.glance_right(amplitude=200, hold_s=0.3)  # look at audience
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)
myRobot.buzzer.play_notes("C4:0.5 E4:0.5 G4:1", bpm=200)
time.sleep(1.2)

# ── DRIFT SEQUENCE ────────────────────────────────────────────────────────────
myRobot.eyes.color(255, 165, 0)
myRobot.move.drift_right(seconds=2.0, speed=380, turn_blend=1.0)
myRobot.eyes.color(255, 0, 0)
myRobot.move.drift_left(seconds=2.0, speed=380, turn_blend=1.0)
myRobot.eyes.color(255, 255, 0)
myRobot.move.drift_right(seconds=1.8, speed=420, turn_blend=1.0)
myRobot.eyes.color(0, 255, 0)
myRobot.move.drift_left(seconds=1.8, speed=420, turn_blend=1.0)
myRobot.eyes.color(0, 100, 255)
myRobot.move.turn_right(seconds=0.8, speed=420)
myRobot.eyes.color(128, 0, 128)
myRobot.move.drift_right(seconds=1.8, speed=350, turn_blend=1.0)
myRobot.move.drift_left(seconds=1.8, speed=350, turn_blend=1.0)
myRobot.move.turn_left(seconds=0.8, speed=420)
myRobot.eyes.color(255, 165, 0)
myRobot.move.drift_right(seconds=1.5, speed=300, turn_blend=1.0)
myRobot.move.drift_left(seconds=1.5, speed=300, turn_blend=1.0)

# ── SIGN OFF ──────────────────────────────────────────────────────────────────
myRobot.move.stop()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()
myRobot.camera.wiggle(cycles=4, amplitude=220)
myRobot.buzzer.play_notes("C5:1 E5:1 G5:2", bpm=160)
myRobot.voice.say("Now that is how you drift!", block=True)
time.sleep(0.5)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
print("Robot 2 done.")
