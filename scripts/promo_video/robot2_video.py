"""
ROBOT 2 — RYAN (Turbo) — MAIN PROMO
Run this on Robot 2.
Robot 1 must be running robot1_video.py at the same time.
"""
from lesson_header import *
import time

myRobot = bot(base_speed=300)
myRobot.voice.select("ryan")
myRobot.voice.set_volume(90)

print("Robot 2 (Turbo/Ryan) ready.")
input("Press Enter when Robot 1 is also ready — then press Enter on BOTH at the same time...")
print("3..."); time.sleep(1)
print("2..."); time.sleep(1)
print("1..."); time.sleep(1)
print("GO!")

# ── INTRO ────────────────────────────────────────────────────────────────────
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
myRobot.camera.center()

myRobot.camera.wiggle(cycles=2, amplitude=180)
myRobot.voice.say("And I am Turbo!", block=True)
time.sleep(0.4)

# ── EXCITED INTRO WIGGLES ────────────────────────────────────────────────────
myRobot.move.forward(seconds=0.15)
myRobot.move.backward(seconds=0.15)
time.sleep(0.3)
myRobot.move.left(seconds=0.12)
myRobot.move.right(seconds=0.12)
time.sleep(2.0)

# ── ASKING TO DRIFT ──────────────────────────────────────────────────────────
myRobot.eyes.color(255, 165, 0)
myRobot.camera.wiggle(cycles=2, amplitude=200)
myRobot.voice.say("Can I drift yet?", block=True)
time.sleep(0.4)

# ── BUILDING ANTICIPATION ────────────────────────────────────────────────────
myRobot.eyes.color(0, 255, 200)
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)

myRobot.camera.nod(depth=200)
time.sleep(0.6)
myRobot.camera.nod(depth=150)
time.sleep(0.5)
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)
myRobot.camera.nod(depth=200)
time.sleep(0.6)

myRobot.eyes.color(0, 255, 0)
myRobot.camera.nod(depth=250)
time.sleep(0.4)
myRobot.camera.wiggle(cycles=2, amplitude=180)
time.sleep(0.4)
myRobot.eyes.color(255, 255, 0)
myRobot.camera.nod(depth=250)
time.sleep(0.5)

# ── CALL TO ACTION ───────────────────────────────────────────────────────────
myRobot.eyes.color(0, 150, 255)
myRobot.camera.tiny_wiggle(seconds=1.0, amplitude=90)
myRobot.voice.say("You just bring the curiosity!", block=True)
time.sleep(0.3)

# ── PRE-DRIFT FANFARE ────────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 255, 255)
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)
myRobot.buzzer.play_notes("C4:0.5 E4:0.5 G4:1", bpm=200)
time.sleep(1.5)

# ── DRIFT SEQUENCE ───────────────────────────────────────────────────────────
myRobot.eyes.color(255, 165, 0)
myRobot.move.drift_right(seconds=2.0, speed=380, turn_blend=0.6)
myRobot.eyes.color(255, 0, 0)
myRobot.move.drift_left(seconds=2.0, speed=380, turn_blend=0.6)
myRobot.eyes.color(255, 255, 0)
myRobot.move.drift_right(seconds=1.8, speed=420, turn_blend=0.4)
myRobot.eyes.color(0, 255, 0)
myRobot.move.drift_left(seconds=1.8, speed=420, turn_blend=0.4)
myRobot.eyes.color(0, 100, 255)
myRobot.move.turn_right(seconds=0.8, speed=420)
myRobot.eyes.color(128, 0, 128)
myRobot.move.drift_right(seconds=1.8, speed=350, turn_blend=0.7)
myRobot.move.drift_left(seconds=1.8, speed=350, turn_blend=0.7)
myRobot.move.turn_left(seconds=0.8, speed=420)
myRobot.eyes.color(255, 165, 0)
myRobot.move.drift_right(seconds=1.5, speed=300, turn_blend=0.5)

# ── SIGN OFF ─────────────────────────────────────────────────────────────────
myRobot.move.stop()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.wiggle(cycles=4, amplitude=220)
myRobot.buzzer.play_notes("C5:1 E5:1 G5:2", bpm=160)
myRobot.voice.say("Now that is how you drift!", block=True)
time.sleep(0.5)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
print("Robot 2 done.")
