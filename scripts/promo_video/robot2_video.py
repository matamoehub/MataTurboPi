"""
ROBOT 2 — RYAN (Turbo) — MAIN PROMO
Robots face each other as starting position.
  camera.center()        = looking at Amy (straight ahead)
  camera.glance_right()  = looking toward filming camera / audience
  camera.glance_left()   = looking away from both

Run this on Robot 2 alongside robot1_video.py on Robot 1.
"""
from lesson_header import *
import time

myRobot = bot(base_speed=300)
myRobot.voice.select("ryan")
#myRobot.voice.set_volume(90)

print("Robot 2 (Turbo/Ryan) ready.")
input("Press Enter when Robot 1 is also ready — then press Enter on BOTH at the same time...")
print("3..."); time.sleep(1)
print("2..."); time.sleep(1)
print("1..."); time.sleep(1)
print("GO!")

# ── WAKE UP — both robots light up together ───────────────────────────────────
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 100, 80)
myRobot.camera.center()                    # face Amy
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.2)
time.sleep(0.5)

# ── WAIT — Amy speaks first, Ryan watches her ─────────────────────────────────
myRobot.camera.center()                    # watching Amy
myRobot.camera.nod(depth=100)             # small acknowledgement nod
time.sleep(2.5)                            # Amy says "Key ora. I am Toni."

# ── INTRO — Ryan responds with energy ────────────────────────────────────────
myRobot.eyes.color(0, 220, 255)
myRobot.camera.wiggle(cycles=2, amplitude=180)
myRobot.voice.say("And I am Turbo!", block=True)
myRobot.camera.glance_right(amplitude=150, hold_s=0.2)  # flash at audience
myRobot.camera.center()                    # back to Amy
time.sleep(0.3)

# ── EXCITED INTRO WIGGLES — Ryan can't contain himself ───────────────────────
myRobot.eyes.color(0, 255, 150)
myRobot.move.forward(seconds=0.15)
myRobot.move.backward(seconds=0.15)
time.sleep(0.2)
myRobot.eyes.color(255, 220, 0)
myRobot.move.left(seconds=0.12)
myRobot.move.right(seconds=0.12)
myRobot.camera.wiggle(cycles=1, amplitude=140)
time.sleep(1.0)

# ── WAIT — Amy talks about the league, Ryan listens ──────────────────────────
myRobot.camera.center()                    # watch Amy
myRobot.eyes.color(0, 200, 255)
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.camera.nod(depth=100)             # nodding along to Amy
time.sleep(2.0)
myRobot.camera.glance_right(amplitude=120, hold_s=0.3)  # glance at audience
myRobot.camera.center()
time.sleep(5.5)                            # Amy finishes league + explore section

# ── ASKING TO DRIFT — Ryan turns to Amy and asks ─────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 165, 0)
myRobot.camera.center()                    # look straight at Amy
myRobot.camera.wiggle(cycles=2, amplitude=200)
myRobot.voice.say("Can I drift yet?", block=True)
myRobot.camera.nod(depth=150)             # pleading nod at Amy
time.sleep(0.3)

# ── WAIT — Amy talks capabilities + welcome, Ryan reacts ─────────────────────
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.camera.center()

# React to "can move"
time.sleep(1.2)
myRobot.move.forward(seconds=0.1)
myRobot.move.backward(seconds=0.1)

# React to colour flash
time.sleep(2.0)
myRobot.camera.wiggle(cycles=1, amplitude=120)

# Nod along to welcome
time.sleep(2.5)
myRobot.camera.nod(depth=120)
time.sleep(1.5)

# ── BUILDING ANTICIPATION — Amy is about to say "You may drift" ──────────────
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.eyes.color(0, 255, 0)
myRobot.camera.center()                    # staring at Amy
myRobot.camera.nod(depth=200)
time.sleep(0.5)
myRobot.camera.nod(depth=250)
time.sleep(0.4)
myRobot.eyes.color(255, 255, 0)
myRobot.camera.wiggle(cycles=2, amplitude=180)
time.sleep(0.4)

# ── AMY SAYS "YOU MAY DRIFT" — Ryan reacts with joy ──────────────────────────
# (Amy's "Now." + "You may drift." takes ~3s including pauses)
myRobot.eyes.color(0, 150, 255)
myRobot.camera.tiny_wiggle(seconds=1.0, amplitude=90)
myRobot.voice.say("You just bring the curiosity!", block=True)
time.sleep(0.2)

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
myRobot.camera.center()                    # face Amy to share the moment
myRobot.camera.wiggle(cycles=4, amplitude=220)
myRobot.buzzer.play_notes("C5:1 E5:1 G5:2", bpm=160)
myRobot.voice.say("Now that is how you drift!", block=True)
time.sleep(0.5)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
print("Robot 2 done.")
