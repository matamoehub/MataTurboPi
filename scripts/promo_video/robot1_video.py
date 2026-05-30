"""
ROBOT 1 — AMY (Toni) — MAIN PROMO
Robots face each other as starting position.
  camera.center()       = looking at Ryan (straight ahead)
  camera.glance_left()  = looking toward filming camera / audience
  camera.glance_right() = looking away from both

Run this on Robot 1 alongside robot2_video.py on Robot 2.
"""
from lesson_header import *
import time

myRobot = bot(base_speed=300)
myRobot.voice.select("amy")
myRobot.voice.set_volume(90)

print("Robot 1 (Toni/Amy) ready.")
input("Press Enter when Robot 2 is also ready — then press Enter on BOTH at the same time...")
print("3..."); time.sleep(1)
print("2..."); time.sleep(1)
print("1..."); time.sleep(1)
print("GO!")

# ── WAKE UP — both robots light up together ───────────────────────────────────
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 100, 80)              # dim teal — waking up
myRobot.camera.center()                     # face Ryan
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)             # full bright teal
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.5)

# ── INTRO — Amy speaks to audience, nods hello ────────────────────────────────
myRobot.camera.glance_left(amplitude=200, hold_s=0.3)   # look at audience
myRobot.camera.nod(depth=180)
myRobot.voice.say("Key ora. I am Toni.", block=True)
myRobot.camera.wiggle(cycles=1, amplitude=100)
time.sleep(0.2)

# ── WAIT — Ryan introduces himself (robot2 speaks now) ───────────────────────
# Amy watches Ryan with interest
myRobot.camera.center()                     # look at Ryan
myRobot.eyes.color(0, 200, 255)
myRobot.camera.nod(depth=120)              # approving nod at Ryan's intro
time.sleep(3.5)                             # Ryan says "And I am Turbo!" + wiggles

# ── LEAGUE — Amy turns to address audience ───────────────────────────────────
myRobot.eyes.color(0, 150, 255)            # blue — official / proud
myRobot.camera.glance_left(amplitude=180, hold_s=0.2)
myRobot.camera.nod(depth=150)
myRobot.voice.say("Welcome to the Po Knee key AI Robot League.", block=True)
myRobot.eyes.color(0, 255, 200)
time.sleep(0.15)

myRobot.eyes.color(100, 200, 255)          # cyan — informative
myRobot.camera.glance_left(amplitude=150, hold_s=0.2)
myRobot.voice.say("A free after-school programme for Wellington secondary school students,", block=True)
myRobot.camera.glance_left(amplitude=120, hold_s=0.2)
myRobot.voice.say("run by Mata moe E, based at Scots College.", block=True)
myRobot.camera.center()                    # glance back at Ryan
time.sleep(0.2)

# ── EXPLORE — excited, eyes shift to purple ───────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(80, 0, 200)             # purple — curious / exciting
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.25)
myRobot.camera.glance_left(amplitude=160, hold_s=0.2)
myRobot.camera.nod(depth=150)
myRobot.voice.say("In this programme, you will explore robotics, coding, and artificial intelligence.", block=True)
myRobot.camera.nod(depth=120)
time.sleep(0.15)

# ── CAPABILITIES — each one gets its own colour, move and look ───────────────
myRobot.eyes.color(0, 255, 100)            # green — "can move"
myRobot.voice.say("Our robots can move,", block=False)
myRobot.move.forward(seconds=0.2, speed=150)
myRobot.move.backward(seconds=0.2, speed=150)
time.sleep(0.2)

myRobot.eyes.color(255, 140, 0)            # orange — "recognise objects"
myRobot.voice.say("recognise objects,", block=False)
myRobot.camera.glance_left(amplitude=200, hold_s=0.15)
myRobot.camera.glance_right(amplitude=150, hold_s=0.15)  # scan both ways
myRobot.camera.center()
time.sleep(0.2)

myRobot.eyes.color(255, 50, 150)           # pink — "detect colours"
myRobot.voice.say("and detect colours.", block=False)
for c in [(255,0,0),(0,200,0),(0,80,255),(255,50,150)]:
    myRobot.eyes.color(*c)
    time.sleep(0.18)
time.sleep(0.2)

# ── WELCOME — warm gold, look at audience then back at Ryan ──────────────────
myRobot.camera.center()
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 210, 0)            # warm gold — welcoming
myRobot.anim.start_blinking(every_s=2.5, blank_s=0.3)
myRobot.camera.glance_left(amplitude=160, hold_s=0.2)
myRobot.camera.nod(depth=160)
myRobot.voice.say("No experience needed. Everyone is welcome.", block=True)
myRobot.camera.wiggle(cycles=1, amplitude=120)
time.sleep(0.2)

# ── GLANCE AT RYAN — check if he's ready to drift ────────────────────────────
myRobot.camera.center()                    # look straight at Ryan
myRobot.eyes.color(0, 200, 255)
myRobot.camera.nod(depth=120)
time.sleep(0.3)

# ── CUE RYAN TO DRIFT — dramatic white spotlight moment ──────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(255, 255, 255)
myRobot.anim.start_blinking(every_s=1.0, blank_s=0.12)
myRobot.camera.center()                    # hold on Ryan
time.sleep(0.4)

myRobot.voice.say("Now.", block=True)
myRobot.anim.stop_blinking()
myRobot.eyes.off()
time.sleep(0.35)
myRobot.eyes.color(255, 255, 255)
myRobot.camera.nod(depth=150)             # authoritative nod at Ryan
time.sleep(0.25)
myRobot.voice.say("You may drift.", block=True)
time.sleep(0.4)

# ── AMY'S DANCE — while Turbo drifts (~14 seconds) ───────────────────────────
myRobot.anim.start_blinking(every_s=1.5, blank_s=0.18)

# beat 1 — strafe bounce
myRobot.eyes.color(255, 0, 100)
myRobot.move.left(seconds=0.3, speed=260)
myRobot.eyes.color(0, 255, 100)
myRobot.move.right(seconds=0.3, speed=260)
myRobot.camera.wiggle(cycles=2, amplitude=180)

# beat 2 — spin with colour change
myRobot.eyes.color(255, 180, 0)
myRobot.move.turn_left(seconds=0.5, speed=300)
myRobot.eyes.color(0, 180, 255)
myRobot.move.turn_right(seconds=0.5, speed=300)
myRobot.camera.nod(depth=200)

# beat 3 — forward/back bounce
myRobot.eyes.color(255, 0, 255)
myRobot.move.forward(seconds=0.3, speed=260)
myRobot.move.backward(seconds=0.3, speed=260)
myRobot.camera.wiggle(cycles=1, amplitude=220)

# beat 4 — rainbow freeze spin
for c in [(255,0,0),(255,120,0),(255,255,0),(0,255,0),(0,80,255),(160,0,255)]:
    myRobot.eyes.color(*c)
    time.sleep(0.28)
myRobot.move.turn_left(seconds=0.7, speed=340)

# beat 5 — drift tease
myRobot.eyes.color(0, 255, 200)
myRobot.move.left(seconds=0.35, speed=280)
myRobot.camera.wiggle(cycles=2, amplitude=200)
myRobot.move.right(seconds=0.35, speed=280)

# beat 6 — dramatic pose to camera
myRobot.eyes.color(255, 255, 255)
myRobot.camera.glance_left(amplitude=200, hold_s=0.5)   # look at audience
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=0.5, blank_s=0.1)
time.sleep(1.2)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()                    # look back at Ryan finishing

# ── SIGN OFF — warm, satisfied ────────────────────────────────────────────────
time.sleep(1.5)                            # let Ryan finish his sign-off line
myRobot.eyes.color(0, 200, 150)
time.sleep(0.3)
myRobot.eyes.color(0, 255, 200)
myRobot.camera.nod(depth=160)
time.sleep(0.8)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
print("Robot 1 done.")
