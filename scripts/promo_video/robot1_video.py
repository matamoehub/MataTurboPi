"""
ROBOT 1 — AMY (Toni) — MAIN PROMO
Run this on Robot 1.
Robot 2 must be running robot2_video.py at the same time.
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

# ── INTRO ────────────────────────────────────────────────────────────────────
myRobot.eyes.off()
time.sleep(0.5)
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
myRobot.camera.center()
time.sleep(0.3)

myRobot.camera.nod(depth=150)
myRobot.voice.say("Key ora. I am Toni.", block=True)
time.sleep(0.4)

# ── LEAGUE DESCRIPTION ───────────────────────────────────────────────────────
myRobot.camera.nod(depth=150)
myRobot.voice.say("Welcome to the Po Knee key AI Robot League.", block=True)
time.sleep(0.2)
myRobot.voice.say("A free after-school programme for Wellington secondary school students,", block=True)
myRobot.camera.glance_left(amplitude=150, hold_s=0.3)
myRobot.voice.say("run by Mata moe E, based at Scots College.", block=True)
time.sleep(0.3)

myRobot.camera.glance_right(amplitude=200, hold_s=0.4)
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 150, 255)
myRobot.anim.start_blinking(every_s=3.0, blank_s=0.3)
time.sleep(0.3)

# ── WHAT YOU WILL LEARN ──────────────────────────────────────────────────────
myRobot.camera.center()
myRobot.voice.say("In this programme, you will explore robotics, coding, and artificial intelligence.", block=True)
myRobot.camera.nod(depth=150)
time.sleep(0.3)

myRobot.eyes.color(0, 255, 200)
myRobot.voice.say("Our robots can move,", block=False)
myRobot.camera.nod(depth=200)
time.sleep(0.8)
myRobot.voice.say("recognise objects,", block=False)
myRobot.camera.glance_left(amplitude=200, hold_s=0.3)
time.sleep(0.8)
myRobot.voice.say("and detect colours.", block=False)
myRobot.camera.glance_right(amplitude=200, hold_s=0.3)
time.sleep(0.8)

# ── WELCOME MESSAGE ──────────────────────────────────────────────────────────
myRobot.camera.center()
myRobot.eyes.color(0, 200, 255)
myRobot.anim.stop_blinking()
myRobot.anim.start_blinking(every_s=2.0, blank_s=0.3)
myRobot.voice.say("No experience needed. Everyone is welcome.", block=True)
myRobot.camera.nod(depth=150)
time.sleep(0.3)

myRobot.camera.glance_right(amplitude=150, hold_s=0.3)
myRobot.camera.nod(depth=150)
time.sleep(0.3)

# ── CUE ROBOT 2 TO DRIFT ─────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.camera.center()
myRobot.eyes.color(255, 255, 255)
time.sleep(0.4)

myRobot.voice.say("Now.", block=True)
time.sleep(0.3)
myRobot.voice.say("You may drift.", block=True)
time.sleep(0.3)

# ── ROBOT 1 MINI DANCE (while Robot 2 drifts) ────────────────────────────────
myRobot.eyes.color(0, 255, 200)
myRobot.anim.start_blinking(every_s=1.5, blank_s=0.2)

myRobot.move.forward(seconds=0.4, speed=200)
myRobot.move.backward(seconds=0.4, speed=200)
myRobot.move.left(seconds=0.3, speed=200)
myRobot.move.right(seconds=0.3, speed=200)
myRobot.camera.wiggle(cycles=2, amplitude=150)

for colour in [(0,255,200),(0,150,255),(128,0,128),(0,255,200)]:
    myRobot.eyes.color(*colour)
    time.sleep(0.4)

myRobot.move.turn_left(seconds=0.4, speed=250)
myRobot.move.turn_right(seconds=0.4, speed=250)
myRobot.camera.wiggle(cycles=2, amplitude=150)

# wait while Robot 2 completes its drift sequence (~14 s)
time.sleep(12.0)

# ── SIGN OFF ─────────────────────────────────────────────────────────────────
myRobot.anim.stop_blinking()
myRobot.eyes.color(0, 255, 200)
myRobot.camera.center()
myRobot.camera.nod(depth=150)
time.sleep(1.0)

myRobot.move.stop()
myRobot.eyes.off()
myRobot.camera.center()
print("Robot 1 done.")
