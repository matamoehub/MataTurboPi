# Lesson 20 - Robot League

## Goal
This lesson is a V2 challenge workbook for Robot League practice.

It keeps the important commands in one place so students can test robot actions quickly.

Main areas:

- movement
- drift
- eyes
- camera
- sonar
- vision
- speech
- horn
- music

## Start here

```python
from lesson_header import *

show_v2_status()
myRobot = bot(base_speed=300)
```

Safety stop:

```python
myRobot.stop()
myRobot.move.stop()
```

## Movement

### Basic movement

```python
myRobot.move.forward(seconds=0.5, speed=100)
myRobot.move.backward(seconds=0.5, speed=100)
myRobot.move.left(seconds=0.5, speed=100)
myRobot.move.right(seconds=0.5, speed=100)
myRobot.move.turn_left(seconds=0.5, speed=100)
myRobot.move.turn_right(seconds=0.5, speed=100)
```

### Diagonal movement

```python
myRobot.move.diagonal_left(seconds=0.8, speed=100)
myRobot.move.diagonal_right(seconds=0.8, speed=100)
```

## Drift

```python
myRobot.move.drift_left(seconds=1.0, speed=100, turn_blend=0.55)
myRobot.move.drift_right(seconds=1.0, speed=100, turn_blend=0.55)
```

`turn_blend` changes how much turning is mixed into the sideways drift.

## Eyes

### Set both eyes

```python
myRobot.eyes.on()
myRobot.eyes.on(0, 255, 0)
myRobot.eyes.set_both(255, 0, 0)
myRobot.eyes.color(0, 0, 255)
```

### Left and right eye

```python
myRobot.eyes.left(255, 0, 0)
myRobot.eyes.right(0, 0, 255)
```

### Blink and wink

```python
myRobot.eyes.blink_once(blank_s=0.2)
myRobot.eyes.start_blink(every_s=2.0, blank_s=0.3)
myRobot.eyes.stop_blink()
myRobot.eyes.wink(side="left", blank_s=0.4)
```

### Turn eyes off

```python
myRobot.eyes.off()
```

## Camera

### Centre and look

```python
myRobot.camera.center()
myRobot.camera.center_all()
myRobot.camera.look_left(amplitude=250, hold_s=0.15)
myRobot.camera.look_right(amplitude=250, hold_s=0.15)
myRobot.camera.look_up(amplitude=250, hold_s=0.15)
myRobot.camera.look_down(amplitude=250, hold_s=0.15)
```

### Quick camera moves

```python
myRobot.camera.glance_left(amplitude=250, hold_s=0.15)
myRobot.camera.glance_right(amplitude=250, hold_s=0.15)
```

### Camera gestures

```python
myRobot.camera.nod(depth=250, speed_s=0.15)
myRobot.camera.shake(width=250, speed_s=0.15)
myRobot.camera.wiggle(cycles=2, amplitude=200, speed_s=0.12)
myRobot.camera.tiny_wiggle(seconds=2.0, amplitude=90, speed_s=0.12)
```

## Sonar

### Read distance

```python
distance_cm = myRobot.sonar.distance_cm()
distance_mm = myRobot.sonar.distance_mm()
print(distance_cm, distance_mm)
```

### Compare distance

```python
if myRobot.sonar.is_closer_than(20):
    myRobot.stop()
```

If a sonar read fails, the V2 wrapper returns `0`.

## Vision

### Take a picture

```python
picture = myRobot.vision.capture()
print(picture["path"])
```

### Calibrate a colour

```python
myRobot.vision.calibrate_color("green")
myRobot.vision.show_color("green")
```

### Set a colour profile manually

```python
myRobot.vision.set_color_profile(
    "green",
    lower_hsv=(47, 98, 98),
    upper_hsv=(71, 238, 238),
)
```

### Decide where a target is

```python
decision = myRobot.vision.target_position("green", deadzone=50, show=True)
print(decision["direction"])
print(decision["error"])
```

Possible results:

```python
"left"
"right"
"center"
"lost"
```

### Move towards a colour

```python
myRobot.vision.move_towards_color(
    "green",
    sideways_seconds=0.15,
    speed=80,
    deadzone=50,
)
```

### Face, hand, and pose helpers

```python
myRobot.vision.show_faces()
myRobot.vision.show_hands()
myRobot.vision.show_pose()
```

## Speech

### Say text

```python
myRobot.voice.say("Robot League ready")
myRobot.voice.speak("Three, two, one, go")
```

### Volume

```python
myRobot.voice.set_volume(60)
myRobot.voice.get_volume()
```

### Voice choices

```python
myRobot.voice.show_voices()
myRobot.voice.select_voice_number(1)
```

## Horn

```python
myRobot.horn()
myRobot.buzzer.horn()
myRobot.buzzer.beep()
```

## Music

### Play notes

```python
myRobot.buzzer.play_notes("C4:1 D4:1 E4:2", bpm=120)
```

### Play melody in music mode

```python
myRobot.buzzer.play_notes_music_mode("C4 D4 E4 G4", bpm=120)
```

### Play a melody list

```python
myRobot.buzzer.play_melody(["C4", "E4", "G4", "C5"], bpm=120)
```

## Simple Robot League patterns

### Drive forward until close

```python
while not myRobot.sonar.is_closer_than(20):
    myRobot.move.forward(seconds=0.1, speed=80)

myRobot.stop()
```

### Look for a colour, then move

```python
decision = myRobot.vision.target_position("green", deadzone=50, show=True)

if decision["direction"] == "left":
    myRobot.move.left(seconds=0.15, speed=80)
elif decision["direction"] == "right":
    myRobot.move.right(seconds=0.15, speed=80)
elif decision["direction"] == "center":
    myRobot.move.forward(seconds=0.4, speed=100)
else:
    myRobot.stop()
```
