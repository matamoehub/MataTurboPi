# Lesson 13 — Ball Tracking + Push to Goal V2

Mirror of Lesson 3 using the V2 aggregate robot object API.

Use `myRobot = bot(...)` and the V2 namespaces such as `myRobot.move.forward(...)`, `myRobot.eyes.wink(...)`, and `myRobot.voice.say(...)`.

## V2 Namespace Map
- `myRobot.move` for robot movement
- `myRobot.eyes` for eye colour, blink, and wink
- `myRobot.camera` for nod, shake, left, right, up, and down
- `myRobot.voice` for speech, voice selection, phrase generation, and phrase playback
- `myRobot.buzzer` for beep and note playback
- `myRobot.sonar` / `myRobot.ultra` for distance sensing
- `myRobot.line`, `myRobot.tracking`, `myRobot.avoidance`, and `myRobot.qrcode` for ROS-driven behaviours

## Version Check
Run:
```python
from lesson_header import *
show_v2_versions()
```
