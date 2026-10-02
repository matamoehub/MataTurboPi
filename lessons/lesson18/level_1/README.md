# Lesson 18 - Rock Paper Scissors Vision V2

This lesson mirrors Lesson 8 using the V2 robot object.

Use this lesson to practise:
- detecting a player's hand gesture
- mapping camera labels to rock / paper / scissors
- using `myRobot.camera`, `myRobot.eyes`, and `myRobot.voice` as game signals
- deciding how the robot should respond

## Check MediaPipe first
Hand recognition uses Google's **MediaPipe**. These lessons are written for
**mediapipe 0.10.9** (the version on the robots). Run this once at the start to
confirm your robot has it and which features are available:

```python
info = myRobot.vision.mediapipe_info()
print("MediaPipe version:", info["version"])
print("Available:", info["solutions"])
```

You should see `version` `0.10.9` and `hands: True`. If `installed` is `False`,
MediaPipe is missing on that robot — tell your teacher (the gesture game needs
it).

## How to read a hand gesture
`recognize_hands()` returns everything you need for the game:

```python
result = myRobot.vision.recognize_hands()
# result["found"]       -> True if a hand was seen
# result["first_move"]  -> "rock" / "paper" / "scissors" / None
# result["hands"][0]["gesture"]  -> e.g. "fist", "open_palm", "peace"
```

`first_move` already maps the gesture to a game move for you, so the simplest
game just reads `result["first_move"]`.

## Main namespaces
- `myRobot.vision`
- `myRobot.camera`
- `myRobot.voice`
- `myRobot.eyes`
- `myRobot.buzzer`

## Suggested rock-paper-scissors mapping
- `fist` = rock
- `open_palm` = paper
- `peace` = scissors

## Suggested game signals
- blue lights = ready
- yellow lights = draw or retry
- green lights = robot win
- red lights = player win
- camera shake = countdown motion
- speech = tell the player what is happening
