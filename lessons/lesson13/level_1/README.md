# Lesson 13 - Random Colour Ball Push

## Goal
Use the V2 robot object to push a randomly chosen ball.

This lesson comes after basic movement and the distance sensor. Students already know how to make the robot move, and they have used a sensor to make a decision.

Now the camera becomes the sensor. Instead of asking "how far away is the object?", the robot asks "is the chosen ball left, right, centred, or lost?"

This lesson is for students who are still learning Python. Keep the process simple:

1. Calibrate the ball colours.
2. Pick a random target ball.
3. Ask the camera if the ball is left, right, centred, or lost.
4. Write the movement code for each decision.
5. Tweak the movement time until the robot can push any random ball.

## Key commands

Set a colour profile:

```python
myRobot.vision.set_color_profile(
    "green",
    lower_hsv=(47, 98, 98),
    upper_hsv=(71, 238, 238),
)
```

Pick a target:

```python
TARGET_COLOUR = pick_random_ball()
```

Ask the camera where the target is:

```python
decision = myRobot.vision.target_position(TARGET_COLOUR, deadzone=50, show=True)
```

Move based on the decision:

```python
if decision["direction"] == "left":
    myRobot.move.left(seconds=0.15, speed=80)

elif decision["direction"] == "right":
    myRobot.move.right(seconds=0.15, speed=80)

elif decision["direction"] == "center":
    myRobot.move.forward(seconds=0.6, speed=100)
```

The main student challenge is to tune the movement times so the robot lines up without overshooting.
