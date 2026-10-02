# Lesson 22 - See It, Grab It (Vision + Claw)

## Goal
Put three skills together into one behaviour: **find a coloured object with the
camera, drive up to it using the distance sensor, and pick it up with the claw.**

This is the "robot fetch" lesson. It builds on:
- **Lesson 13** — using the camera to line up with a coloured object.
- **Lesson 21** — using the claw to grab and place.

> **This lesson needs a robot with the claw** for the final grab. On a robot
> with no claw it will still find the object and drive up to it — it just skips
> the grab and tells you. So the class can follow along on any robot.

## The idea
The robot repeats a simple look-then-act loop:

1. Ask the camera where the object is — `left`, `right`, `center`, or `lost`.
2. If it is off to a side, **strafe** that way a little.
3. If it is centred, check the **distance sensor**:
   - far away → edge **forward**;
   - close enough → **stop and grab**.

## Key commands

Where is the object?
```python
decision = myRobot.vision.target_position("red", deadzone=50, show=True)
decision["direction"]   # "left" / "right" / "center" / "lost"
```

How far is the nearest thing?
```python
myRobot.sonar.distance_cm()   # centimetres, or None if nothing is in range
```

Get ready and grab:
```python
myRobot.arm.ready()   # open claw, arm at middle height
myRobot.arm.grab()    # open -> lower -> close -> raise
```

Check for a claw before using it:
```python
if myRobot.arm.available:
    myRobot.arm.grab()
else:
    print("No claw on this robot.")
```

## The main challenge
Tune two things so the robot ends up right on top of the object before it grabs:

- `GRAB_DISTANCE_CM` — how close to get before grabbing. Too big and it grabs at
  thin air; too small and it bumps the object away first.
- the **strafe** and **forward** move times — smaller for finer aim, bigger to
  line up faster.

## Things to watch
- Use a **light, soft** object the claw can hold.
- The distance sensor reads the *nearest* thing in front — make sure that is
  your object, not a wall or a chair leg.
- Keep the object's colour calibrated (same as Lesson 13) or the camera will
  report `lost`.

## Where this goes next
Right now "close enough" comes from the sonar. A future
`myRobot.vision.locate_on_floor()` could return the exact forward/sideways
distance in centimetres from the camera, so the robot could drive to the object
in **one computed move** instead of edging forward step by step.

## Main namespaces
- `myRobot.vision`
- `myRobot.sonar`
- `myRobot.arm`
- `myRobot.move`
- `myRobot.camera`
