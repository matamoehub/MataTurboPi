# Lesson 21 - Pick It Up (Claw Basics)

## Goal
Learn to drive the optional **robotic arm / claw** (the "ultimate kit" attachment)
with the V2 robot object: check it is working, open and close the gripper, and
grab and place an object.

> **Only some robots have the claw.** On a robot with no arm, every arm command
> is safe — it just does nothing and returns `False`. So you can read and run
> this whole lesson on any robot; you only see movement on an arm robot.

This lesson comes after the movement and sensor lessons. You already know how to
make the robot move and how to read a sensor. Now you control a new part of the
robot: an arm with a gripper on the end.

## Step 0 — Does this robot have a claw? Test it first.
Always start on an arm robot by running the **self-test**. Watch the arm: the
gripper should open, close, and open again, then the arm should raise, lower, and
settle in the middle.

```python
myRobot.arm.test()
```

It prints each step and whether the command was sent. If nothing moves on a robot
that *should* have a claw, check the arm cabling/power with your teacher before
going on.

You can also just ask whether an arm is present:

```python
if myRobot.arm.available:
    print("This robot has a claw!")
else:
    print("No claw on this robot — arm commands will do nothing.")
```

## Key commands

Open and close the gripper (the claw):

```python
myRobot.arm.open_gripper()    # claw opens
myRobot.arm.close_gripper()   # claw closes (grips)
```

Raise and lower the arm:

```python
myRobot.arm.lift_up()         # arm raises
myRobot.arm.lift_down()       # arm lowers
```

Go to a tidy "ready" pose (gripper open, arm at middle height):

```python
myRobot.arm.ready()
```

## Grab and place — the whole thing in one command
`grab()` does the full pick-up for you: **open → lower → close → raise.**
Put a soft/light object just in front of the open claw, then:

```python
myRobot.arm.grab()
```

`place()` puts it back down: **lower → open → raise.**

```python
myRobot.arm.place()
```

## Student challenge
1. Run `myRobot.arm.test()` and confirm every step moves.
2. Use `open_gripper` / `close_gripper` / `lift_up` / `lift_down` one at a time so
   you can see exactly what each does.
3. Place a small object in front of the claw and `grab()` it. Did it hold?
4. Drive a little (`myRobot.move.forward(...)`), then `place()` the object
   somewhere new — you just carried something across the room.
5. **Tuning:** if the claw closes too early or the arm moves before the claw is
   ready, slow the sequence down with `myRobot.arm.grab(settle=1.0)` (a longer
   pause between each step).

## Things to watch
- Only grab **light, soft** objects (a foam ball, a small block). The claw is not
  strong.
- Give each move a moment — real servos are slower than code. `settle` controls
  the pause between the steps of `grab()` / `place()`.
- If "lift up" and "lift down" look swapped on your robot, that is a wiring
  difference — tell your teacher; it is a one-line fix in the arm library.

## Main namespaces
- `myRobot.arm`
- `myRobot.move`
