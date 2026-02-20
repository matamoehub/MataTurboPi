# Lesson 01 – Level 2: Build the Robot Movement Library

In this lesson, you are no longer just *using* robot code.

You are **building it**.

This is how real robotics and software teams work.

---

## What you are given

You are provided with a robot that already knows how to:

- move forward
- turn left
- drift left
- move left (sideways)
- move diagonally left

These movements already work and are implemented for you.

---

## Your task

Your job is to complete the rest of the robot’s movement abilities.

You need to open and edit this file:

- `lessons/lib/student_robot_moves.py`

Then return to the notebook and test your moves.

Provided in Level 2:
- `forward(seconds, speed=None)`
- `move_left(seconds, speed=None)`
- `turn_left(seconds, speed=None)`

Students implement:
- `backward(seconds, speed=None)`
- `move_right(seconds, speed=None)`
- `turn_right(seconds, speed=None)`

Challenge extension:
- `drift_right(seconds, speed=None, turn_blend=...)`
- Use pseudo steps in the movement tutorial to combine strafe-right + turn-right.

---

## Movement tutorial

Use this guide while working through Level 2:

- `lessons/lesson01/level_2/Movement_Tutorial.md`
