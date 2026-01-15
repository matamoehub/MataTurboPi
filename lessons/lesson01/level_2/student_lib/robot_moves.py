"""
Lesson 01 – Level 2
Student Robot Movement Library

You are given working LEFT-side movements.
Your task is to complete the missing RIGHT and BACKWARD movements.

Use the left versions as examples.
Test carefully on the real robot.

Inside that file, you will find functions that are **incomplete**.

You must complete the following movements:

- `backward`
- `turn_right`
- `drift_right`
- `move_right`
- `diagonal_right`

---

## How to approach the task

1. **Read the working left-side functions**
   - Look closely at how speed, direction, and duration are used
   - Notice which values are positive and which are negative

2. **Copy the pattern**
   - The right-side movements are mirrors of the left-side ones
   - You do not need to invent new logic

3. **Change only what matters**
   - Direction (positive vs negative)
   - Axis (x, y, or z)
   - Speed values if needed

4. **Test often**
   - Complete one function at a time
   - Run it on the robot before moving on

---

## Testing your code

In the notebook `Lesson01_Level2.ipynb`, you will find cells where you can test each movement.

Only test **one movement at a time**.

If the robot behaves unexpectedly:
- stop the robot
- re-check your values
- try again with smaller speeds or shorter durations

---

## Important rules

- Do **not** change the robot hardware
- Do **not** edit the base robot libraries
- Only edit `student_lib/robot_moves.py`
- Make small changes and test frequently

---

## Activity

Once all movements work:

- Create a movement sequence using:
  - left and right
  - forward and backward
  - diagonal moves
- Try to complete the cups challenge from Level 1
- Make the robot move smoothly and accurately

---

## Why this matters

By completing this lesson, you are learning:

- how software libraries are designed
- how abstraction works
- how robots interpret direction and movement
- how small code changes affect real-world behaviour

This is the same process used by professional robotics teams.

Take your time. Test carefully. Enjoy the build.
"""

import time
from robot_controller_api import send_cmd_vel

# =========================
# PROVIDED (DO NOT CHANGE)
# =========================

def stop():
    send_cmd_vel(0, 0, 0)


def forward(speed=0.3, duration=1.0):
    send_cmd_vel(x=speed, y=0, z=0)
    time.sleep(duration)
    stop()


def turn_left(speed=0.6, duration=0.5):
    send_cmd_vel(x=0, y=0, z=speed)
    time.sleep(duration)
    stop()


def drift_left(x_speed=0.25, z_speed=0.5, duration=1.0):
    send_cmd_vel(x=x_speed, y=0, z=z_speed)
    time.sleep(duration)
    stop()


def move_left(speed=0.3, duration=1.0):
    send_cmd_vel(x=0, y=speed, z=0)
    time.sleep(duration)
    stop()


def diagonal_left(x_speed=0.3, y_speed=0.3, duration=1.0):
    send_cmd_vel(x=x_speed, y=y_speed, z=0)
    time.sleep(duration)
    stop()

# =========================
# TODO: STUDENTS COMPLETE
# =========================

def backward(speed=0.3, duration=1.0):
    """
    TODO:
    - Move the robot backwards
    - Hint: x speed should be negative
    """
    # YOUR CODE HERE
    pass


def turn_right(speed=0.6, duration=0.5):
    """
    TODO:
    - Rotate the robot to the right
    - Hint: z rotation direction is reversed
    """
    # YOUR CODE HERE
    pass


def drift_right(x_speed=0.25, z_speed=0.5, duration=1.0):
    """
    TODO:
    - Drift to the right while moving forward
    - Hint: z rotation should be negative
    """
    # YOUR CODE HERE
    pass


def move_right(speed=0.3, duration=1.0):
    """
    TODO:
    - Move sideways to the right
    - Hint: y direction is reversed
    """
    # YOUR CODE HERE
    pass


def diagonal_right(x_speed=0.3, y_speed=0.3, duration=1.0):
    """
    TODO:
    - Move diagonally forward-right
    - Hint: y direction should be negative
    """
    # YOUR CODE HERE
    pass
