# Lesson 3 — Find Colour Balls (OpenCV + Moves)

## Goal
Using OpenCV and your **movement functions**, make the robot:
1) choose a random target colour
2) find the correct ball using the camera
3) decide which way to move
4) strafe to line up, then move forward to push the ball

There are **three balls on purpose**.

## How this lesson works
- You will work in a Jupyter notebook
- You will reuse the movement functions from Lesson 1
- The robot does NOT move to search
- Only the camera moves during searching

## Coding concepts you’ll learn (by accident)
- Variables
- Lists
- Random choice
- Loops
- Using library return values
- Tuning constants (error margins, distance factors)

## Required flow
### Step 0 — Calibrate colours
Before running the notebook:
- use the robot’s colour calibration tool
- calibrate red, green, and blue

### Step 1 — Choose a target colour
Pick one colour randomly and store it in a variable.

### Step 2 — Camera scan path
Design a scan path using a **3×3 grid mental model**.
The camera starts neutral.
Balls are on the ground, so looking down matters.

### Step 3 — Vision output
Vision gives you:
- direction: left / right / centre
- distance: proxy or metres (depending on the library)

### Step 4 — Movement algorithm
Your code decides:
- how much to strafe left or right
- how far to move forward

You must tune:
- an error margin
- distance thresholds (far / mid / close)

## Debugging requirement
You must use:
- `print()` statements
- spoken output (TTS)

Robot should say:
- chosen colour
- “searching”
- “left”, “right”, “forward”

## Success criteria
- Robot finds the chosen colour reliably
- Robot aligns using strafing, not spinning
- Robot pushes the correct ball toward the goal

## Challenge ideas
- Faster scan paths
- Smoother alignment
- Scan again if confidence is low
