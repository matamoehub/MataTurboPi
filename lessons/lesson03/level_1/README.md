# Lesson 3 — Find Colour Balls + Push Into Goal (OpenCV + Moves)

## Goal
Using your `moves.py` library, make the robot:
1) choose a random target ball colour (red/green/blue)
2) scan with the CAMERA to find the target ball (do not move the robot to search)
3) use OpenCV detection to decide which way to move
4) strafe left/right to line up, then move forward to push the target ball into the goal

There are 3 balls on purpose — you must find the chosen colour, not just any ball.

## Coding concepts you’ll learn (by accident)
- Variables
- Lists
- Random choice
- Loops (`while`)
- Using library functions
- Tuning constants (error margins, distance factors)

## Required flow
### Step 0 — Calibrate colours (tool)
Use the robot’s colour calibration tool first.
You must calibrate **red, green, blue**.

### Step 1 — Choose a target colour
Pick one colour randomly and store it in a variable.

### Step 2 — Camera scan path (3x3 grid thinking)
Design a scan path that covers the area.
Think in a 3×3 grid:
- camera starts neutral, pointing forward
- balls are on the ground, so include “down” views

### Step 3 — Vision returns direction + distance
Vision supplies:
- direction: left / right / centre
- distance: proxy or metres (depends on the library)

### Step 4 — Algorithm decides movement amounts
Your algorithm chooses:
- how long to strafe left/right
- how long to go forward

You must tune:
- an error margin (when is centre “good enough”?)
- distance thresholds (far/mid/close)

## Debugging requirement (must do)
Use both:
- `print()` statements
- TTS speech (Piper)
Robot must say:
- target colour
- “searching”
- “left/right/forward”
This helps you test and tune.

## Success criteria
- Robot reliably finds the chosen colour ball
- Robot lines up using strafe (not turning for alignment)
- Robot pushes the ball towards/into the goal

## Challenge ideas
- Optimise scan path (faster find)
- If confidence is low, scan again before moving
- Smooth approach: smaller, more frequent adjustments
- Fast approach: bigger steps, fewer moves
