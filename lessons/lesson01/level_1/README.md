# Lesson 1 — Movement Around Cups (Build the Moves Library)

## Goal
Build a reusable `moves.py` library, then use it to drive a robot around cups with style.

This `moves.py` file is used again in Lesson 2 and Lesson 3.

## Coding concepts you’ll learn (by accident)
- Functions
- Variables + parameters (speed, seconds)
- `print()` for debugging
- Using voice as debugging (TTS)
- Reusing code (a library)

## What you will do
1) Open `moves.py`  
2) You are GIVEN working code for:
- `forward()`
- `move_left()` (strafe left — not spin)
- `turn_left()` (rotate left)
- `drift_left()` (diagonal / style move)

3) You must COMPLETE the missing moves:
- `back()`
- `move_right()` (strafe right)
- `turn_right()` (rotate right)
- `drift_right()`

Every move must:
- `print()` what it is doing (eg “Moving left”)
- speak what it is doing (TTS)
- stop safely at the end

4) Run `run_obstacle_course.py` and create your own cup course routine.

## Creativity (Matamoe style)
Going around cups is not meant to look boring.
Add personality:
- drift past cups
- spins before turns
- beeps to celebrate
- “signature move” at the end

## Success criteria
- Your `moves.py` works and is reusable
- Your obstacle routine avoids cups most of the time
- Your robot has style (not just forward/left/forward/left)

## Challenge ideas
- Make a “dance finish” routine
- Do the course in reverse
- Add a beep for each cup passed
- Make it smooth (less jerky movement)
