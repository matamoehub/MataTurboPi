# Lesson 2 — Line Following (Sensors + Moves Library)

## Goal
Use line sensors together with your **movement functions from Lesson 1** to follow a line.

You are not rewriting movement code.  
You are **reusing it** and adding decision logic.

## How this lesson works
- You will work in a Jupyter notebook
- You will import or re-run the movement function cell from Lesson 1
- You will write a loop that:
  - reads sensors
  - decides what to do
  - calls movement functions

## Coding concepts you’ll learn (by accident)
- `while` loops
- `if / elif / else`
- booleans (`True / False`)
- reading sensor data
- debugging with `print()` and voice

## What you will do
1) Read and `print()` line sensor values so you understand them
2) Decide what sensor patterns mean:
   - on the line
   - drifting left
   - drifting right
   - sharp corner
   - lost line
3) Call your movement functions:
   - `forward()`
   - `move_left()` / `move_right()`
   - optional turns or drift for style
4) Add spoken debug messages:
   - “left”
   - “right”
   - “forward”
   - “sharp left”
   - “lost”

## Important rule
Do not change your movement functions unless they are broken.
This lesson is about **logic**, not movement.

## Success criteria
- Robot follows the line for most of the course
- Robot handles at least one sharp corner
- You can explain what each sensor pattern means

## Challenge ideas
- Two modes: “safe” and “race”
- Speak only when the decision changes
- Count laps and celebrate with sound or speech
