# Lesson 2 — Line Following (Sensors + Moves Library)

## Goal
Use your `moves.py` library from Lesson 1 plus line sensors to follow a line course.

You are not rewriting movement. You are using your library and adding decision logic.

## Coding concepts you’ll learn (by accident)
- `while` loops
- `if / elif / else`
- booleans (`True` / `False`)
- reading sensor values
- debugging with `print()` + voice

## What you will do
1) Import your movement library:
- `forward()`
- `move_left()` / `move_right()` (strafing corrections)
- optional: turns or drift if you want style

2) Read line sensor values and `print()` them so you understand what each sensor means.

3) Write the logic:
- If centred on the line → move forward
- If the line is left → correct left
- If the line is right → correct right
- If lost → recovery behaviour (small search pattern)

4) Add sharp corner cases:
- sharp left
- sharp right

5) Use TTS to speak decisions (for debugging):
- “left”, “right”, “forward”, “sharp left”, “lost”

## Success criteria
- Robot follows the line around most of the course
- Robot handles at least one sharp corner
- You can explain what sensor patterns mean

## Challenge ideas
- Two modes: “safe” and “race” (a variable changes speed/behaviour)
- Speak only when your decision changes (less noise)
- Count laps and celebrate with beeps
