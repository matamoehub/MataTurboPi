# Lesson 1 — Movement Around Cups (Build the Moves Library)

## Goal
Build a reusable **movement library** inside a Jupyter notebook, then use it to move the robot around cups with style.

The movement functions you create in this lesson will be reused in **Lesson 2** and **Lesson 3**.

## How this lesson works
- You will work in a **Jupyter notebook**
- You will write movement functions in one cell
- You will test and tune them in later cells
- At the end, this notebook becomes the source of truth for your robot’s moves

## Coding concepts you’ll learn (by accident)
- Functions
- Parameters (speed, time)
- `print()` for debugging
- Using voice (TTS) for debugging
- Reusing your own code

## What you are given
In the notebook, you are given working examples for:
- `forward()`
- `move_left()` (strafe left, not spin)
- `turn_left()` (rotate)
- `drift_left()` (diagonal movement)

Each of these:
- prints what it is doing
- speaks what it is doing (TTS)
- stops safely at the end

## What you must complete
You must finish the missing movement functions:
- `back()`
- `move_right()` (strafe)
- `turn_right()` (rotate)
- `drift_right()`

All movement functions must:
1) `print()` the action  
2) speak the action (TTS)  
3) stop the robot safely  

## Activity — Cups obstacle course
Using **cells in the same notebook**:
- write a short movement sequence using your functions
- move around cups placed on the floor
- tune speed and timing values until it works reliably

This is not meant to look boring.

## Creativity (Matamoe style)
Add personality:
- drift past cups
- spins before turns
- beeps or speech at milestones
- a “signature move” at the end

## Success criteria
- All movement functions work
- You can move around cups without touching them most of the time
- Your movement has style, not just correctness
- You understand that this notebook defines your robot’s moves
