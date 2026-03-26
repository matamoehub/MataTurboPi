# Lesson 11 — Movement Around Cups With V2

## Goal
Use the V2 robot object to move the robot around cups with control and consistency.

This lesson mirrors Lesson 1, but instead of building movement functions from scratch, students use the standard V2 object:
`myRobot = bot(...)`

## How this lesson works
- You work in a Jupyter notebook
- You create one robot object
- You test movement through the V2 namespace `myRobot.move`
- You tune timing, turning, and drift around cups

## Coding concepts you’ll learn
- variables
- functions
- parameters
- loops
- using a standard library API correctly

## What students are given
The V2 library already provides movement functions such as:
- `myRobot.move.forward()`
- `myRobot.move.backward()`
- `myRobot.move.left()`
- `myRobot.move.right()`
- `myRobot.move.turn_left()`
- `myRobot.move.turn_right()`
- `myRobot.move.drift_left()`
- `myRobot.move.drift_right()`

## What students should practise
- run a tiny movement test plan
- tune drift left and drift right
- write a short plan before driving around the cups
- stop the robot safely with `myRobot.stop()`

## V2 Namespace Map
- `myRobot.move` for movement
- `myRobot.eyes` for eye reactions
- `myRobot.camera` for camera gestures
- `myRobot.voice` for speech
- `myRobot.buzzer` for note playback
- `myRobot.sonar` / `myRobot.ultra` for distance sensing

## Version Check
```python
from lesson_header import *
show_v2_versions()
```
