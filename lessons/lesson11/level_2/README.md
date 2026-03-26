# Lesson 11 — Level 2 (Two Cups) With V2

## Goal
Use the V2 robot object to plan and drive around two cups.

This mirrors Lesson 1 Level 2, but uses the V2 robot object. In this level you should explicitly switch the movement backend to `student_robot_moves` with `myRobot.use_student_moves()` so students are testing their own movement implementation through the V2 API.

## Focus
- movement timing
- turning control
- using variables and functions
- making a two-cup plan before driving

## Suggested flow
1. test one short movement
2. test one short turn
3. write a simple plan
4. drive around both cups
5. stop safely

## V2 Namespace Map
- `myRobot.move.forward(...)`
- `myRobot.move.turn_left(...)`
- `myRobot.move.turn_right(...)`
- `myRobot.move.drift_left(...)`
- `myRobot.move.drift_right(...)`
- `myRobot.stop()`
