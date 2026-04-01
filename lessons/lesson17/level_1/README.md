# Lesson 17 — Advanced Project: Autonomous Obstacle Course With V2

## Goal
This lesson is for advanced students.

The purpose is not to copy a finished solution.
The purpose is to become confident with the V2 robot object and its sensors, then design stronger control logic.

## V2 namespaces for this lesson
- `myRobot.move`
- `myRobot.eyes`
- `myRobot.voice`
- `myRobot.buzzer`
- `myRobot.camera`
- `myRobot.vision`
- `myRobot.sonar`

## What students should practise
- variables
- functions that return values
- dictionary lookups
- `for` loops
- `while` loops
- `if / elif / else`
- a class to group robot state and behaviour

## Teaching approach
Do not hand students a completed navigation program.
Instead:
1. test each V2 namespace and sensor
2. discuss what information it gives them
3. plan the logic using pseudocode
4. let students build and refine their own solution

## Vision workflow
Use the camera and colour tools as a test-and-decide sensor, not as a finished solution.

Suggested workflow:
1. use `myRobot.vision.capture()` to check that the cups are visible
2. point the camera so the target cup is near the middle of the image
3. run `myRobot.vision.calibrate_color('red')` on the target cup
4. run `myRobot.vision.show_color('red')` to check the highlighted result
5. run `myRobot.vision.which_object('red')` to ask which visible cup is red
6. save that result in a variable and decide what the robot should do next

If the result is poor:
- move a little closer
- center the camera again
- recalibrate and test again
