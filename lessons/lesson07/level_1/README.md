# Lesson 7 — Advanced Project: Autonomous Obstacle Course

## Goal
This lesson is for advanced students.

The purpose is not to copy a finished solution.
The purpose is to become confident with the libraries and sensors, then design your own control logic.

Students should test the robot's capabilities first, then build a stronger autonomous obstacle-course program.

## Libraries for this lesson
- `student_robot_moves`
- `eyes_lib`
- `tts_lib`
- `buzzer_lib`
- `camera_lib`
- `vision_lib`
- `sonar_lib` / `ultrasonic_lib`

In this lesson, `camera_lib` is for moving the camera head and `vision_lib` is for OpenCV camera images and colour detection.

## What students should practise
Students should show they can use:
- variables
- functions that return values
- dictionary lookups
- `for` loops
- `while` loops
- `if / elif / else`
- a class to group robot state and behaviour

## Teaching approach for this lesson
Do not hand students a completed navigation program.

Instead:
1. test each library and sensor
2. discuss what information each one gives them
3. plan the logic using pseudocode
4. let students build and refine their own solution

## Camera and OpenCV workflow
Use the camera and OpenCV colour tools as a test-and-decide sensor, not as a finished solution.

Suggested workflow:
1. use `vision_lib.get_vision().capture()` to check that the cups are visible
2. use `cam.center_all()`, `cam.glance_left()`, or `cam.glance_right()` to aim the camera
3. run `calibrate_color('red')` or set a profile with `set_color_profile(...)`
4. run `show_color('red')` to check the highlighted result
5. run `target_position('red', deadzone=50)` to ask if the target is left, right, centre, or lost
6. save that result in a variable and decide what the robot should do next

If the result is poor:
- move a little closer
- center the camera again
- recalibrate and test again

Useful OpenCV helper calls:

```python
vision = vision_lib.get_vision()
vision.capture()
vision.show_color("red")
decision = vision.target_position("red", deadzone=50, show=True)
print(decision["direction"])
```

## Suggested design flow
A strong advanced project might follow this structure:
1. setup robot state and thresholds
2. test sonar repeatedly
3. classify distance into states such as `clear`, `warning`, `blocked`
4. choose an action for each state
5. perform recovery when the robot gets too close or stuck
6. track progress through the course
7. celebrate or report when obstacles are passed

## Questions students should answer before coding
- What sonar value means the path is clear?
- What sonar value means the robot should slow down?
- What sonar value means the robot should back away or turn?
- Which function should return the robot state?
- Which dictionary could map states to colours, sounds, or messages?
- How will the robot know it has passed an obstacle?
- What should recovery look like if the robot makes a mistake?

## Success criteria
- The robot uses the required libraries.
- The student tests the sensors before building the full program.
- The student uses structured logic instead of one long script.
- The student creates their own decision-making flow.
- The student can explain why their algorithm works.
