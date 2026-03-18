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
- `sonar_lib` / `ultrasonic_lib`

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
