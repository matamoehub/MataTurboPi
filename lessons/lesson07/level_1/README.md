# Lesson 7 — Advanced Project: Autonomous Obstacle Course

## Goal
Use the same project libraries as Lesson 6, but organise the code in a stronger way.

This lesson is for advanced students who are ready to build a robot that behaves more autonomously and can recover from navigation mistakes.

## Libraries for this lesson
- `student_robot_moves`
- `eyes_lib`
- `tts_lib`
- `buzzer_lib`
- `camera_lib`
- `sonar_lib` / `ultrasonic_lib`

## Advanced coding ideas to include
Students should use:
- variables
- functions that return values
- dictionary lookups
- `for` loops
- `while` loops
- `if / elif / else`
- a class to group robot state and behaviour

## Advanced project idea
The robot is entering a harder obstacle course.
It should:
1. sense distance repeatedly
2. classify situations such as `clear`, `warning`, or `blocked`
3. choose actions from those states
4. recover when it gets too close or makes a poor choice
5. track progress and celebrate passed obstacles

## Required advanced concepts
Students should demonstrate:
- dictionary lookups
- at least two functions that return values
- one class that stores robot state
- one recovery strategy for errors
- repeated decision-making using a loop

## Success criteria
- The robot uses all required libraries.
- The code is structured and reusable.
- The robot makes repeated decisions using sonar information.
- The robot can try a recovery move instead of stopping at the first problem.
