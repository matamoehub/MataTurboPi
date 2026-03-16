# Lesson 6 — Project: Obstacle Course Performance

## Goal
Use the robot libraries to write code for a robot that moves through an obstacle course.

This lesson is about combining everything you already know into one creative project.

## Libraries for this lesson
- `student_robot_moves` as the movement library
- `eyes_lib`
- `tts_lib`
- `buzzer_lib`
- `camera_lib`
- `sonar_lib` / `ultrasonic_lib`

## Coding ideas to include
Students should use:
- variables
- functions
- `for` loops
- `while` loops
- `if / elif / else`
- iteration through an array or list

## Advanced extension
For advanced students, the project should also include:
- dictionary lookups
- functions that return values
- a more object-oriented structure
- logic that can recover from navigation mistakes
- behaviour that gets closer to autonomous decision-making

## Project idea
The robot is entering an obstacle course challenge.
It should:
1. react to the course with eyes, sound, and speech
2. move between obstacles using `student_robot_moves`
3. use sonar distance to decide what to do next
4. count obstacles passed
5. celebrate progress using a loop through eye colours

## Required programming concepts
Students should demonstrate each of these at least twice:
- variables
- functions
- loops
- `if / else`

Advanced students should also demonstrate:
- at least one dictionary used for robot decisions
- at least two functions that return values
- one class that groups robot state and behaviour together
- at least one recovery strategy when navigation goes wrong

## Example idea for iteration
Use a list of eye colours that matches obstacle progress.
- obstacle 1: first colour, flash once
- obstacle 2: second colour, flash twice
- obstacle 3: third colour, flash three times

This shows:
- list indexing
- iteration
- using a variable to control behaviour

## Example idea for a while loop
Use a `while` loop to go around a cup more than once, or to keep moving until sonar says the robot is close.

## Success criteria
- The robot completes a planned obstacle-course sequence.
- The code uses all required libraries.
- The code uses variables, functions, loops, and conditionals multiple times.
- The robot behaviour feels creative, not just functional.

## Advanced success criteria
- The robot can detect a problem and try a recovery move.
- The code is organised so behaviour is easier to extend.
- The robot uses distance information to make repeated decisions instead of following one fixed path.
