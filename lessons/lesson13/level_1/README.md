# Lesson 13 — Find Colour Balls With V2

## Goal
Use the V2 robot object to help the robot:
1. choose a target colour
2. find the correct ball using vision
3. decide how to move
4. line up and push the ball

This mirrors Lesson 3, but movement and reactions use the V2 namespaces.

## How this lesson works
- you work in a notebook
- you reuse the V2 movement API
- the robot does not move to search unless your logic says it should
- the camera and tracking services help decide movement

## Required flow
### Step 0 — Calibrate colours
Before running the notebook, make sure colour calibration is correct.

### Step 1 — Choose a target colour
Pick one colour and store it.

### Step 2 — Camera scan path
Design how the camera or tracking service should scan.

### Step 3 — Vision output
Use the tracking information to decide left, right, or forward movement.

### Step 4 — Push to goal
Use `myRobot.move` for the push stage.
