# Lesson 5 — Library Test Bench

## Goal
Use one notebook to confirm that the robot libraries load and respond correctly.

This lesson is for testing and debugging, not for building a final behaviour.

## Libraries covered
- `robot_moves`
- `eyes_lib`
- `camera_lib`
- `tts_lib`
- `buzzer_lib`
- `sonar_lib` / `ultrasonic_lib`
- `infrared_lib`
- `line_follower_lib`
- `avoidance_lib`
- `tracking_lib`
- `qrcode_lib`
- `student_animation_lib`

## How to use this lesson
1. Run the setup cell.
2. Run `show_lesson_status()`.
3. Run the test snippet for the library you want to check.
4. If a ROS node is not running, the matching client library will usually raise a service timeout.

## Important
- The tracking, QR, avoidance, and ROS line-following libraries are service clients. They expect the matching ROS node to already be running.
- The snippets are designed to be run one cell at a time.
- Most snippets include a stop/cleanup call at the end.

## Success criteria
- Each library imports cleanly.
- Hardware libraries respond as expected.
- ROS service clients can reach their nodes.
- Students have a known-good place to test features before using them in a lesson.
