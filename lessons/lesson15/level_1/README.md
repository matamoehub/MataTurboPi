# Lesson 15 — V2 Library Test Bench

## Goal
Use one notebook to confirm that the V2 robot object and its namespaces load and respond correctly.

This mirrors Lesson 5, but the tests are grouped under the V2 object.

## Areas covered
- movement
- eyes
- camera
- speech
- buzzer
- sonar
- infrared and line following
- avoidance
- tracking
- QR code
- animation

## How to use this lesson
1. run the setup cell
2. run `show_v2_status()`
3. run `show_v2_versions()`
4. test one namespace at a time

## Important
Some V2 namespaces wrap ROS services. Those services still need the matching node to be running.
