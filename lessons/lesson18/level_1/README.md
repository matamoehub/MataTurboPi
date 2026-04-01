# Lesson 18 - MediaPipe Vision V2

This lesson mirrors Lesson 8 using the V2 robot object.

Use this lesson to practise:
- `myRobot.vision.detect_faces()`
- `myRobot.vision.recognize_hands()`
- `myRobot.vision.detect_pose()`
- combining camera results with robot actions

## Main namespaces
- `myRobot.vision`
- `myRobot.camera`
- `myRobot.voice`
- `myRobot.eyes`
- `myRobot.buzzer`

## Important teaching note
This uses face detection and pose/gesture recognition, not named-person identity recognition.

## Suggested workflow
1. capture one frame
2. detect a face
3. test hand gesture labels
4. test pose labels
5. choose a robot response for one condition
