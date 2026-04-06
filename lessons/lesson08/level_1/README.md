# Lesson 8 - Rock Paper Scissors Vision

This lesson introduces notebook-friendly MediaPipe vision tools through the V1 library style.

Use this lesson to practise:
- detecting a player's hand gesture
- mapping camera labels to rock / paper / scissors
- using the camera head, sonar lights, and speech as part of the game
- deciding how the robot should respond

## Main libraries
- `vision_lib`
- `camera_lib`
- `tts_lib`
- `eyes_lib`
- `buzzer_lib`

## Suggested rock-paper-scissors mapping
- `fist` = rock
- `open_palm` = paper
- `peace` = scissors

## Suggested game signals
- blue lights = ready
- yellow lights = draw or retry
- green lights = robot win
- red lights = player win
- camera shake = countdown motion
- speech = tell the player what is happening

## Suggested workflow
1. show a ready signal
2. count down with head shake and speech
3. capture the hand gesture
4. map it to rock, paper, or scissors
5. compare moves
6. announce the result with lights and speech
