# Lesson 8 - Rock Paper Scissors Vision

This lesson introduces notebook-friendly MediaPipe vision tools through the V1 library style.

Use this lesson to practise:
- detecting a player's hand gesture
- mapping camera labels to rock / paper / scissors
- checking whether a player is ready to play
- deciding how the robot should respond

## Main libraries
- `vision_lib`
- `camera_lib`
- `tts_lib`
- `eyes_lib`
- `buzzer_lib`

## Important teaching note
`vision_lib` can detect faces and describe what it sees, but it is not identity-based face login. In this lesson, the main goal is hand gesture recognition for a game.

## Suggested rock-paper-scissors mapping
- `fist` = rock
- `open_palm` = paper
- `peace` = scissors

## Suggested workflow
1. start with one capture and check the camera framing
2. test the hand gesture labels
3. decide how the robot will map those labels to rock, paper, or scissors
4. make the robot announce the player's move
5. add robot choice and winner logic

## Challenge ideas
- Say the player's move out loud
- Make the robot choose its own move
- Keep score for three rounds
- Use face detection as a `ready to play` check
