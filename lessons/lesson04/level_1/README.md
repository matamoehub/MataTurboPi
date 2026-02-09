# Lesson 4 — Animate the Robot (Personality)

## Goal
Give the robot personality using:
- camera input
- eye animation
- small movements
- speech and sound

This lesson is about expression, not navigation.

## How this lesson works
- You will work in a Jupyter notebook
- You will create an animation loop
- You will combine movement, eyes, and speech
- The robot should feel intentional and alive

## Coding concepts you’ll learn (by accident)
- Loops
- Timing (`time.sleep`)
- State (idle vs active)
- Random choice
- Organising behaviour into functions

## What you will do
1) Create an idle animation loop
2) Add small fidget movements
3) Animate eyes (blink, colour, expression)
4) Use speech sparingly to explain state changes
5) Use the camera to trigger reactions

## Suggested behaviours
- Idle: blink + tiny movement
- Curious: eyes change colour, short voice line
- Happy: drift or spin + beep
- Thinking: slow blink, small left/right movement

## Debugging requirement
- Use `print()` to show the current state
- Use speech only when the state changes

## Success criteria
- Robot feels like it has personality
- Animation does not spam speech
- Camera input affects behaviour in some way

## Challenge ideas
- Create 3 moods and switch between them
- Build a 30-second “show mode”
- Make the robot react differently to different visual inputs
