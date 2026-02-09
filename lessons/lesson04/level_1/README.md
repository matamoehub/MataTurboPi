# Lesson 4 — Animate the Robot (Camera + Eyes + Movement + Speech)

## Goal
Make the robot feel alive like a character.

Use:
- camera (for awareness / triggers)
- eyes (blink, colours, expressions)
- movement (small fidgets, little “car-like” motions)
- speech (Piper)
- optional: beeps/horn

This is not a navigation lesson. It’s performance and personality.

## Coding concepts you’ll learn (by accident)
- Loops
- Timing (`time.sleep`)
- State (idle vs active)
- Random choice
- Organising code into small functions

## What you will do
1) Create an animation loop with “idle behaviour”
2) Add small movements (fidgets) that look intentional
3) Animate eyes:
- blink
- change colour based on mood/state
4) Speak short lines:
- not constantly
- ideally only when state changes
5) Use the camera to trigger changes (examples below)

## Suggested behaviours (pick your own)
- Idle: blink + tiny wiggle + quiet hum/beep
- “Notices something” (camera sees motion/ball/face): eyes change colour, speaks, does a small move
- “Happy”: drift or spin + beep + voice line
- “Thinking”: slow blink pattern + small left/right fidget

## Debugging requirement
- Use `print()` to show your current state
- Use speech to announce state changes (not every loop)

## Success criteria
- Robot feels like it has personality
- Animation loop is stable (doesn’t spam voice)
- Camera influences behaviour in some way

## Challenge ideas
- Create 3 moods (calm, curious, excited) and switch between them
- Add a “show mode” that runs a 30-second routine
- Make it react differently depending on what the camera sees
