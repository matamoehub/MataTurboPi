# Lesson 12 — Character Animation (Level 1) With V2

## Goal
Learn the V2 animation flow by combining:
- movement
- eye colour, blink, and wink
- camera gestures
- voice phrases
- buzzer sounds
- horn

This mirrors Lesson 2, but uses the V2 object API instead of separate imports.

## Sync vs Async (important)
- `myRobot.move.forward(...)` is synchronous
- `myRobot.anim.move_async(...)` is asynchronous
- `myRobot.eyes.blink(...)` starts background blinking
- `myRobot.anim.start_fidget(...)` starts background fidgeting
- `myRobot.voice.play_phrase(..., block=True)` is synchronous
- `myRobot.voice.play_phrase(..., block=False)` is asynchronous

## What students practise
1. movement + blinking at the same time
2. voice selection and phrase generation
3. camera gestures and acting
4. building a short character sequence

## Demo in this level
The notebook includes the Turbo demo sequence, then short examples, then a student build area.

## V2 Namespace Map
- `myRobot.move`
- `myRobot.eyes`
- `myRobot.camera`
- `myRobot.voice`
- `myRobot.buzzer`
- `myRobot.anim`
