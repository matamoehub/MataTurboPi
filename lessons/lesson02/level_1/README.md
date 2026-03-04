# Lesson 2 — Character Animation (Level 1)

## Goal
Learn the animation library by running short examples that combine:
- movement
- eye color/blink/wink
- camera gestures
- voice phrases
- horn

## What students practice
1) Async behavior: robot movement + blinking at the same time
2) Voice selection and phrase generation for re-use
3) Character acting with camera gestures and fidget movement
4) Building a short sequence with a beginning, middle, and ending move

## Demo in this level
The notebook includes this full demo:

`Hi I'm Turbo, I've been asked to talk to you. When all I want to do is drift around corners. *sigh*`

While talking:
- fidgeting runs continuously
- eyes blink every few seconds (500 ms blank)
- camera looks left then right
- camera shakes head on “sigh”

Ending:
- drift left for 1 second
- drift right for 1 second
- use `turn_blend=0.95` for both
