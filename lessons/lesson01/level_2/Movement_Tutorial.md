# Movement Tutorial

This tutorial explains how movement works in Level 2 using only these reference moves:

- `forward(...)`
- `move_left(...)`
- `turn_left(...)`
- `drift_left(...)`

The exercise moves are **not** answered here. You will build them in `student_robot_moves.py`.

## Core idea: each wheel gets its own command

Your movement helper sends four wheel values in this order:

- `fl`: front-left wheel
- `fr`: front-right wheel
- `rl`: rear-left wheel
- `rr`: rear-right wheel

Using:

```python
base._spam(fl, fr, rl, rr, seconds)
```

`seconds` is how long that wheel pattern is applied.

Think of each wheel as pushing the robot in a specific direction.  
By combining the four pushes, you create different motion types.

## How signs affect motion

- Positive value (`+s`) means wheel spins one direction.
- Negative value (`-s`) means wheel spins the opposite direction.
- Bigger absolute value means stronger push.

If all wheels have the same sign and magnitude, the robot moves straight.  
If left and right sides oppose each other, the robot rotates.  
If wheels form a criss-cross pattern, the robot strafes (slides sideways).

## 1) Forward

Function:

```python
srm.forward(seconds, speed=None)
```

Pattern:
- `fl=+s, fr=+s, rl=+s, rr=+s`

What happens:
- All four wheels push in the same movement direction.
- Robot translates forward with no intended sideways slide or spin.

## 2) Move Left (strafe)

Function:

```python
srm.move_left(seconds, speed=None)
```

Pattern:
- `fl=-s, fr=+s, rl=+s, rr=-s`

What happens:
- Front-left and rear-right oppose front-right and rear-left.
- The wheel forces cancel most forward/backward push and produce sideways translation.

## 3) Turn Left (rotate in place)

Function:

```python
srm.turn_left(seconds, speed=None)
```

Pattern:
- `fl=-s, fr=+s, rl=-s, rr=+s`

What happens:
- Left wheels push opposite to right wheels.
- Robot spins around its center instead of translating much.

## 4) Drift Left (curved movement)

Function:

```python
srm.drift_left(seconds, speed=None, turn_blend=0.55)
```

How it works:
1. Start from a strafe-left pattern.
2. Add part of a turn-left pattern.
3. `turn_blend` controls how much turning is mixed in.

Interpretation:
- `turn_blend = 0.0` means mostly sideways slide.
- Higher `turn_blend` means stronger curve.
- Too high can turn too much instead of drifting.

## Why this matters for the exercise

To create missing moves, you do not guess random numbers.  
You reuse known patterns and mirror/combine them.

That is the main robotics skill in this level:
- understand wheel patterns,
- transform them,
- test and tune with real robot behavior.
