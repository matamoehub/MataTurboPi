# Movement Tutorial

This guide explains the movement functions you use in `Lesson01_Level2.ipynb`.

## Use this inside Jupyter

In your notebook, run these cells:

```python
from lesson_loader import setup
setup()
```

```python
import student_robot_moves as srm
```

Then run the examples from this tutorial in new code cells.

## Level 2 goal

In Level 2 you are building your own movement library in:

- `lessons/lib/student_robot_moves.py`

You keep these provided functions:
- `forward(...)`
- `move_left(...)`
- `turn_left(...)`

You must implement:
- `backward(...)`
- `move_right(...)`
- `turn_right(...)`

Challenge extension:
- `drift_right(..., turn_blend=...)`

## Wheel model (important)

Your helper uses this order:

- `fl` = front-left wheel
- `fr` = front-right wheel
- `rl` = rear-left wheel
- `rr` = rear-right wheel

When using:

```python
base._spam(fl, fr, rl, rr, seconds)
```

each wheel value is a speed command for that wheel.

### 1) `srm.forward(seconds)`
- Moves the robot straight forward.
- `seconds` means how long to move.

Example:
```python
srm.forward(0.8)  # forward for 0.8 seconds
```

Wheel pattern:
- `fl=+s, fr=+s, rl=+s, rr=+s`

### 2) `srm.move_left(seconds)`
- Strafes left (sideways), no spin.

Example:
```python
srm.move_left(0.6)
```

Wheel pattern:
- `fl=-s, fr=+s, rl=+s, rr=-s`

### 3) `srm.turn_left(seconds)`
- Rotates the robot on the spot to the left.
- `seconds` controls how far it turns.

Example:
```python
srm.turn_left(0.4)
```

Wheel pattern:
- `fl=-s, fr=+s, rl=-s, rr=+s`

### 4) `srm.backward(seconds)` (you implement)
- Moves straight backward.

Example:
```python
srm.backward(0.5)
```

Expected wheel pattern:
- `fl=-s, fr=-s, rl=-s, rr=-s`

### 5) `srm.move_right(seconds)` (you implement)
- Strafes right (sideways), not turning.

Example:
```python
srm.move_right(0.6)
```

Expected wheel pattern:
- `fl=+s, fr=-s, rl=-s, rr=+s`

### 6) `srm.turn_right(seconds)` (you implement)
- Rotates on the spot to the right.

Example:
```python
srm.turn_right(0.4)
```

Expected wheel pattern:
- `fl=+s, fr=-s, rl=+s, rr=-s`

### 7) `srm.drift_right(seconds, speed=None, turn_blend=0.55)` (challenge)
- Drift right in a curve.
- This combines:
1. strafe-right wheel pattern
2. turn-right wheel pattern

Pseudo steps:
1. Compute strafe-right wheel values.
2. Compute turn-right wheel values.
3. Clamp `turn_blend` to `0.0..1.0`.
4. Combine each wheel with: `wheel = strafe + turn_blend * turn`.
5. Call `base._spam(fl, fr, rl, rr, seconds)`.

## Challenge task (Level 2)

1. Implement the 3 TODO functions in `student_robot_moves.py`.
2. Challenge: implement `drift_right(...)` using the pseudo steps above.
3. Run a smoke test in notebook:
```python
srm.backward(0.4)
srm.move_right(0.4)
srm.turn_right(0.4)
srm.drift_right(0.7, turn_blend=0.55)
```
4. Build a two-cup path using your functions.
