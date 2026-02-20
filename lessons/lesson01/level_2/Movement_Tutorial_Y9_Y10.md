# Movement Tutorial (Year 9-10)

This guide explains the movement functions you use in `Lesson01_Level2.ipynb`.

## What these functions do

You call functions from `student_robot_moves` like this:

```python
import student_robot_moves as srm
```

### 1) `srm.forward(seconds)`
- Moves the robot straight forward.
- `seconds` means how long to move.

Example:
```python
srm.forward(0.8)  # forward for 0.8 seconds
```

### 2) `srm.turn_left(seconds)`
- Rotates the robot on the spot to the left.
- `seconds` controls how far it turns.

Example:
```python
srm.turn_left(0.4)
```

### 3) `srm.drift_left(seconds, turn_blend=...)`
- Slides left while also turning (curved drift).
- This function has 2 important variables:
1. `seconds`: drift time.
2. `turn_blend`: how strong the turn is.

`turn_blend` guide:
- `0.0` = mostly sideways slide
- `0.5` = medium curve
- `1.0` = strong turning arc

Example:
```python
srm.drift_left(0.9, turn_blend=0.55)
```

### 4) `srm.backward(seconds)`
- Moves straight backward.

Example:
```python
srm.backward(0.5)
```

### 5) `srm.move_right(seconds)`
- Strafes right (sideways), not turning.

Example:
```python
srm.move_right(0.6)
```

### 6) `srm.spin_right(seconds)`
- Rotates on the spot to the right.

Example:
```python
srm.spin_right(0.4)
```

## How to tune movement

Real robots are not identical. Battery level, floor grip, and wheel friction all change movement.

Use this loop:
1. Try one move.
2. Observe what happened.
3. Change one variable.
4. Test again.

For drift tuning, change `turn_blend` first, then fine-tune `seconds`.

## Challenge task (Level 2)

Goal: pass between two cups cleanly with a left drift.

Start from:
```python
drift_seconds = 0.9
turn_blend = 0.55
srm.drift_left(drift_seconds, turn_blend=turn_blend)
```

Try at least 5 settings and record:
- Which settings miss the cups
- Which settings clip a cup
- Which settings are best

Best teams can explain *why* their chosen values worked.
