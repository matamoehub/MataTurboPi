#!/usr/bin/env python3
"""blocks_runtime.py

Python-side support for programs generated from the Blockly block
catalogue (common/lib/blocks/). Every generated program has this shape:

    from blocks_runtime import start, blk, guard
    myRobot = start()
    with guard(myRobot):
        blk("a1b2")
        myRobot.move.forward(seconds=1)
        blk("c3d4")
        myRobot.arm.grab()

- start() creates the robot the same way lesson_header.py does.
- blk(id) prints a one-line marker so the Blocks page (robot_ops_web) can
  highlight the block currently running. Blockly's Python generator emits
  one of these before every top-level statement via STATEMENT_PREFIX.
- guard(robot) is a context manager: on any exception it prints a
  plain-English message tied to the failing block's id (instead of a raw
  traceback reaching the student) and re-raises nothing -- the process
  exits 0 either way, since "the block failed" is a normal lesson outcome,
  not a crash. It ALWAYS calls robot.stop() on the way out, success or not.
  The real traceback still goes to stderr for a teacher to read in the log.

See docs/blockly_blocks_plan.md (4.2) and docs/blockly_build_plan.md (M2).
"""
from __future__ import annotations

import os
import sys
import time
import traceback
from contextlib import contextmanager
from typing import Optional

# The marker prefix uses \x1e (ASCII Record Separator) so it can never be
# confused with ordinary print() output from student code or from myRobot's
# own status lines -- nothing else in this codebase emits that byte.
_MARKER = "\x1e"
_BLOCK_MARKER = f"{_MARKER}BLK "
_ERROR_MARKER = f"{_MARKER}ERR "
_READY_MARKER = f"{_MARKER}READY"
_DONE_MARKER = f"{_MARKER}DONE"

# A block program is a short-lived subprocess that exits right after its
# last command -- unlike a Jupyter kernel, which stays alive indefinitely.
# Some backends (a ROS publish that needs a moment for discovery/delivery,
# a queued hardware write) may not have actually reached the hardware by
# the time the student's last statement returns. Same idea as arm_lib's
# ARM_SEQ_SETTLE_S: a short pause before the process (and its ROS context,
# if any) tears down, so the last command gets a chance to land. 0 disables
# it entirely.
_EXIT_SETTLE_S = float(os.environ.get("BLOCKS_EXIT_SETTLE_S", "0.3"))

_last_block_id: Optional[str] = None


def _friendly_message(exc: Exception) -> str:
    """Map a handful of expected failure modes to plain-English text.
    Anything unrecognised falls back to the exception's own message --
    better than nothing, but the raw traceback is also always printed to
    stderr so a teacher can see the real cause.
    """
    text = str(exc)
    name = type(exc).__name__
    if "arm" in text.lower() and ("not" in text.lower() or "unavailable" in text.lower()):
        return "This robot has no claw."
    if name in ("ConnectionError", "TimeoutError") or "timed out" in text.lower():
        return "The robot lost connection to a sensor or servo. Try again."
    if "camera" in text.lower() or "vision" in text.lower():
        return "I can't see well enough right now. Check the camera and lighting."
    return f"Something went wrong: {text}"


def start(base_speed: float = 300, rate_hz: float = 20, verbose: bool = False):
    """Create and return the shared robot object, printing a ready marker.

    Mirrors lesson_header.py's bot() creation so a block program behaves
    exactly like a notebook cell -- same errors, same degraded behaviour
    when a sensor/attachment isn't present.
    """
    import student_robot_v2 as srv2

    robot = srv2.bot(base_speed=base_speed, rate_hz=rate_hz, verbose=verbose)
    print(_READY_MARKER, flush=True)
    return robot


def blk(block_id: str) -> None:
    """Mark that the block with this id is about to run. Called by the
    generated program before every top-level statement."""
    global _last_block_id
    _last_block_id = block_id
    print(f"{_BLOCK_MARKER}{block_id}", flush=True)


def finish(robot=None) -> None:
    """Stop the robot and print the done marker. Always safe to call, even
    if robot creation failed -- guard() calls this on every exit path.

    Motors are stopped FIRST, with no delay (safety: never hold up the
    stop command). The settle delay comes AFTER that, right before the
    process exits -- giving the student's own last command (e.g. an eye
    colour, a sound) a moment to actually reach the hardware before the
    process (and whatever ROS context/I2C handle it holds) tears down.
    Without this, a block program that only sets eye colour or plays a
    sound can appear to "do nothing": the process exits so quickly after
    the call returns that an async write never lands."""
    if robot is not None:
        try:
            robot.stop()
        except Exception:
            pass
    if _EXIT_SETTLE_S > 0:
        time.sleep(_EXIT_SETTLE_S)
    print(_DONE_MARKER, flush=True)


@contextmanager
def guard(robot=None):
    """Wrap a generated program's body. Catches any exception, reports a
    plain-English message tied to the last block that ran, and always
    stops the robot on the way out -- success, student error (e.g. "no
    claw"), or an unexpected crash all end the same way: motors stopped,
    a done marker printed, process exits 0 (a failed block is a normal
    lesson outcome, not a runner crash)."""
    try:
        yield
    except Exception as exc:
        _report_error(exc)
    finally:
        finish(robot)


def distance_cm_or(robot, default: float = 999) -> float:
    """sensor_distance_cm block helper: distance_cm() returns None when
    nothing is in range; block programs need a plain number to compare
    against, so missing readings become a large "nothing there" value."""
    try:
        value = robot.sonar.distance_cm()
    except Exception:
        return default
    return default if value is None else value


def ball_direction_is(robot, color: str, direction: str) -> bool:
    """sensor_ball_direction_is block helper."""
    try:
        decision = robot.vision.target_position(color, show=False)
    except Exception:
        return direction == "lost"
    return decision.get("direction") == direction


def _report_error(exc: Exception) -> None:
    message = _friendly_message(exc)
    block_id = _last_block_id or ""
    print(f"{_ERROR_MARKER}{block_id}|{message}", flush=True)
    traceback.print_exc(file=sys.stderr)


__all__ = [
    "start",
    "blk",
    "finish",
    "guard",
    "distance_cm_or",
    "ball_direction_is",
]
