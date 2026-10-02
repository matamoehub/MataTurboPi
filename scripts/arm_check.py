#!/usr/bin/env python3
"""Quick hardware check for the TurboPi arm / claw (the "ultimate kit").

Run this on an ARM-equipped robot:

    python3 scripts/arm_check.py

Watch the arm while it runs — the gripper/claw should open, close, and open
again, then the arm should raise, lower, and settle at mid height. If a robot
has no arm attachment this prints that and exits without doing anything.

It talks to arm_lib directly (no ROS / camera startup) so it's a fast, focused
check you can run on the one arm robot before a lesson.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "common", "lib"))

import arm_lib  # noqa: E402


def main() -> int:
    arm = arm_lib.get_arm()
    if not arm.available:
        print("No working arm detected:", arm.error())
        print("If this robot has the ultimate-kit claw, check the arm servo "
              "cabling/power and the ARM_LIFT_SERVO_ID / ARM_GRIPPER_SERVO_ID channels.")
        return 1
    result = arm.self_test()
    return 0 if result.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
