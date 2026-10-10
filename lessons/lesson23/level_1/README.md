# Lesson 23 - Gripper Challenges

A **challenge variant of Lesson 22**. In Lesson 22 you learned how the robot can
*see* an object, drive to it, and pick it up. Here you put that to work: a set of
**gripper tasks to complete**, each one a little harder than the last.

> **You need a robot with the claw** (the "ultimate kit") to actually grab. On a
> claw-less robot the find-and-drive parts still run — the grab is skipped and
> the robot tells you. Always run the warm-up self-test first.

## How to work through this
Each exercise below has a **Goal**, a **Success check**, and starter code in the
notebook with `# TODO` lines for you to change. Get one working before moving to
the next. Tune the numbers (distances, times) until the task succeeds reliably.

---

### Warm-up — Is the claw working?
Run `myRobot.arm.test()` and watch every step move. If nothing moves on an
arm robot, fix that with your teacher before starting the challenges.

### Exercise 1 — Grab on command
**Goal:** place a light object in the open claw, then pick it up and put it back
down on command.
**You use:** `myRobot.arm.ready()`, `.grab()`, `.place()`.
**Success check:** the claw closes on the object, lifts it, and sets it down
again without dropping it.

### Exercise 2 — Find it, drive to it, grab it
**Goal:** the robot finds a coloured object, drives up to it, and grabs it
(this is the core of Lesson 22, now as a timed task).
**You use:** `myRobot.vision.target_position(...)`, `myRobot.sonar.distance_cm()`,
`myRobot.arm.grab()`.
**TODO:** tune `GRAB_DISTANCE_CM` so the claw lands on the object.
**Success check:** from ~40 cm away, the robot ends up holding the object.

### Exercise 3 — Fetch and deliver
**Goal:** grab the object, carry it to a "drop zone", and place it there.
**You use:** Exercise 2 + `myRobot.move.forward(...)` / turns + `myRobot.arm.place()`.
**TODO:** write the drive from the pick-up spot to the drop zone.
**Success check:** the object ends up in the drop zone, released by the claw.

### Exercise 4 — Pick only the target colour
**Goal:** with two or three different-coloured objects in view, grab **only** the
target colour and leave the others.
**You use:** a `TARGET_COLOUR` + Exercise 2's loop.
**TODO:** make sure the robot ignores the wrong colours.
**Success check:** the robot grabs the target colour and does not disturb the
others.

### Stretch — Grab by image position (Hiwonder style)
Hiwonder's own grasping demo does **not** use the distance sensor. It watches how
far **down the camera image** the object is: when the object reaches a set height
near the bottom of the frame, it is close enough to grab.
**Goal:** trigger the grab from the object's vertical pixel position
(`decision["object"]["cy"]`) instead of sonar, and compare which is more reliable.
**TODO:** pick a `PICK_Y` threshold and grab when `cy` passes it.
**Think about:** which trigger copes better when the object is a different size?

---

## Tips
- Only grab **light, soft** objects — the claw is not strong.
- Slow a sequence down with `myRobot.arm.grab(settle=1.0)` if it closes too early.
- Keep your colours calibrated (Lesson 13) or the camera reports `lost`.
- The distance sensor reads the *nearest* thing in front — keep clutter away.

## Main namespaces
- `myRobot.arm`
- `myRobot.vision`
- `myRobot.sonar`
- `myRobot.move`
- `myRobot.camera`
