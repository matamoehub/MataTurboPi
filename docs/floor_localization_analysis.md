# Can TurboPi use SpiderPi's "inverse kinematics" idea to locate & push a coloured object?

**Analysis, 2026-07-25.** Short answer: **the inverse *kinematics* itself doesn't
transfer — TurboPi has wheels, not an arm — but the half of SpiderPi's pipeline
that actually finds the object, its pixel→floor-coordinate transform, transfers
directly and would be a real upgrade over TurboPi's current "strafe until centred"
push loop.**

---

## 1. What SpiderPi actually does

SpiderPi's fetch is two separate stages. People say "inverse kinematics" for the
whole thing, but only the *second* stage is IK:

1. **Localise the object on the ground (vision + geometry).** Turn the pixel
   location of the block into a real-world position in centimetres relative to
   the robot. SpiderPi has **two** implementations of this:

   - **Simple** — `transform.convertCoordinate()`: a single calibrated scalar
     `map_param` (centimetres per pixel) plus a fixed `image_center_distance`
     offset:
     ```python
     x_ = (px - 320) * map_param_                      # lateral cm
     y_ = (240 - py) * map_param_ + image_center_distance  # forward cm
     ```
     This works because SpiderPi's camera looks **steeply down** at a small, near
     workspace, where one cm/pixel scale is "good enough" across the arm's reach.

   - **Proper** — `block_fetch.camera_to_world()`: full **inverse perspective
     mapping**. Back-project the pixel through the camera intrinsics to a ray,
     rotate it into the ground-plane frame, and intersect it with the floor:
     ```python
     ray_cam   = K_inv · [u, v, 1]      # pixel → ray in camera frame
     ray_world = R_inv · ray_cam        # rotate into floor frame
     scale     = t_z / ray_world_z      # intersect ground plane (z = 0)
     world_pt  = scale · ray_world − t   # → (X, Y) on the floor, in cm
     ```
     This is the gold-standard method: it needs the camera intrinsics `K`, the
     camera's orientation `R` and height/position `t`, and it gives the object's
     real floor position **without knowing the object's size**.

2. **Inverse kinematics (arm only).** Feed that `(x, y)` into
   `arm_ik` / `setPitchRangeMoving(...)` to solve the leg/arm joint angles that
   put the gripper on the object.

**Stage 2 does not apply to TurboPi** — a mecanum base has no arm to solve joint
angles for. **Stage 1 is the valuable, transferable part**, and TurboPi is
missing the good version of it.

---

## 2. What TurboPi does today

`vision_lib` (`target_position` / `locate_object`) gives:

- `direction`: `"left" | "center" | "right"` from the pixel offset of the blob
  centroid vs. image centre, within a `deadzone`.
- `error` / `error_norm`: pixel / normalised offset.
- `angle_x_deg`: lateral **angle** using the calibrated intrinsics (good).
- `lateral_cm`: **only if you pass the object's real diameter** —
  `estimate_lateral_cm()` infers depth from the object's *apparent size*
  (`Z = fx · D_real / D_pixels`). No object size → no distance.

The lesson-13 / Robot-League push loop then does **bang-bang control**: look,
strafe a fixed step left/right until the blob is roughly centred, nudge forward,
repeat (up to 8 tries).

**Limitations vs. SpiderPi's stage 1:**
- It never computes **how far away** the object is from geometry — only "which
  side", or a size-dependent guess that breaks for unknown/︎varied objects.
- The push is iterative and can overshoot; it has no single "the ball is 24 cm
  ahead and 6 cm left" answer to drive to.
- It can't reason about **where to push toward** — it only centres and shoves.

---

## 3. Why the geometry matters (and why a single scalar isn't enough for TurboPi)

SpiderPi's simple `map_param` works because its camera stares **down** at a small
patch → nearly orthographic → one cm/pixel is fine everywhere in the patch.

TurboPi's camera looks **roughly forward / slightly down** at a floor that
recedes into the distance. That view has **strong perspective**: cm-per-pixel
near the bottom of the frame is very different from cm-per-pixel near the top.
A single scalar would be badly wrong except at one exact distance. So TurboPi
needs the **proper** method (`camera_to_world`-style IPM), not the scalar one.

Good news: TurboPi **already has the camera intrinsics** (`calibration_param.npz`
→ `k_array`, `d_array`, loaded by `vision_lib.load_calibration()` and
`pixel_to_angle`). What's missing is the **camera pose relative to the floor**
(`R`, `t`) — i.e. the tilt angle and mount height.

---

## 4. Proposed upgrade for TurboPi: `locate_on_floor()`

Add a method that returns the object's **real floor position** relative to the
robot, size-independent:

```python
pos = myRobot.vision.locate_on_floor("red")
# → {"found": True, "forward_cm": 24.3, "lateral_cm": -6.1, "range_cm": 25.1,
#    "bearing_deg": -14.1}
```

Key detail: use the object's **floor-contact pixel** (bottom-centre of the
bounding box), not its centroid — that's the point that actually lies on the
ground plane. (For a ball, the contact point is directly under the centre, so the
bottom of the bbox is correct.)

Two ways to implement it, both classroom-viable:

### Option A — 4-point homography  ★ recommended
Place a coloured marker (or the ball) at four **known** spots on the floor in
front of the robot, record their pixels, and solve once:
```python
H = cv2.getPerspectiveTransform(pixel_pts, floor_cm_pts)   # 3×3, saved to disk
# later, per detection:
X, Y = cv2.perspectiveTransform([[u, v]], H)               # pixel → floor cm
```
- **Pros:** no need to measure camera height or tilt angle; handles perspective
  exactly across the whole floor; ~10 lines; robust; it's literally what
  SpiderPi's calibration is doing, done properly for a forward-looking camera.
- **Cons:** must redo the 4-point capture if the camera tilt is changed (so fix
  the tilt to one "look at the floor" position for the push task).

### Option B — pinhole IPM (SpiderPi's `camera_to_world`)
Reuse the existing intrinsics `K` + a measured camera **height** and **tilt
angle** to build `R`, `t` and back-project to the ground plane.
- **Pros:** reuses `calibration_param.npz`; no floor markers.
- **Cons:** needs the mount height and an accurate tilt-angle→servo-position
  calibration (the tilt joint is the asymmetric one from the URDF); more error-prone
  than the direct homography.

---

## 5. What this unlocks for the push task

With `(forward_cm, lateral_cm)` in hand:

1. **One-shot approach instead of iterate-and-hope.** Strafe `lateral_cm`
   sideways and advance `forward_cm − standoff` to arrive squarely behind the
   ball, then push. Fewer moves, less overshoot, works for any object size.
   (Mecanum makes this natural — it can strafe and advance independently.)

2. **"Where to push it" becomes solvable — the real payoff.** Localise **both**
   the ball *and* the target/goal on the floor. Compute the approach point on the
   **far side of the ball along the ball→goal line**, drive there, and push
   through the ball toward the goal:
   ```
   goal ●───────────● ball ───► approach from HERE, push toward goal
   ```
   The current centre-and-shove can't do this; floor coordinates make it a couple
   of vector operations.

3. **Distance-aware behaviour** — slow down as `range_cm` shrinks, stop at a set
   standoff, report "ball 25 cm away" for lesson feedback.

---

## 6. Requirements & caveats

- **Fixed, known camera tilt** during localisation (pick one "look at the floor"
  pose; both options assume the tilt doesn't move mid-task).
- **Flat floor** and the object's **base visible** in frame (ground-plane
  assumption — same as SpiderPi).
- **Undistort first** — `vision_lib.undistort_frame()` already exists; feed the
  undistorted pixel into the transform.
- **Per-robot calibration** — the homography (or height/tilt) is per-camera; save
  it next to `calibration_param.npz`. A robot with no floor calibration should
  fall back cleanly to the current `target_position` behaviour.
- **Ball radius** — the contact point is below the ball centre; using the bbox
  bottom handles this. For a tall object you'd push its base, not its centre.

---

## 7. Recommendation

Yes — adopt **stage 1** of SpiderPi's approach (the pixel→floor transform), not
the arm IK. Implement `locate_on_floor()` via **Option A (4-point homography)**:
it's the smallest, most robust change, needs only a one-time floor calibration
per robot, and turns the Robot-League push from an iterative shuffle into a
computed "drive behind the ball and push it toward the goal" — which is a
genuinely better, more teachable behaviour.

Rough effort: ~half a day for `locate_on_floor()` + a calibration helper +
a `push_towards(ball, goal)` demo, plus one hardware calibration session per
robot. It layers on top of the existing colour detection and calibration code;
nothing already there has to change.

**Not recommended:** porting the joint-space inverse kinematics — there's no arm,
so it has no meaning on the mecanum base.

---

### Appendix — source pointers
- SpiderPi simple transform: `vendor/hiwonder_spiderpi/spiderpi_sdk/arm_ik_sdk/arm_ik/transform.py` → `convertCoordinate`, `map_param`.
- SpiderPi proper IPM: `vendor/hiwonder_spiderpi/advanced/block_fetch.py` → `camera_to_world`.
- SpiderPi arm IK (not transferable): `.../arm_ik_sdk/arm_ik/inverse_kinematics.py`, `arm_move_ik.py`.
- TurboPi current vision geometry: `common/lib/vision_lib.py` → `pixel_to_angle`, `estimate_lateral_cm`, `target_position`, `locate_object`, `load_calibration`.
