# Block Coding for TurboPi — Build Plan

**Status:** design only — nothing here is built yet.
**Date:** 2026-10-10
**Repos touched:** `MataTurboPi` (blocks + lesson templates) and `robot_ops_web` (editor page, run/save API).
**Goal:** let younger students program the robot with drag-and-drop blocks, in the browser, served by the robot itself — the same way Jupyter is today — using the existing `myRobot` Python library underneath.

---

## 1. Summary

- **Editor:** [Google Blockly](https://developers.google.com/blockly) (Apache-2.0, open source, web-only). Vendored into `robot_ops_web` so it works offline on the robot hotspot.
- **Blocks are an addition to the Python library, not a replacement.** Every block generates a line of ordinary `myRobot` Python. The generated program runs on the robot exactly like lesson code.
- **Runs on the robot.** `robot_ops_web` gets a new `/blocks` page (next to Jupyter) plus a small run/stop/save API. The robot serves the page, the student's browser does the editing, and the robot runs the Python.
- **Master stop.** A red **STOP** button on every ops_web page (and the teacher's existing **stop all robots** in `robot-console`) stops the motors on the robot. It first kills whatever program is running, blocks or notebook, so nothing can start the motors again (§7).
- **Lessons work like notebooks.** A lesson level can point at a `.blocks.json` template instead of an `.ipynb`. Templates have steps, instructions, starter blocks, a per-step block palette, locked blocks and hints.
- **Student work is saved like notebooks.** The `.blocks.json` file lives in the student's workspace at the same path a notebook would, so the existing restore, sync and protection rules apply unchanged.

**Delivered in two phases:**
- **Phase 1 — Proof of concept (staff only):** editor, run/stop, master stop, lesson templates and saving on the robot, backed up through the existing workspace sync. Tried by staff on proof-of-concept robots; **not used by students**.
- **Phase 2 — Class sync:** make sure block work reliably reaches the class store and teachers across robots and consoles, using `docs/student-work-sync-diagnosis.md` as the source of truth (see §9.2). **Phase 2 is completed before any student release.**

**Not in scope:**
- Replacing Jupyter, or changing the student login.
- **Lesson selection in `robot-console`.** No changes to how teachers pick or assign lessons in the console. Block lessons reach robots in the existing lesson bundle and open from the robot's own **Lessons** page.
- **Robots other than TurboPi** (TonyPi, SpiderPi, MentorPi). The block catalogue, Blocks page and runner are TurboPi-only.
- A cloud/portal block editor (only considered once portal saving is fixed — §9.2).

---

## 2. How it works for a student

1. Student signs in on the robot as today (class, name, PIN).
2. The lesson page lists the class lesson. Python lessons open in Jupyter as now; **block lessons open the Blocks page** in a new tab.
3. The Blocks page shows:
   - **Instructions panel** for the current step (Next / Back between steps)
   - **Block palette** with only the blocks this step needs
   - **Canvas** with any starter blocks already placed
   - **Run** / **Stop** buttons, an **output panel**, and an optional **Show Python** panel
4. Student snaps blocks together and presses **Run**. The robot does it. The block currently running is highlighted; if something fails, that block is marked with a plain-English message.
5. Work saves automatically. Next time they sign in (on any robot), it is restored.

```
 Student laptop (browser)                         TurboPi robot (robot_ops_web, port 80)
 ─────────────────────────                        ──────────────────────────────────────
 open /blocks?lesson=…        ── GET page ──────▶  serves blocks.html + vendored Blockly
                              ◀─ block catalogue ─  from synced common/lib/blocks/
 drag blocks
 autosave (debounced)         ── PUT save ──────▶  atomic write into student workspace
 Run                          ── POST run ──────▶  generated .py → subprocess (robot env)
 output + highlighted block   ◀─ WebSocket ──────  stdout/stderr + block-trace markers
 Stop                         ── POST stop ─────▶  kill process → allstop
```

---

## 3. Components and where they live

| Component | Repo / path | Why there |
|---|---|---|
| Block catalogue (definitions + Python generators + toolbox categories) | `MataTurboPi/common/lib/blocks/` | Versioned with `student_robot_v2`; ships in the lesson bundle robots already pull. API and blocks change in the same commit. |
| Block runtime helper (`blocks_runtime.py`) | `MataTurboPi/common/lib/` | Python side of block programs: robot setup, block-trace markers, friendly errors. |
| Block lesson templates (`*.blocks.json`) | `MataTurboPi/lessons/lessonNN/level_N/` | Same place notebooks live; same bundle, same materialisation. |
| Vendored Blockly | `robot_ops_web/backend/static/vendor/blockly/` | Same pattern as vendored Bootstrap; offline. |
| Blocks page | `robot_ops_web/backend/templates/blocks.html`, `backend/static/blocks.js` | New page next to `lessons.html`. |
| Blocks API routes | `robot_ops_web/backend/routes/blocks.py` | One router per domain, matching existing layout. |
| Run/save logic | `robot_ops_web/backend/services/blocks.py`, `backend/services/blocks_runner.py` | Business logic with no FastAPI deps, matching existing layout. |
| Shared Python environment script | `robot_ops_web/scripts/robot_python_env.sh` | Extracted from `start_jupyter.sh` so Jupyter and block runs share one environment definition. |

---

## 4. MataTurboPi changes

### 4.1 Block catalogue — `common/lib/blocks/`

```
common/lib/blocks/
  manifest.json          # catalogue version, categories, block list, min student_robot_v2 version
  blocks.json            # Blockly JSON block definitions (shape, inputs, colours, tooltips)
  generators.js          # Python generator for each block → myRobot calls
  toolbox.json           # default categories: Move, Eyes, Sound, Sensors, Claw, Camera, Logic, Loops
  README.md              # how to add a block
```

- **Definitions use Blockly's JSON format** (not JS) wherever possible, so they are data and easy to test.
- **Generators are small JS functions** that emit Python. Example:

  | Block | Generates |
  |---|---|
  | `move [forward] for (1) seconds` | `myRobot.move.forward(seconds=1)` |
  | `stop moving` | `myRobot.stop()` |
  | `set eyes to [green]` | `myRobot.eyes.set_both(0, 255, 0)` |
  | `say "…"` | `myRobot.voice.say("…")` |
  | `distance (cm)` | `(myRobot.sonar.distance_cm() or 999)` |
  | `ball [red] is [left]?` | `myRobot.vision.target_position("red")["direction"] == "left"` |
  | `grab with claw` / `put down` / `claw ready` | `myRobot.arm.grab()` / `.place()` / `.ready()` |
  | `test claw` | `myRobot.arm.test()` |
  | `wait (1) seconds` | `time.sleep(1)` |

- **First catalogue (~15 blocks):** move/turn/strafe, stop, wait, eyes colour, say, beep, distance, colour direction, claw grab/place/ready/test, plus Blockly's built-in logic, loops, numbers and text.
- **`None`-safe sensor blocks.** `distance_cm()` returns `None` when nothing is in range; the block maps that to a large number so comparisons never crash a young student's program.
- `manifest.json` records `catalogue_version` and the minimum `student_robot_v2` version it needs, so the page can warn if a robot has an old library.

### 4.2 Block runtime — `common/lib/blocks_runtime.py`

Every generated program has the same shape:

```python
from blocks_runtime import start, blk, finish
myRobot = start()          # creates bot(), prints a "ready" marker
blk("a1b2")                # injected before each statement (block trace)
myRobot.move.forward(seconds=1)
blk("c3d4")
myRobot.arm.grab()
finish()                   # stops motors, prints a "done" marker
```

- **`start()`**: creates the robot the same way `lesson_header.py` does, with the same error handling.
- **`blk(id)`**: prints a one-line marker (e.g. `\x1eBLK a1b2`) so the page can highlight the running block. Blockly supports this directly via the Python generator's `STATEMENT_PREFIX`.
- **Friendly errors**: the program body runs inside a wrapper that catches exceptions and prints a marker with the last block id plus a plain-English message (e.g. "I can't see the red ball", "This robot has no claw"). The raw traceback is kept for the teacher view / logs.
- **`finish()`**: always calls `myRobot.stop()` at the end of a program.

### 4.3 Lesson template format — `*.blocks.json`

A block lesson level points at a template, exactly like a notebook entry:

```json
// lessons/lesson21b/lesson.json
{
  "id": "lesson21b",
  "title": "Lesson 21b — Pick It Up (Blocks)",
  "levels": [
    { "id": "level_1", "title": "Level 1 (Blocks)", "entry": "Lesson21b.blocks.json", "kind": "blocks" }
  ]
}
```

Template file:

```json
{
  "format": "turbopi-blocks-lesson",
  "format_version": 1,
  "template_id": "lesson21b/level_1",
  "template_version": 3,
  "catalogue_min_version": 1,
  "title": "Pick It Up",
  "steps": [
    {
      "id": "step1",
      "instructions": "## 1. Test the claw\nPress **Run** and watch the claw open and close.",
      "toolbox": ["arm_test"],
      "starter": { "blocks": { "...": "Blockly serialized workspace" } },
      "locked": ["when_run"],
      "hint": null,
      "show_python": false
    },
    {
      "id": "step2",
      "instructions": "## 2. Grab and carry\nGrab the object, drive forward, then put it down.",
      "toolbox": ["arm_grab", "arm_place", "move_forward", "wait"],
      "starter": { "blocks": { "...": "'when Run clicked' only" } },
      "locked": ["when_run"],
      "hint": "Grab first, then move, then place.",
      "show_python": true
    }
  ]
}
```

| Jupyter idea | Blocks template field |
|---|---|
| Markdown cells | `steps[].instructions` (Markdown) |
| Sequence of cells | `steps[]` with Next / Back |
| Starter code / `# TODO` | `steps[].starter` (Blockly serialized workspace) |
| Code students shouldn't change | `steps[].locked` (blocks set non-deletable / non-movable) |
| Only the relevant commands | `steps[].toolbox` (per-step palette) |
| Hints | `steps[].hint` |
| See the code | `steps[].show_python` (off for youngest groups) |

- **`kind: "blocks"`** in `lesson.json` is optional; ops_web also detects it from the `.blocks.json` extension.
- Block lessons sit alongside Python lessons. A Python lesson and its block version can be separate lessons (e.g. `lesson21` and `lesson21b`) so classes choose one or the other.
- **Reference copies** of block lessons open view-only (no save, Run allowed).

### 4.4 Authoring templates (tutor mode)

Teachers should not hand-write JSON. The Blocks page gets a **tutor-only template mode** (behind the existing admin token):
- Lay out starter blocks on the canvas for each step
- Type the step instructions
- Tick which blocks appear in the palette and which starter blocks are locked
- **Export** downloads the `.blocks.json`; it is then committed into `MataTurboPi/lessons/…` like any lesson file.

### 4.5 Tests (MataTurboPi)

- Every block in `blocks.json` has a generator; every generator's output is valid Python (generate → `compile()`).
- Golden tests: known block layouts → exact expected Python.
- Every `.blocks.json` template parses, references only blocks that exist in the catalogue, and its starter workspaces load.
- `blocks_runtime` tests with the fake-board/fake-robot pattern already used for `arm_lib`: markers printed in order, `finish()` always stops, errors map to friendly messages.

---

## 5. robot_ops_web changes

### 5.1 The Blocks page

- **Route:** `GET /blocks` in `backend/routes/pages.py`, rendering `templates/blocks.html` (extends `base.html`).
- **Nav:** add `{"id": "blocks", "label": "Blocks", "href": "/blocks"}` to the TurboPi profile's `nav_items` in `backend/robots/turbopi/__init__.py`.
- **Static:** `backend/static/vendor/blockly/` (pinned version, bundled) and `backend/static/blocks.js` (page logic). Cache-bust `blocks.js` with a version query string, like `vision.js`.
- **Query params:** `?file=<workspace-relative path>` for the student's lesson; `?ref=<lesson slug>&level=<id>` for a view-only reference copy.
- The page loads the block catalogue from the robot's synced library (5.3), so updating blocks never needs an ops_web release.

### 5.2 API interface

All endpoints act on **the currently signed-in student** (from the existing single-student session file, `_read_student_session()`). Paths are always workspace-relative and validated to stay inside that student's workspace.

| Method + path | Purpose | Request | Response |
|---|---|---|---|
| `GET /api/blocks/catalogue` | Block definitions, generators, toolbox, version | — | `{ok, catalogue_version, blocks, toolbox, generators_url}` |
| `GET /api/blocks/file?path=…` | Load a student's block lesson (falls back to newest good backup if corrupt) | — | `{ok, doc, revision, recovered_from?}` |
| `PUT /api/blocks/file` | Save | `{path, doc, base_revision}` | `{ok, revision}` or `409 {error:"stale_revision", current_revision}` |
| `GET /api/blocks/versions?path=…` | Local save history | — | `{ok, versions:[{id, saved_at, revision}]}` |
| `POST /api/blocks/restore` | Restore a local version | `{path, version_id}` | `{ok, revision}` |
| `POST /api/blocks/reset-step` | Put a step back to its template starter | `{path, step_id}` | `{ok, revision}` |
| `GET /api/blocks/reference?slug=…&level=…` | View-only template from `lessons_cache` | — | `{ok, doc, read_only:true}` |
| `POST /api/blocks/run` | Run a program | `{path?, step_id?, python}` | `{ok, run_id}` or `409 {error:"already_running"}` / `423 {error:"mode_blocked"}` / `503 {error:"library_not_ready"}` |
| `WS /api/blocks/run/{run_id}/stream` | Live output | — | messages `{type:"stdout"|"stderr"|"block"|"error"|"done", …}` |
| `POST /api/blocks/stop` | Stop the running program | `{run_id?}` | `{ok, stopped, allstop}` |
| `GET /api/blocks/status` | Is something running? | — | `{running, run_id?, started_at?, student?}` |
| `POST /api/blocks/template/export` *(tutor, bearer token)* | Validate + return a template for download | `{doc}` | `{ok, doc}` |

Notes:
- The page sends the **generated Python** with Run (Blockly generates it in the browser). The server re-checks it compiles and saves it beside the blocks file, so what ran is always on disk.
- Stream message for a block highlight: `{type:"block", id:"a1b2"}`; for a friendly failure: `{type:"error", block:"c3d4", message:"This robot has no claw"}`.

### 5.3 Serving the catalogue from the synced library

- `GET /api/blocks/catalogue` reads `common/lib/blocks/` from the **lesson-cache copy first, then `/opt/robot/common/lib`**, the same preference `start_jupyter.sh` uses for `PYTHONPATH`.
- `generators.js` is served from the same place (`GET /blocks/lib/generators.js`) with the catalogue version as a cache-busting query.
- Bundle validation: TurboPi's `bundle_validation.forbidden_paths` doesn't cover `common/lib/blocks/`, so no change is expected. Add `common/lib/blocks/manifest.json` to `required_paths` only once block lessons are live.

### 5.4 Running programs — `services/blocks_runner.py`

**Decision: one subprocess per Run** (not a Jupyter kernel). It keeps Stop hard and reliable and doesn't depend on Jupyter's health.

1. **Shared environment script.** Extract the environment part of `scripts/start_jupyter.sh` (loading `.env`, sourcing ROS Humble + `ros2_ws`, building `PYTHONPATH` from lesson cache then common lib, choosing the Python binary) into `scripts/robot_python_env.sh`. `start_jupyter.sh` sources it, and the runner launches through it. One environment definition, so blocks and notebooks can't drift.
2. **Pre-checks before starting:**
   - **Master mode** allows lesson code (same gate that decides whether Jupyter runs). Otherwise `423 mode_blocked`.
   - **Library health:** read `/tmp/robot/jupyter_lib_health.json` (written by `jupyter_sonar_watchdog`). If unhealthy → `503 library_not_ready` and the page shows "The robot is still getting ready."
   - **Nothing else running** (one block program per robot) → otherwise `409 already_running`.
   - **Free the motors from Jupyter:** call Jupyter's REST API (`GET /api/sessions`, then `DELETE /api/kernels/{id}`) to shut down kernels whose notebook path is inside this student's workspace. (Jupyter has no token, so ops_web can do this locally.)
3. **Start:** write the program to `<workspace>/…/.blocks_run/program.py`, then run `bash scripts/robot_python_env.sh python3 -u program.py` with the lesson folder as working directory, in its **own process group**.
4. **Stream:** read stdout/stderr line by line; turn `\x1eBLK` / error markers into `block` / `error` messages; forward everything else as output. Keep the last ~200 lines so a page reload can catch up.
5. **Stop** (button, timeout, page closed, new run, logout):
   - send `SIGINT` to the process group (lets `finish()` stop the motors), wait ~1 s
   - then `SIGKILL` the group
   - then **always** call the existing `cmd_svc.allstop()`
6. **Time limit:** default 120 s per run (env `BLOCKS_RUN_TIMEOUT_S`), mirroring Jupyter's busy-kernel culling.
7. **Page closed:** when the WebSocket disconnects and doesn't reconnect within ~10 s, stop the run — a robot shouldn't keep driving with nobody watching.
8. **ops_web restart:** on startup, kill any leftover `.blocks_run` process groups and call `allstop`.

No new supervisord program is needed; runs are short-lived children of the web process.

### 5.5 Lesson opening

- `_current_lesson_with_jupyter()` (session.py) and `/api/content/lesson/{slug}/jupyter-url` (lessons.py) currently always return a Jupyter link. Add a sibling helper that looks at the entry:
  - `.ipynb` → `{kind:"jupyter", open_url: <existing Jupyter URL>}`
  - `.blocks.json` → `{kind:"blocks", open_url:"/blocks?file=<workspace-relative path>"}` (or `?ref=…` for reference copies)
- Keep the existing `jupyter_url` field for compatibility; add `open_url` + `kind`. `lessons.js` opens `open_url`.
- `/api/console/snapshot` and `/api/health` gain the same `open_url`/`kind`, so `robot-console` can link to the right editor.
- `/api/lesson/open` still materialises the lesson; it only starts Jupyter when the entry is a notebook.

### 5.6 Tests (robot_ops_web)

- Route smoke tests for every endpoint (matching `tests/test_route_smoke.py`).
- Save: atomic write, revision conflict → 409, corrupt file → recovered from backup, path traversal rejected.
- Runner: fake program that prints markers → correct stream messages; Stop always results in `allstop` (mock `cmd_svc.allstop`); timeout fires; second Run → 409; mode gate → 423; unhealthy library → 503.
- Lesson open: `.blocks.json` entry → `kind:"blocks"`; `.ipynb` unchanged.
- No notebook-execution tests (classroom pytest config excludes them).

---

## 6. Saving student work

### 6.1 Where it lives

```
/opt/robot/students/workspaces/<class>/<student>/lessons/lesson21b/level_1/
  Lesson21b.blocks.json        ← the student's copy (materialised from the template, then edited)
  Lesson21b.py                 ← generated Python of the last save/run (readable by teachers)
  .blocks_history/
    Lesson21b.blocks.json/
      2026-10-10T10-31-02Z_r41.json
      …                        ← last 10 local versions
  .blocks_run/                 ← scratch for the running program (not synced)
```

- **Same path and rules as notebooks.** The template is copied into the workspace on first open by the existing `_copy_lesson_to_workspace` (`_copytree_merge(..., overwrite=False)`), so a student's file is never overwritten by a normal lesson refresh.
- **Teacher reset still works the same.** If a teacher assigns a lesson with `overwrite_student_files`, block files are overwritten exactly like notebooks. Document this for teachers.

### 6.2 The saved document

The student file is the template plus their state:

```json
{
  "format": "turbopi-blocks-lesson",
  "format_version": 1,
  "template_id": "lesson21b/level_1",
  "template_version": 3,
  "revision": 41,
  "saved_at": "2026-10-10T10:31:02Z",
  "current_step": "step2",
  "steps": [
    { "id": "step1", "workspace": { "...": "student blocks" }, "hint_opened": false },
    { "id": "step2", "workspace": { "...": "student blocks" }, "hint_opened": true }
  ]
}
```

Instructions, toolbox and locks come from the template at load time; the student file stores only their blocks and progress per step.

### 6.3 When it saves

| Layer | Trigger | Mechanism |
|---|---|---|
| On the robot | ~2 s after any change (debounced), on step change, on Run | `PUT /api/blocks/file` |
| Backed up off the robot | every 20 min, on logout, on **Sync now** (Phase 2 adds a save-triggered upload, §9.2 step 2.2) | existing `sync_student_work` (whole-workspace zip → `robot-console`) |
| Restored | login on any robot | existing restore (with the RC1 login-race guard already in `routes/student.py`) |

### 6.4 Safeguards

1. **Atomic writes:** write `…tmp`, `fsync`, rename. No half-written or zero-byte files.
2. **Local history:** before each save, keep the previous version in `.blocks_history/` (last 10). The page has an **Earlier versions** menu.
3. **Never silently blank:** if the file won't parse, load the newest good history entry and tell the student; never fall back to the empty template without saying so.
4. **Revision check:** each save sends `base_revision`; a stale tab gets `409` and is asked to reload instead of overwriting newer work.
5. **Template updates:** the file records `template_id` + `template_version`. If the template is newer, the page shows "This lesson was updated — reset this step?"; it never replaces work automatically.
6. **Sync size:** `.blocks_run/` is added to the sync skip list (`_should_skip_synced_name`). `.blocks_history/` is synced (small JSON) so history survives switching robots.

### 6.5 Beyond the robot — handled in Phase 2

Once a block file leaves the robot it travels in the same workspace zip as notebooks, through `robot-console` and `robot-classroom`. The server-side behaviour described in `docs/student-work-sync-diagnosis.md` therefore applies to block work exactly as to notebooks. Phase 1 deliberately doesn't change it; **Phase 2 (§9.2)** covers it.

---

## 7. Master stop

**Requirement:** there is always one button that stops the robot completely, whatever is running on it: a block program, a notebook, or anything else.

### 7.1 Why the current stop isn't enough

`/api/cmd/allstop` (ops_web) and the console's **stop all robots** (`/act/allstop` in `robot-console/app.py`, which calls every robot's `/api/cmd/allstop`) only stop the **motors**. A program that is still running, such as a block loop or a notebook `while True:`, sends the next motor command a moment later and the robot drives off again. A master stop must stop the **programs** first, then the hardware.

### 7.2 Three levels of stop

| Level | Who presses it | Where | What it stops |
|---|---|---|---|
| **Program stop** | Student | Blocks page **Stop** button | That student's block program (§5.4), then `allstop` |
| **Robot master stop** | Anyone at the robot | A big red **STOP** button in the header of **every** ops_web page (in `base.html`, so it appears on Blocks, Lessons, Vision, Home, Admin), plus the **Esc** key on the Blocks page | Everything on that robot (§7.3) |
| **Class master stop** | Teacher | The existing **stop all robots** button in `robot-console` | Everything on every robot in the class, because it calls the same robot endpoint |

### 7.3 What a master stop does on the robot

`POST /api/cmd/allstop` is extended (same URL, so the console button gets the new behaviour automatically). Steps run in this order. Each step runs even if an earlier one fails, and the response reports each one:

1. **Kill the block program**: `SIGKILL` its process group straight away (no grace period, unlike the student Stop).
2. **Interrupt every Jupyter kernel** on the robot through Jupyter's REST API (`POST /api/kernels/{id}/interrupt`). This stops running cells but keeps students' variables. A teacher-only `?kernels=shutdown` option shuts them down fully.
3. **Stop the motors**: the existing `allstop` (in-process with timeout, shell fallback).
4. **Hold the claw where it is**: the claw is *not* opened, so a held object isn't dropped.
5. **Silence sound**: the existing `soundoff`.

**Timing:** steps 1 and 2 don't depend on ROS, so programs stop in well under a second, even if ROS is slow to stop the motors.

**No login needed:** like today's `allstop`, anyone at the robot can press it.

### 7.4 Stop and hold (class pause)

When the teacher presses the class master stop, robots go into a **hold** state, `/tmp/robot/stop_hold.json` with who set it and when:

- Block **Run** returns `423 {error:"robot_on_hold"}`, and the Blocks page shows "Your teacher has paused the robots".
- Notebook cells still run (Jupyter can't be gated), but the motors are stopped again if they're commanded during hold. The robot-side motor wrapper checks the hold flag, so this is enforced inside the library, not just in the UI.
- The teacher releases it from `robot-console` (**Release robots**), or it expires after a configurable time (default 10 min). This avoids a forgotten hold blocking the next class.
- A robot master stop pressed at the robot itself does **not** set a hold; it just stops.

### 7.5 Interface changes

| Endpoint | Change |
|---|---|
| `POST /api/cmd/allstop` (ops_web) | Extended as in §7.3. Optional `{"hold": true, "hold_minutes": 10, "by": "teacher"}`. Returns per-step results: `{ok, steps:{blocks, kernels, motors, sound}, hold}` |
| `POST /api/cmd/release` (ops_web, new) | Clears the hold. Needs the bearer token (console-only) |
| `GET /api/cmd/stop_state` (ops_web, new) | `{hold, set_by, set_at, expires_at}`, so the Blocks page and the console can show it |
| `/act/allstop` (robot-console) | Sends `hold: true`. The UI adds **Release robots** and shows the hold state per robot |

### 7.6 Tests

- With a fake block program looping on motor commands: master stop ends the process and `allstop` is called, and the motors don't restart.
- A busy Jupyter kernel is interrupted (mock the Jupyter API).
- One step failing (e.g. the motor stop timing out) doesn't stop the others; the response shows which step failed.
- During hold: Run → 423; a motor command from the library is refused; release or expiry clears it.
- The header STOP button is on every page template (template smoke test).

## 8. Safety rules (motors and claw)

- **Master stop always works** (§7), from every page and from the teacher's console.
- **Stop always ends in `allstop`**, whatever the cause (button, timeout, error, page closed, logout, restart).
- **One program at a time** per robot; notebook kernels for the same student are shut down before a block run.
- **Time limit** on every run.
- **Master mode respected**: no block runs in remote-control or LLM modes.
- **Arm blocks are safe on claw-less robots** (`arm_lib` already no-ops); the block shows "This robot has no claw" instead of failing.
- The page shows a large, always-visible **Stop** button while running.

---

## 9. Build phases

### 9.1 Phase 1 — Proof of concept (staff only)

| Step | Deliverable | Done when |
|---|---|---|
| 1.0 Master stop | Extended `POST /api/cmd/allstop` (§7.3: kill block program, interrupt Jupyter kernels, stop motors, hold claw, silence sound) and the red **STOP** button in the `base.html` header on every page | A notebook `while True:` drive loop and a block loop both stop, and the motors stay stopped, from the header button and from the console's **stop all robots** button. Done first, before any block can drive a motor. |
| 1.0b Spike | Vendored Blockly page on one robot, 3 hard-coded blocks, Run via subprocess, Stop → allstop | A robot drives and stops from the browser, offline |
| 1.1 Catalogue | `common/lib/blocks/` with ~15 blocks + generator tests | All blocks generate valid Python; golden tests pass |
| 1.2 Runner | `blocks_runner.py`, shared env script, run/stop/status/stream API, block highlighting, friendly errors | Runner tests pass; Stop/timeout/page-close all reach `allstop` on hardware |
| 1.3 Saving | file/versions/restore/reset API, atomic writes, history, revision checks | Save tests pass; work survives logout → login on the same robot and on a second robot behind the same console |
| 1.4 Lessons | template format, steps UI, per-step toolbox, locks, hints, `open_url`/`kind` routing | A `.blocks.json` lesson opens from the Lessons page |
| 1.5 PoC review | Lesson 21b (claw basics) as blocks; tutor template export; staff walkthrough with test accounts | Works end to end on hardware; go/no-go write-up. No students. |

Steps 1.1 and 1.2 can proceed in parallel (MataTurboPi vs robot_ops_web) once the generated-program shape (§4.2) is agreed.

### 9.2 Phase 2 — Class sync

**Goal:** a student's block work (and notebooks, since they share the zip) is never lost or overwritten as it moves robot → console → `robot-classroom` → other consoles. Teachers can see it from `robot-console`.

**Repos:** `robot_ops_web`, `robot-console`, `robot-classroom`.
**Source of truth:** `docs/student-work-sync-diagnosis.md`, root causes RC1–RC7 and recommendations R1–R7.

| Step | Deliverable | Done when |
|---|---|---|
| 2.0 Status check | Re-verify each RC against current code in all three repos. The diagnosis marks R1 (login restore order) and R2/R2b (timestamp-gated copies, manifest reconciliation, console-first uploads) as implemented; confirm them, and confirm what remains of R3–R7. | A short status table (fixed / partly / open) is added to the diagnosis doc |
| 2.1 Restore with missing metadata (from R3) | A backup with no `.matamoe/backup_meta.json` no longer skips restoring `lessons/`; it restores with a visible warning. (Separating work by robot type is out of scope with the other robots.) | A restore of an older or externally made zip still brings back the student's lessons |
| 2.1b Class stop and hold | Console **stop all robots** sends `hold`; **Release robots** button; hold state shown per robot; library motor wrapper refuses motor commands while held (§7.4–7.5) | Teacher can pause the whole class, robots stay stopped until released or the hold expires |
| 2.2 Shorter loss window (RC7.2) | Save-triggered sync: after a block save, upload the workspace once it has been dirty for ~2 min (debounced, never more often), plus on Run and step change. The 20-min periodic sync stays as the backstop. | Switching the robot off at the bell loses at most ~2 min of block work |
| 2.3 Stale robots (RC7.3, RC7.6) | A robot left logged in stops re-publishing once the student has saved newer work elsewhere (`created_at` gate, plus a "superseded" check before upload); after a mid-class reboot, unsynced work is uploaded on the next boot instead of stranded | Two robots logged in as the same student can't ping-pong `latest.zip` |
| 2.4 Size tripwire (R1.4) | The "new zip much smaller than last server copy" guard accounts for `.blocks_history/` and `.blocks_run/` (skipped) | A fresh, empty workspace can't overwrite real block work |
| 2.5 Identity (R5) | One shared name-normalisation rule across robot, console and classroom | "Jack S", "jack s" and "Jack  S" resolve to the same student's work |
| 2.6 Class mirror safety (R6) | Class-mirror loops can't delete classes or revert PIN/roster changes | Removing or corrupting the console's class file can't wipe classroom data |
| 2.7 Console view of block work | `robot-console` versions view lists block files; read-only Blockly view of a student's blocks for teachers. (Lesson selection/assignment is out of scope.) | A teacher can open a student's saved blocks from the console |
| 2.8 Portal (R4), optional | Only if a portal block editor is wanted: portal saves feed the snapshot store, and portal login stops overwriting web work | Portal and robot block work converge instead of overwriting each other |

**End-to-end tests for Phase 2** (scripted, against test instances):
- Save blocks on robot A (console 1) → log in on robot B (console 2) → same work, same revision.
- Console offline while a student saves → console reconnects → work reaches `robot-classroom` and the other consoles.
- Same student logged in on two robots → only the newest save wins; the older one is kept as a version.
- Robot switched off 1 min after a save → work is on the console.

### 9.3 Phase 3 — More lessons

Block versions of Lessons 11, 13 and 22; teacher feedback from the pilot folded into the catalogue and templates.

---

## 10. Open questions

1. **Lesson naming:** separate block lessons (`lesson21b`) or a blocks *level* inside the existing lesson (`lesson21/level_blocks`)? Plan assumes separate lessons; levels would also work with no API change.
2. **Show Python default** per age group — teacher setting in the class, or per template only?
3. **Teacher assignment:** console lesson selection is out of scope, so if a teacher assigns a block lesson through the console today, does the robot materialise it correctly? Check once in step 1.4; if not, students open block lessons from the robot's Lessons page.
4. **Phase 2 ownership:** the sync fixes span three repos. Run them as one workstream, or per repo?

## 11. Risks

| Risk | Mitigation |
|---|---|
| Robot start-up time per Run (creating `bot()`) feels slow | Measure in the Phase 1 spike (step 1.0); if >2–3 s, add a warm worker process per signed-in student later |
| Blocks and library drift apart | Catalogue lives in the same repo as the library; generator golden tests; `manifest.json` min-version check |
| Notebook and block program fight over motors | Shut down the student's Jupyter kernels before a run; one block run at a time |
| Students lose work | Phase 1: atomic writes, local history, revision checks, existing sync. Phase 2: save-triggered sync, stale-robot fencing, identity normalisation |
| Blockly upgrades break saved work | Pin the vendored version; `format_version` in files; upgrade deliberately |
