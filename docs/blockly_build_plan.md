# Block Coding for TurboPi — Build Plan

**Status:** plan only — nothing built yet.
**Date:** 2026-10-10
**Approach:** **Phase 1 is a proof of concept** — built and tried by staff on proof-of-concept robots, **not used by students**. **Phase 2 (class sync) must be complete before any student release.**
**Design:** [`docs/blockly_blocks_plan.md`](blockly_blocks_plan.md) explains *how it works*. This document is *the order we build it in*: milestones, tasks, files, tests and what "done" means. Section references like "design §7" point at the design doc.

---

## 1. Scope

**In scope**
- TurboPi only.
- A block editor served by `robot_ops_web` on each robot, generating `myRobot` Python and running it on the robot.
- A **master stop** that stops the motors on the robot from every page and from the teacher's existing **stop all robots** button.
- Block lesson templates, delivered in the existing lesson bundle and opened from the robot's own **Lessons** page.
- Saving student block work on the robot, backed up by the existing workspace sync (Phase 1); hardening that sync (Phase 2).

**Out of scope**
- **Lesson selection in `robot-console`.** No changes to how teachers pick or assign lessons.
- **Other robots** (TonyPi, SpiderPi, MentorPi).
- Replacing Jupyter, changing the student login, a cloud/portal block editor.

---

## 2. Repos, branches and conventions

| Repo | What changes | Tests |
|---|---|---|
| `MataTurboPi` | `common/lib/blocks/` (catalogue), `common/lib/blocks_runtime.py`, block lesson templates, `student_robot_v2` version bump if the API changes | `common/tests/` — plain test functions (the dev machine has no pytest; run with the importlib runner already used for the arm tests). **No notebook-execution tests.** |
| `robot_ops_web` | master stop, `/blocks` page, run/save API, lesson-open routing, env script | `pytest` (`testpaths = tests`), route smoke tests in `tests/test_route_smoke.py` |
| `robot-console` | Phase 2 only: stop-and-hold UI, read-only view of student blocks | existing `tests/` |
| `robot-classroom` | Phase 2 only: sync fixes from the diagnosis doc | existing tests |

- **One branch per milestone** per repo, merged to `main` when the milestone's "done when" passes.
- **Versions:** bump `robot_ops_web/VERSION` (currently `6.14+0`) on each merged milestone; bump `student_robot_v2.__version__` only when the Python API changes; `common/lib/blocks/manifest.json` carries its own `catalogue_version`.
- **Feature flag:** `BLOCKS_ENABLED` in the robot's `.env` (default `0`). When off, the Blocks nav item, page and API are hidden/disabled. **The master stop is never behind the flag** — it ships on for every TurboPi.
- **Hardware:** every milestone that drives motors has a hardware check on a real TurboPi; claw checks use the one arm-equipped robot (`python3 scripts/arm_check.py` first).

---

## 3. Build order at a glance

```
Phase 1 — Proof of concept (staff only, no students)
  M0  Master stop ───────────────┐   (ops_web, first, before anything can drive)
  M1  Spike ─────────────────────┤   (ops_web; measures start-up time)
  M2  Block catalogue + runtime ─┼──▶ M4  Blocks page ──▶ M6  Lessons ──▶ M7  PoC review
  M3  Runner ────────────────────┘        ▲
                                  M5  Saving ───┘
Phase 2 — Class sync   (after M7; S-steps mostly independent)
  S0 status check ▶ S1 … S8
  ── gate: Phase 2 complete ──▶  Student release (R1 pilot class ▶ R2 wider rollout)
Phase 3 — More lessons
```

- **Parallel tracks:** M2 (MataTurboPi) runs alongside M3 (ops_web) once the generated-program shape (design §4.2) is agreed in M1.
- **M5** can start once M3's routes exist; it doesn't need the full page.

Sizes: **S** ≈ a small change with tests, **M** ≈ a few files plus tests and a hardware check, **L** ≈ several files across modules with new UI.

---

## 4. Phase 1 — Proof of concept (staff only)

Everything in Phase 1 runs on proof-of-concept robots with `BLOCKS_ENABLED=1` set by staff. **No student accounts use it.** The point is to prove the editor, runner, master stop, lessons and on-robot saving work on real hardware, and to decide whether to continue.

### M0 — Master stop (ops_web) · size M · *first*

Design §7. Stops the motors on the robot by first killing whatever program could restart them.

| Task | Files | Work |
|---|---|---|
| T0.1 Stop orchestrator | `backend/services/commands.py` | New `master_stop()` that runs, in order, each step independently and reports each: (1) kill block runs via a hook `blocks_runner.kill_all()` (a no-op stub until M3), (2) interrupt every Jupyter kernel via `GET /api/kernels` + `POST /api/kernels/{id}/interrupt` on `127.0.0.1:JUPYTER_PORT`, (3) existing motor `allstop()`, (4) leave the claw as it is, (5) existing `soundoff`. |
| T0.2 Route | `backend/routes/commands.py` | `POST /api/cmd/allstop` calls `master_stop()`; response `{ok, steps:{blocks, kernels, motors, sound}}`. Same URL, so the console's **stop all robots** gets it automatically. Still no login required. |
| T0.3 Header button | `backend/templates/base.html`, `backend/static/stop.js` | Red **STOP** button in the header of every page; shows a brief "Stopped" / "Stop failed: motors" result. |
| T0.4 Tests | `tests/test_master_stop.py`, `tests/test_route_smoke.py` | Steps run even if one fails; kernel interrupt calls the Jupyter API (mocked); motor stop always attempted; every page template contains the button. |

**Done when:** on a real TurboPi, a notebook running a `while True:` drive loop stops **and stays stopped** when you press the header STOP, and again when the teacher presses **stop all robots** in `robot-console`.

### M1 — Spike (ops_web) · size S

| Task | Files | Work |
|---|---|---|
| T1.1 Vendor Blockly | `backend/static/vendor/blockly/` | Pin one Blockly release, bundle the minified core + Python generator + English messages. Note the version in a `VERSION` file there. |
| T1.2 Throwaway page | `backend/templates/blocks.html`, `backend/routes/pages.py` | `/blocks` behind `BLOCKS_ENABLED`; 3 hard-coded blocks (forward, stop, eyes); Run posts the generated Python to a temporary endpoint that runs it as a subprocess. |
| T1.3 Measure | — | Time from Run to first motor movement; decide whether a warm worker is needed later (design risk table). |
| T1.4 Agree the program shape | design §4.2 | Freeze the `start()` / `blk()` / `finish()` contract so M2 and M3 can proceed in parallel. |

**Done when:** offline on the robot hotspot, a student browser drives the robot forward and the Stop button and header STOP both stop it. Start-up time recorded in this doc.

### M2 — Block catalogue and runtime (MataTurboPi) · size M

| Task | Files | Work |
|---|---|---|
| T2.1 Catalogue | `common/lib/blocks/manifest.json`, `blocks.json`, `toolbox.json`, `README.md` | ~15 blocks (design §4.1). Each block's Python output is a **declarative template in `blocks.json`** (e.g. `"python": "myRobot.move.forward(seconds={SECONDS})"`), so it can be tested from Python. |
| T2.2 Generator interpreter | `common/lib/blocks/generators.js` | Small generic JS that turns those templates into Blockly Python generators; hand-written generators only where a template can't express it. Sets `STATEMENT_PREFIX` to emit `blk("<id>")`. |
| T2.3 Runtime | `common/lib/blocks_runtime.py` | `start()`, `blk()`, `finish()`, friendly-error wrapper (design §4.2). Creates the robot the same way `lesson_header.py` does. |
| T2.4 Tests | `common/tests/test_blocks_catalogue.py`, `test_blocks_runtime.py` | Every block has a template; every template renders to code that `compile()`s; golden layouts → exact Python; runtime prints markers in order; `finish()` always stops (fake robot); errors map to friendly messages; arm blocks degrade on a no-claw robot. |
| T2.5 Bundle check | — | Confirm the lesson bundle ships `common/lib/blocks/` to robots (it copies `common/lib`), and TurboPi's `bundle_validation` doesn't reject it. |

**Done when:** all catalogue/runtime tests pass and the catalogue appears in `lessons_cache/common/lib/blocks/` on a test robot after a lesson sync.

### M3 — Runner (ops_web) · size L

Design §5.4.

| Task | Files | Work |
|---|---|---|
| T3.1 Shared env script | `scripts/robot_python_env.sh`, `scripts/start_jupyter.sh` | Move the `.env` / ROS / `PYTHONPATH` / Python-picking lines out of `start_jupyter.sh` into the shared script; `start_jupyter.sh` sources it. **Hardware-check Jupyter still starts and imports the libs** before merging. |
| T3.2 Runner service | `backend/services/blocks_runner.py` | One run per robot, own process group, line-by-line stream parsing (`\x1eBLK`, error markers), 200-line replay buffer, 120 s limit (`BLOCKS_RUN_TIMEOUT_S`), stop sequence SIGINT → 1 s → SIGKILL → `allstop`, page-gone stop after 10 s, `kill_all()` for M0's hook, cleanup of leftover runs on ops_web start-up. |
| T3.3 Pre-checks | same | Master mode gate (423), library health from `/tmp/robot/jupyter_lib_health.json` (503), already running (409), interrupt/shut down this student's Jupyter kernels before starting. |
| T3.4 Routes | `backend/routes/blocks.py` | `POST /api/blocks/run`, `WS /api/blocks/run/{id}/stream`, `POST /api/blocks/stop`, `GET /api/blocks/status` (design §5.2). Register in `backend/main.py`. |
| T3.5 Tests | `tests/test_blocks_runner.py` | Fake program → correct stream messages; every stop path ends in `allstop` (mocked); timeout; second run 409; mode 423; unhealthy lib 503; master stop kills a run. |

**Done when:** tests pass, and on hardware a looping block program is stopped by the page Stop, the header STOP, the console's **stop all robots**, closing the tab, and the time limit — each time the motors stop and stay stopped.

### M4 — Blocks page (ops_web) · size L

| Task | Files | Work |
|---|---|---|
| T4.1 Page | `backend/templates/blocks.html`, `backend/static/blocks.js` | Replace the spike page: palette from `GET /api/blocks/catalogue`, canvas, Run/Stop, output panel, running-block highlight, friendly error on the failed block, **Show Python** panel, **Esc** = master stop. Cache-bust `blocks.js`. |
| T4.2 Catalogue route | `backend/routes/blocks.py` | `GET /api/blocks/catalogue` and `/blocks/lib/generators.js`, read from the lesson cache first, then `/opt/robot/common/lib` (design §5.3). Warn if `manifest.json` needs a newer `student_robot_v2`. |
| T4.3 Nav | `backend/robots/turbopi/__init__.py` | Add **Blocks** to `nav_items`, shown only when `BLOCKS_ENABLED`. |
| T4.4 Tests | `tests/test_route_smoke.py` | Page renders with the flag on; 404/hidden with it off; catalogue served. |

**Done when:** a student can build and run a program from the full catalogue, see which block is running, and read the generated Python.

### M5 — Saving (ops_web) · size M

Design §6.

| Task | Files | Work |
|---|---|---|
| T5.1 File service | `backend/services/blocks.py` | Load/save for the signed-in student only (path validated inside their workspace); atomic write (tmp → fsync → rename); history of last 10 in `.blocks_history/`; revision check (409 on stale); recovery from newest good history on a corrupt file; generated `.py` written beside the blocks file. |
| T5.2 Routes | `backend/routes/blocks.py` | `GET/PUT /api/blocks/file`, `GET /api/blocks/versions`, `POST /api/blocks/restore`, `POST /api/blocks/reset-step`. |
| T5.3 Sync skip | `backend/services/lesson_sync.py` | Add `.blocks_run` to `_should_skip_synced_name`. |
| T5.4 Page wiring | `backend/static/blocks.js` | Debounced autosave (~2 s), save on Run and step change, **Earlier versions** menu, reload prompt on 409. |
| T5.5 Tests | `tests/test_blocks_files.py` | Atomic write; history rotation; 409; corrupt → recovered; path traversal rejected; `.blocks_run` excluded from the sync zip. |

**Done when:** work survives logout → login on the same robot and on a second robot behind the same console (existing sync), and a deliberately corrupted file comes back from history.

### M6 — Lessons (MataTurboPi + ops_web) · size L

Design §4.3–4.4, §5.5.

| Task | Repo / files | Work |
|---|---|---|
| T6.1 Template format | MataTurboPi `common/lib/blocks/lesson_schema.json`, `common/tests/test_blocks_lessons.py` | Schema for `*.blocks.json`; test that every template in `lessons/` validates, references only existing blocks, and its starter workspaces are well-formed. |
| T6.2 Steps UI | ops_web `blocks.js` | Instructions panel (Markdown), Next/Back, per-step toolbox, locked blocks, hints, `show_python` per step, current step saved. |
| T6.3 Template updates | ops_web `blocks.py`, `blocks.js` | Compare `template_version`; offer "reset this step?" — never replace work automatically. |
| T6.4 Lesson-open routing | ops_web `backend/services/session.py`, `backend/routes/lessons.py`, `backend/static/lessons.js` | `open_url` + `kind` alongside the existing `jupyter_url`; `.blocks.json` → `/blocks?file=…`; reference copies → `/blocks?ref=…` (view-only); `/api/lesson/open` only starts Jupyter for notebooks. |
| T6.5 Tutor template mode | ops_web `blocks.js`, `POST /api/blocks/template/export` | Behind the admin bearer token: build steps, set palette and locks, export `.blocks.json`. |
| T6.6 Tests | both repos | Template validation; `.blocks.json` entry → `kind:"blocks"`; `.ipynb` unchanged; reference view can't save. |

**Done when:** a `.blocks.json` lesson in the bundle opens from the robot's **Lessons** page as the student's own copy and as a view-only reference copy. (Check once whether a console-assigned block lesson also materialises correctly; changing console selection is out of scope.)

### M7 — Proof-of-concept review · size M

| Task | Work |
|---|---|
| T7.1 Lesson 21b | Block version of Lesson 21 (claw basics) in `lessons/lesson21b/`, built with tutor mode. |
| T7.2 PoC robots | `BLOCKS_ENABLED=1` on the PoC robots only (including the arm robot), using staff test accounts. |
| T7.3 Hardware checklist | §6 below, on each PoC robot. |
| T7.4 Staff walkthrough | Staff (and teachers, as testers) work through Lesson 21b end to end as if they were students; note usability problems for younger students. |
| T7.5 Go / no-go | Write up findings: start-up time, stop reliability, saving, lesson authoring effort. Decide whether to continue to Phase 2 and student release. |

**Done when:** every stop path is verified on hardware, Lesson 21b works end to end with staff test accounts, and the go/no-go write-up is done. **Students do not use it yet.**

---

## 5. Phase 2 — Class sync

Starts after M7. Source of truth: `docs/student-work-sync-diagnosis.md` (RC1–RC7, R1–R7). Design §9.2.

| Step | Repo(s) | Work | Done when |
|---|---|---|---|
| S0 Status check | all three | Re-verify every RC against current code. The diagnosis marks R1 and R2/R2b as implemented — confirm, and list what remains | Status table added to the diagnosis doc; S-steps below adjusted to match |
| S1 Missing-metadata restore (R3, TurboPi part) | ops_web | A backup without `backup_meta.json` restores `lessons/` with a visible warning instead of skipping | Older zips restore |
| S2 Save-triggered sync (RC7.2) | ops_web | Upload once the workspace has been dirty ~2 min after a block save (debounced), plus on Run / step change; 20-min sync stays as backstop | Power-off at the bell loses ≤ ~2 min |
| S3 Stale robots (RC7.3, RC7.6) | ops_web, console | Robot left logged in stops re-publishing once newer work exists; unsynced work uploads after a reboot | Two robots as one student can't ping-pong `latest.zip` |
| S4 Size tripwire (R1.4) | ops_web | Tripwire accounts for `.blocks_history/` and the skipped `.blocks_run/` | Empty workspace can't overwrite real work |
| S5 Identity (R5) | all three | One shared name-normalisation rule | "Jack S" / "jack s" / "Jack  S" are one student |
| S6 Class mirror safety (R6) | console, classroom | Mirrors can't delete classes or revert PIN/roster changes | Corrupt/missing class file can't wipe classroom data |
| S7 Class stop and hold | ops_web, console | Design §7.4–7.5: `hold` on **stop all robots**, **Release robots**, hold state per robot, library refuses motor commands while held, 10-min expiry | Teacher can pause the class; robots stay stopped until released |
| S8 Console view of block work | console | Versions view lists block files; read-only Blockly view of a student's blocks | Teacher opens a student's blocks from the console |
| S9 Portal (R4), optional | classroom | Only if a portal block editor is wanted later | — |

**Gate:** student release (§5.1) only starts when S0–S8 are done (S9 optional) and the end-to-end tests below pass.

**End-to-end tests (scripted, against test instances):** save on robot A (console 1) → restore on robot B (console 2); console offline during saves then reconnects; same student on two robots; power-off one minute after a save; class stop-and-hold then release.

### 5.1 Student release (after Phase 2)

| Step | Work | Done when |
|---|---|---|
| R1 Pilot class | `BLOCKS_ENABLED=1` on one class's robots; Lesson 21b with real students; teachers briefed on the master stop and stop-and-hold | A class completes Lesson 21b with no lost work; feedback written up |
| R2 Wider rollout | Flag on per class / robot group | — |

---

## 6. Hardware checklist (each PoC robot, repeated for the pilot class robots)

- [ ] Header STOP stops a notebook drive loop; motors stay stopped
- [ ] Console **stop all robots** does the same
- [ ] Block program stopped by: page Stop, Esc, header STOP, console stop, closing the tab, time limit
- [ ] Robot restarts ops_web mid-run → motors stopped on start-up
- [ ] Run refused in remote-control / LLM master mode
- [ ] Run refused while libraries are still syncing after boot, with a friendly message
- [ ] Claw robot: `python3 scripts/arm_check.py` passes; grab/place blocks work; master stop leaves a held object held
- [ ] No-claw robot: claw blocks show "This robot has no claw"
- [ ] Works offline on the robot hotspot (no internet)
- [ ] Work survives logout → login on the same robot and on another robot

---

## 7. Rollout and rollback

1. **M0 (master stop)** ships to every TurboPi as soon as it's done — it's a safety fix and is backward compatible (same `allstop` URL).
2. **M1–M6** ship with `BLOCKS_ENABLED=0`, so robots get the code but students don't see it.
3. **M7 (proof of concept)** turns the flag on for staff PoC robots only.
4. **Phase 2** is completed before any student sees the Blocks page.
5. **Student release** (§5.1): pilot class first, then wider rollout per class/robot group.
6. **Rollback:** set `BLOCKS_ENABLED=0` (page, nav and API off). Block files stay in student workspaces and keep syncing, so nothing is lost. The master stop is never rolled back.

---

## 8. Decisions needed before starting

1. **Lesson naming:** separate block lessons (`lesson21b`) — assumed — or a blocks level inside the existing lesson?
2. **Show Python default:** per template only (assumed), or a class setting later?
3. **Phase 2 ownership:** one workstream across the three repos, or one per repo?
4. **Run time limit:** 120 s default OK for lessons, or longer for Robot League-style tasks?
