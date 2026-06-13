# Student Work Sync — Diagnosis and Recommendations

**Date:** 2026-06-13
**Scope:** `robot-console` (Flask, teacher/fleet dashboard), `robot_ops_web` (FastAPI, runs on each robot), `robot-classroom` (FastAPI + Postgres, always-on cloud master).
**Symptom:** students report saved work being **overwritten** and **missing**. This document is written for an implementation agent; every finding cites file and line so it can be verified before changing anything.

---

## 1. How the system works today (as-built, not as-documented)

### 1.1 The three stores of student work

| Store | Location | Key | Versioned? |
|---|---|---|---|
| Robot workspace (live) | `WORKSPACES_DIR/<class>/<student>/` on each robot (`robot_ops_web/backend/services/workspace.py:307`) | safe-segmented class+student name | No |
| Console store | `student_work/<class>/<student>/[<robot_type>/]latest.zip` + timestamped versions (`robot-console/services/student_work.py:55-186`) | safe-segmented names, robot-type subdir **on writes from robots only** | Yes |
| Classroom store | `WORKSPACES_DIR/_sync/<class>/<student>/latest.zip` + versions, plus a `StudentSnapshot` DB row (`robot-classroom/backend/services/integration_service.py:278-415`) | safe-segmented names, **no robot-type namespacing** | Yes (files), DB row only points at `latest.zip` |

There is also a **fourth, disconnected copy**: the cloud portal workspace at `WORKSPACES_DIR/<class>/<student>/<lesson>/` on the classroom server (`workspace_service.py:20-26`), used by hosted Jupyter / the v2 editor / the simulator.

### 1.2 Data flows

- **Robot → upstream save:** `sync_student_work` zips the workspace and POSTs to the **first reachable source** in order (master=console, then classroom) — never both (`robot_ops_web/backend/services/console.py:483-511`). Triggers: every 20 min (`WORK_SYNC_INTERVAL_S=1200`, `session.py:255-275`), on logout (`routes/student.py:166`), on manual sync, and after import/restore-version.
- **Robot → restore:** on login, `download_student_work` tries sources in order and extracts with `overwrite_existing=False` (`routes/student.py:103-137`, `workspace.py:830`).
- **Console relay:** every robot upload to console is immediately pushed to classroom (`robot-console/app.py:3474` → `_sync_student_work_to_robot_classroom`, app.py:327). Classroom-side uploads marked `source != "console_sync"` are pushed back to console (`robot-classroom/backend/routes/integrations.py:267-269`).
- **Console pull loop:** if `ROBOT_CLASSROOM_AUTO_PULL` is on, every 10 min (default `ROBOT_CLASSROOM_PULL_INTERVAL=600`, app.py:812) the console pulls the class list **and every student's snapshot** from classroom; if the SHA differs from the console's copy, the classroom copy **unconditionally replaces** the console copy (`app.py:385-415`, called at `app.py:716-742`).
- **Class mirror, classroom side:** classroom polls the console export every 60 s and applies it with `replace_existing=True`, **deleting classrooms not present in the export** (`robot-classroom/backend/services/console_sync_service.py:79-84,184`).
- **Class mirror, console side:** console pull merges classroom data per class code, **remote wins wholesale per class** (`robot-console/app.py:649-667`).
- **Portal:** on every portal login, the latest robot snapshot zip is extracted **over** the cloud workspace (`robot-classroom/backend/routes/home.py:117-126` → `restore_snapshot_zip_to_directory`, `integration_service.py:477-516`). Nothing ever zips the cloud workspace back into the snapshot store — `store_student_snapshot` is only called from the robot-facing endpoint (`integrations.py:266`).

---

## 2. Diagnosis — root causes, ranked by likely contribution to "work overwritten / missing"

### RC1 — Login race: lesson materialisation vs. work restore (HIGH severity, robot-side)

On login, `_apply_assignment_from_master` and `_master_download_student_work` run **in parallel** (`routes/student.py:119-123`). Applying the assignment materialises pristine lesson files into the workspace (`session.py:447-506` → `_copy_lesson_to_workspace`). The restore then extracts the student's saved zip with `overwrite_existing=False` (`student.py:131`), which **skips every file that already exists** (`workspace.py:854-856`).

The student's notebooks live inside the same `lessons/lessonXX/...` paths the materialiser writes. The bundle is usually already cached, so materialisation (local copy) almost always beats the download (network). Result, on any robot where this student has no prior local workspace (i.e. **whenever a student switches robots**):

1. Pristine lesson files land first.
2. Restore skips them — the student's saved notebooks are silently discarded ("my work is missing").
3. ≤20 min later, `_work_sync_loop` uploads the pristine workspace as the new `latest.zip` — the server's good copy is now buried ("my work was overwritten"). It survives only in the versioned backups, which students don't know about.

This single interaction explains both halves of the complaint and fires precisely in the everyday classroom scenario of students not getting the same robot each session.

### RC2 — Blind last-writer-wins between console and classroom (HIGH severity, server-side)

`_sync_student_work_from_robot_classroom` (`robot-console/app.py:385-415`) compares only SHA-256 — **no timestamp, no version ordering**. If the shas differ, the classroom copy replaces the console copy. So whenever console has a *newer* save than classroom (e.g. the per-upload relay to classroom failed once — network blip, classroom restart, 50 MB limit difference), the next pull cycle replaces the console's newest save with the older classroom blob. The robot then restores the stale copy, and the cycle re-publishes it.

The reverse direction (`_sync_all_console_data_to_robot_classroom`, app.py:418) and the per-upload relay are equally unconditioned. Two interval loops copying the same blob in opposite directions with "different ⇒ overwrite" semantics is a lost-update generator.

### RC3 — Robot-type namespacing exists only on the console (HIGH severity for mixed-robot classes)

- Robots upload with `robot_type`; console stores under `student_work/<class>/<student>/<robot_type>/` (`student_work.py:55-78`).
- The console→classroom relay drops `robot_type` (`app.py:330-335`), and classroom stores one un-namespaced `latest.zip` per student (`integration_service.py:278-285`). A student who uses a TurboPi and a TonyPi has the two robots' saves **clobbering each other in the classroom store**.
- The console pull loop writes the classroom blob back to the **legacy un-typed** path (`app.py:398` uses `_student_zip_path(class, student)` with no robot_type; the subsequent `_write_student_work_version` call at app.py:409 passes no `requested_robot_type`). The legacy path is the read-fallback for any student without a typed save (`student_work.py:92-103`) — so cross-type contamination flows back into reads.
- Mitigation half-works: the robot refuses to restore the `lessons/` tree when backup metadata names a different robot type (`workspace.py:771-774,851-853`) — preventing breakage but surfacing as "no previous work". Note: a zip with **missing** metadata (older saves, any externally produced zip) also fails this check and silently restores nothing of `lessons/`.

### RC4 — Cloud portal work is a dead end and is itself overwritten (HIGH severity if the portal is in use)

- Portal/Jupyter/simulator edits are saved only to the cloud workspace directory. Nothing exports them to the snapshot store or to the console — robots can never see web work ("missing").
- Worse, every portal login force-extracts the latest **robot** snapshot over the cloud workspace, protecting only two hard-coded filenames (`home.py:121-126`, `STUDENT_OWNED_SUPPORT_FILES = {"student_robot_moves.py", "student_animation_lib.py"}`, `workspace_service.py:161`). Any notebook a student edited in the browser is overwritten by the (possibly much older) robot save on the next portal visit.

### RC5 — Identity is free-typed text (MEDIUM severity, chronic)

Students type their own name at every login; class+student strings are the only key everywhere. Normalisation differs by system:

- Console roster dedups case-insensitively (`services/classes.py:105`) but **storage paths keep the typed case** (`_safe_path_seg`, `student_work.py:43`): "Jack" and "jack" share a roster entry but have different work directories.
- Classroom matches `display_name` **case-sensitively** for both upsert (`classroom_service.py:50-56`) and download (`integrations.py:306`) — "jack" gets a 404 for "Jack"'s work and a duplicate Student row.
- Robot workspace paths use yet another sanitiser (`startup._safe_segment`).

"Jack S" / "jack s" / "Jack  S" are three different students in at least one of the three systems. This is a steady source of "my work disappeared" reports that no sync fix will cure.

### RC6 — Class-mirror loops can destroy classroom state (MEDIUM severity, episodic)

- Classroom applies the console export with `replace_existing=True` every 60 s (`console_sync_service.py:79-84,184`): any class missing from the export is **deleted**, cascading students and `StudentSnapshot` rows (snapshot files remain on disk but become unreachable via the API → robots get 404 → "no previous work" → fresh uploads). Console's `load_classes()` returns `[]` on any JSON parse error (`services/classes.py:65-66`) — a corrupt `classes.json` or an empty new console instance pointed at production classroom would wipe every classroom.
- Console's pull merge takes the remote class record wholesale per code (`app.py:659`), so a PIN/roster change made on the console can be reverted by the next pull if the classroom hadn't mirrored it yet (60 s + 600 s windows racing each other, no timestamps anywhere).

### RC7 — Assorted contributing defects (LOW each, additive)

1. **404 short-circuit:** robot `download_student_work` returns `not_found` as soon as the *first* source 404s, never trying the second (`console.py:446-447`); work that exists only on classroom is invisible whenever console answers 404 and the console-side classroom fallback isn't configured. (The versions endpoint handles this correctly — `console.py:467-470` continues.)
2. **Up to 20 minutes of loss on power-off:** `WORK_SYNC_INTERVAL_S=1200` with no save-triggered or shutdown-triggered sync; robots in classrooms get switched off at the bell.
3. **Stale logged-in robot keeps publishing:** sessions persist until logout/reboot (`session.py:73-79`); a robot left logged in re-uploads its stale workspace every 20 min, ping-ponging `latest.zip` against the robot the student actually moved to. No device fencing/heartbeat on the upload path.
4. **`overwrite_student_files` assignment flag** silently replaces student edits on every materialise when set (`workspace.py:611`, `.bak` is kept but invisible to students).
5. **Local PIN cache accepts revoked PINs** indefinitely when offline-first (`console.py:286-291` checks local cache before upstream).
6. **Boot-id session invalidation** (`session.py:77-78`): after a mid-class reboot the sync loop stops (no session) — local work sits unsynced until the student logs in again on that same robot; if they move robots instead, the unsynced work is stranded.
7. **No upload fan-out:** a save that lands only on classroom (console down) is later invisible to a robot whose console is back up and holds an older copy — interacts with RC2/RC7.1.

### What is *not* broken

Worth preserving: atomic writes on the console (`_atomic_write_json/_atomic_write_bytes`), versioned saves on both servers, the zip backup metadata (`.matamoe/backup_meta.json`, `workspace.py:721-750`), corrupt-lib validation/sanitising, core-dump purging, the `source != "console_sync"` echo guard (`integrations.py:267-269`), and the robot's offline login caches.

---

## 3. Recommendations

Ordered so each step delivers value alone; 1–4 are the high-payoff fixes.

### R1 — Fix the login restore order (kills RC1; small, robot-only change)

In `api_student_login` (`robot_ops_web/backend/routes/student.py`):

1. Download student work **first** (or run in parallel but **block materialisation until the restore has been applied**).
2. Restore into the workspace, **then** apply the assignment/materialise. `_copy_lesson_to_workspace` already merges with `overwrite=False`, so running it after the restore is safe for student files.
3. If the download fails for any reason other than `not_found` on a **fresh workspace**, do *not* let the periodic sync upload: set a "restore incomplete" flag on the session and have `_work_sync_loop` (`session.py:255`) skip uploads while it is set, retrying the download instead. This is the single most important guard: never publish a workspace that may be missing the student's prior work.
4. Additionally, make `_workspace_zip_bytes`/`sync_student_work` refuse to upload when the new zip is dramatically smaller than the last-known server copy (e.g. <50% of previous size and previous > some floor) without an explicit override — cheap tripwire against any residual "publish pristine workspace" path.

### R2 — One authority per student blob; stop blind bidirectional copying (kills RC2)

**Confirmed topology (fleet has ~3 consoles):** work sync flows **robot_ops_web → its console → robot-classroom → back to every console**. `robot-classroom` is the **single master of all student work** across the whole fleet; each console is a per-site relay/cache. The robot uploads to *its* console (not straight to classroom); the console relays up to the master; the master redistributes down to all consoles so a student can resume on a robot behind any console.

- Add a monotonic `saved_at` (already present in zip metadata: `backup_meta.created_at`) plus the uploading `robot_id` to the sync payload, and store both server-side.
- Replace "sha differs ⇒ overwrite" in `_sync_student_work_from_robot_classroom` (`app.py:385`) and in the relay/push directions with **"incoming `created_at` newer than stored `created_at` ⇒ accept, else reject (keep as version only)"**. Both servers already write versioned files, so a rejected blob can still be archived as a version without touching `latest`.
- **Master arbitration lives at robot-classroom.** `store_student_snapshot` always archives a version and only advances `latest` when the incoming `created_at` is not older — this is what arbitrates between saves arriving from different consoles/robots.
- **The robot uploads console-first**, falling back to classroom only when its console is unreachable (offline resilience), stopping at the first success — it does **not** push straight to classroom in the normal case. Downloads/restores still try console first then classroom, so a student moving to a robot behind a different console still gets the master copy even before that console's pull-back has run.
- **Console-side anti-regression gate is retained:** a console only refuses the master's copy while its own *un-relayed* local copy is newer; once it relays up, the master subsumes it and all consoles converge.

The minimum viable fix (implemented) is the timestamp gate on all four copy paths (robot→console, console→classroom, classroom→console, plus the classroom-side gate covering portal saves) with console-primary uploads.

#### R2b — Delta (rsync-style) reconciliation for offline consoles (implemented)

Some consoles run **with no internet for long stretches**. The realtime relay (console→classroom on each robot upload) is fire-and-forget: if the console is offline when a robot saves, that work was **never retried** and stayed stranded. And the old pull-back re-downloaded every student's full blob every cycle. Both are fixed with a manifest diff:

- **Master manifest endpoint:** `GET /api/console/work/manifest` on robot-classroom returns one entry per `(class, student, robot_type)` latest with its `created_at` + `sha256` + size, read from sidecar meta (no zip opening). `store_student_snapshot` now records `sha256`/`created_at`/`robot_type` in that meta.
- **Two-way reconciliation on the console** (`_reconcile_work_with_robot_classroom`, runs on the pull loop **and** on startup/reconnect): diffs the master manifest against the console's local manifest and transfers **only deltas**:
  - local newer than master, or master missing → **push up** (this is the offline catch-up — work saved while disconnected goes up next time online),
  - master newer/equal, or local missing → **pull down** (master wins ties, since classroom is authority),
  - identical `sha256` → **skip** (no transfer).
- The decision logic is a pure planner (`student_work._reconcile_plan`) so it is unit-tested without the network; every transfer is still timestamp-gated on the receiving side, so it is safe to run repeatedly and alongside the realtime relay.
- This replaced the old "download every student's full blob every cycle" loop inside the console pull cycle.

> **Defaults flipped ON (implemented):** `ROBOT_CLASSROOM_AUTO_PULL` and `ROBOT_CLASSROOM_STARTUP_PULL` now default `true` on the console, and `AUTO_CONSOLE_SYNC_ENABLED` defaults `true` on robot-classroom, so the master↔console legs run out of the box. Each console still needs `ROBOT_CLASSROOM_URL` + token configured (a console with no classroom URL simply no-ops these loops). Startup pull is what drives the offline-then-online catch-up.

### R3 — Carry `robot_type` end-to-end (kills RC3)

- Console relay (`app.py:330-335`): include `requested_robot_type`.
- Classroom store (`integration_service.py:278-294`, `store_student_snapshot`, download/versions endpoints): namespace exactly like the console (`_sync/<class>/<student>/<robot_type>/latest.zip` with legacy fallback for reads), and add `robot_type` to `StudentSnapshot` (one row per student+type).
- Console pull (`app.py:385-415`): pass the robot type through to `_student_zip_path` and `_write_student_work_version`.
- Robot restore: when backup metadata is **missing** (not mismatched), treat it as matching rather than skipping the whole `lessons/` tree (`workspace.py:771-774`) — or at least surface a visible warning in the login response.

### R4 — Give the portal a save path and stop clobbering it (kills RC4)

- On portal session end / autosave debounce, zip the cloud student root with the same member filter and `.matamoe/backup_meta.json` writer the robot uses, and feed it through `store_student_snapshot` (then it flows to the console via the existing push).
- Make `restore_snapshot_zip_to_directory` in `build_portal_payload` (`home.py:121-126`) timestamp-aware: only extract snapshot members **newer than** the cloud workspace's last save (or simply skip files modified in the cloud workspace since the snapshot's `created_at`). The two-filename `STUDENT_OWNED_SUPPORT_FILES` allowlist is not a real protection mechanism.
- Until that's built, the honest stopgap is to **stop auto-restoring over the cloud workspace on every login** and only restore when the cloud workspace doesn't exist yet.

### R5 — Normalise identity once, identically (kills RC5)

- Define one canonical key: `lower(trim(collapse_whitespace(name)))`, applied in **all three** repos for lookups and storage paths (keep `display_name` separately for UI).
- Classroom: make `upsert_student_by_name` and the download/versions student match case-insensitive (`classroom_service.py:50`, `integrations.py:306,348`); add a one-off migration to merge case-duplicate Student rows and snapshot dirs.
- Console/robot: route every path construction through one shared normaliser (today there are three different sanitisers).
- Longer term: pick the student from the **roster** (dropdown sourced from `/api/classes/public`, which already returns students) instead of free typing; allow "add new name" as the exception path.

### R6 — Make the class mirrors safe (kills RC6)

- Remove `replace_existing=True` from the auto-pull path (`console_sync_service.py:184`); never auto-delete classrooms. Reserve destructive replace for the explicit migration script (which already has `--replace-classes`).
- Refuse to apply an export with zero classes when the DB has classes (corrupt-`classes.json` guard); also make console `load_classes()` fail loudly instead of returning `[]` on parse errors (`services/classes.py:65-66`).
- Add `updated_at` per class record on both sides and apply newest-wins per field group (PIN/name vs roster vs lesson selection), or — simpler — declare one side authoritative per field: console authoritative for roster/PIN (teacher tooling lives there today), classroom authoritative for lesson browsability.

### R7 — Smaller hardening items

- `download_student_work`: on 404 from a source, **continue** to the next source (mirror the versions-endpoint behaviour, `console.py:446` vs `467`).
- Shorten the loss window: sync on Jupyter save events or drop `WORK_SYNC_INTERVAL_S` to ~5 min with a content-hash check so unchanged workspaces don't re-upload (the robot already computes the zip; hash it and skip if unchanged since last push — also removes most version churn server-side).
- Add a systemd `ExecStop`/shutdown hook on the robot that fires one final `sync_student_work`.
- Stale-device fencing: include `robot_id` + session `updated_at` in uploads; server rejects uploads from a robot whose session for that student is older than another robot's more recent login (simple per-student "active device" record on the classroom server).
- Expose "Restore an earlier version" prominently in the student/teacher UI — the versioned saves on both servers mean most "lost" work from past incidents is still recoverable today; check `student_work/<class>/<student>/**/*.zip` and `_sync/<class>/<student>/*.zip` before assuming anything is gone.
- Add retention/pruning for version files (currently unbounded: one zip per student per 20-min cycle per robot in the worst case).

### Suggested implementation order

| Phase | Items | Why first |
|---|---|---|
| 1 | R1 (login order + restore-incomplete guard), R7 404-continue | Stops the most common active data-loss path; robot-only deploy |
| 2 | R2 timestamp gates on all four server copy paths | Stops server-side stomping; no schema change needed (timestamp lives in zip meta already) |
| 3 | R3 robot_type end-to-end, R6 mirror safety | Correctness for mixed fleets; removes the classroom-wipe hazard |
| 4 | R5 identity normalisation (+ migration), R4 portal save path | Bigger, cross-repo; needs a data migration and portal UX work |

### Testing notes for the implementer

- All three repos have `pytest.ini`; run each suite before/after.
- The killer regression test for R1: simulate login on a robot with an empty workspace, a cached lesson bundle, and a slow (mocked) `download_student_work`; assert the restored notebook content — not the pristine bundle content — is present afterwards, and that `_work_sync_loop` does not upload while the restore is unresolved.
- For R2: pair of mocked servers where console holds `created_at=T2` and classroom `T1<T2`; assert the pull loop does **not** replace the console `latest`.
- For R6: feed `sync_classes_from_console` an empty item list against a populated DB; assert no deletions on the auto path.
