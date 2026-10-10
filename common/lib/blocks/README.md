# common/lib/blocks — the TurboPi block catalogue

Part of the Blockly proof of concept. See `docs/blockly_blocks_plan.md` (design)
and `docs/blockly_build_plan.md` (build order, milestone M2) at the repo root.

## Files

| File | What it is |
|---|---|
| `manifest.json` | Catalogue version + which `student_robot_v2` version it needs. `robot_ops_web` checks this before letting a program run. |
| `blocks.json` | Every block: its Blockly JSON shape (`json`) and its Python output template (`python`, e.g. `"myRobot.move.forward(seconds={SECONDS}, speed={SPEED})"`). |
| `toolbox.json` | The default full palette, grouped into categories. A lesson step can offer a smaller list (see `lesson_schema.json`). |
| `generators.js` | Turns each block's template into a real Blockly Python generator by substituting `{NAME}` with the block's field/input code. Runs in the student's browser. |
| `lesson_schema.json` | What a `*.blocks.json` lesson template must contain. |

## Adding a new block

1. Add an entry to `blocks.json`: a unique `type`, a `category` (must exist in
   `toolbox.json` or be added there), the Blockly `json` shape, and a `python`
   template.
2. Every `{PLACEHOLDER}` in the template must match the `name` of a `field_number`,
   `field_dropdown` or `input_value` entry in `json.args0`.
3. Add the block's `type` to a category in `toolbox.json` (a block that's in
   the catalogue but not in any toolbox category is dead — nobody can drag it
   onto the canvas).
4. Run the tests:
   - `common/tests/test_blocks_catalogue.py` (Python, no deps) — checks the
     JSON is well-formed and cross-referenced correctly.
   - `common/tests/test_blocks_generators.js` (`node common/tests/test_blocks_generators.js`) —
     checks the real template-substitution logic against every block.
5. Bump `catalogue_version` in `manifest.json` if you removed or renamed a
   block (adding one is backward compatible, so a version bump isn't required).

## Why templates are plain strings, not JavaScript functions

Every block's Python output is a declarative string (`blocks.json`), not a
hand-written JS function, specifically so the MataTurboPi test suite (plain
Python, no Node/Blockly needed) can check every block's generated code
without a browser. `generators.js` is the thin, generic interpreter that
turns those strings into real Blockly generators at runtime.

## Safety

Claw blocks (`arm_test`, `arm_ready`, `arm_grab`, `arm_place`) all call through
`myRobot.arm.*`, which already degrades to a safe no-op on a robot with no
claw attachment (see `arm_lib.py`). Don't reimplement that check in a block
template — route through the namespace and it comes for free.
