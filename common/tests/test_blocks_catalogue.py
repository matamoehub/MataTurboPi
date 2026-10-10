"""Tests for the block catalogue files (common/lib/blocks/*.json) --
validates structure and cross-references without needing Blockly or Node.
The JS-side template-substitution behaviour is tested separately in
common/tests/test_blocks_generators.js (run with `node`).
"""
import json
import re
from pathlib import Path

_BLOCKS_DIR = Path(__file__).resolve().parents[1] / "lib" / "blocks"

_PLACEHOLDER_RE = re.compile(r"\{([A-Z_][A-Z0-9_]*)\}")


def _load(name):
    return json.loads((_BLOCKS_DIR / name).read_text())


def test_manifest_references_files_that_exist():
    manifest = _load("manifest.json")
    for key in ("blocks_file", "toolbox_file", "lesson_schema_file"):
        assert (_BLOCKS_DIR / manifest[key]).exists(), f"manifest.{key} -> missing file"
    assert isinstance(manifest["catalogue_version"], int)
    assert manifest["catalogue_version"] >= 1


def test_every_block_has_a_unique_type_and_category():
    catalogue = _load("blocks.json")
    types_seen = set()
    for entry in catalogue["blocks"]:
        assert "type" in entry
        assert entry["type"] not in types_seen, f"duplicate block type: {entry['type']}"
        types_seen.add(entry["type"])
        assert "category" in entry
        assert "json" in entry
        assert entry["json"]["type"] == entry["type"]


def test_statement_blocks_have_previous_and_next_statement_or_are_the_event_root():
    catalogue = _load("blocks.json")
    for entry in catalogue["blocks"]:
        if entry.get("shape") == "value":
            assert "output" in entry["json"], f"{entry['type']}: value block missing output"
            continue
        if entry.get("is_event_root"):
            assert entry["json"].get("nextStatement") is None or "nextStatement" in entry["json"]
            continue
        assert "previousStatement" in entry["json"], f"{entry['type']}: missing previousStatement"
        assert "nextStatement" in entry["json"], f"{entry['type']}: missing nextStatement"


def test_python_template_placeholders_match_declared_args():
    """Mirrors the check in test_blocks_generators.js (Python side), so a
    broken template is caught even if nobody runs the Node suite."""
    catalogue = _load("blocks.json")
    problems = []
    for entry in catalogue["blocks"]:
        template = entry.get("python")
        if template is None:
            continue
        placeholders = set(_PLACEHOLDER_RE.findall(template))
        arg_names = {a["name"] for a in entry["json"].get("args0", [])}
        missing = placeholders - arg_names
        if missing:
            problems.append(f"{entry['type']}: template uses {missing} not in args0")
    assert problems == [], "\n".join(problems)


def test_every_block_has_a_tooltip():
    catalogue = _load("blocks.json")
    for entry in catalogue["blocks"]:
        tooltip = entry["json"].get("tooltip", "")
        assert tooltip.strip(), f"{entry['type']}: missing/empty tooltip"


def test_arm_blocks_use_the_arm_namespace_so_they_degrade_safely_on_no_claw_robots():
    """arm_lib / student_robot_v2's ArmNamespace already no-ops (returns
    False) when there's no claw -- these block templates must route through
    myRobot.arm.* so that safety behaviour is inherited, not re-implemented."""
    catalogue = _load("blocks.json")
    arm_blocks = [e for e in catalogue["blocks"] if e["category"] == "Claw"]
    assert len(arm_blocks) >= 4
    for entry in arm_blocks:
        assert entry["python"].startswith("myRobot.arm."), entry["type"]


def test_toolbox_categories_only_reference_blocks_in_the_catalogue_or_blockly_core():
    catalogue = _load("blocks.json")
    toolbox = _load("toolbox.json")
    catalogue_types = {e["type"] for e in catalogue["blocks"]}
    for category in toolbox["default_categories"]:
        if category.get("source") == "blockly-core":
            continue  # built into Blockly, not ours to validate
        for block_type in category["block_types"]:
            assert block_type in catalogue_types, (
                f"toolbox category {category['name']!r} references "
                f"unknown block type {block_type!r}"
            )


def test_every_catalogue_block_is_reachable_from_the_default_toolbox():
    """A block nobody can drag onto the canvas is dead weight -- catch one
    before it ships forgotten."""
    catalogue = _load("blocks.json")
    toolbox = _load("toolbox.json")
    referenced = set()
    for category in toolbox["default_categories"]:
        referenced.update(category["block_types"])
    for entry in catalogue["blocks"]:
        assert entry["type"] in referenced, f"{entry['type']} not in any toolbox category"


def test_lesson_schema_names_match_actual_conventions():
    schema = _load("lesson_schema.json")
    assert schema["format_value"] == "turbopi-blocks-lesson"
    assert "template_id" in schema["required_top_level_keys"]
    assert "steps" in schema["required_top_level_keys"]
