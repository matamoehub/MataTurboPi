/**
 * Node-run tests for common/lib/blocks/generators.js's template substitution.
 * Run with: node common/tests/test_blocks_generators.js
 *
 * These do not require Blockly — they test buildPythonFromTemplate and
 * templatePlaceholders directly, and separately verify every block in
 * blocks.json has a template whose placeholders are all covered by its
 * declared args0 field/input names.
 */
const fs = require("fs");
const path = require("path");
const assert = require("assert");

const { buildPythonFromTemplate, templatePlaceholders } = require("../lib/blocks/generators.js");

let passed = 0;
let failed = 0;

function test(name, fn) {
  try {
    fn();
    passed++;
    console.log(`ok   ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL ${name}`);
    console.log(`     ${e.message}`);
  }
}

test("buildPythonFromTemplate substitutes a single placeholder", () => {
  const out = buildPythonFromTemplate("myRobot.move.forward(seconds={SECONDS})", { SECONDS: "1" });
  assert.strictEqual(out, "myRobot.move.forward(seconds=1)");
});

test("buildPythonFromTemplate substitutes multiple placeholders", () => {
  const out = buildPythonFromTemplate(
    "myRobot.move.forward(seconds={SECONDS}, speed={SPEED})",
    { SECONDS: "0.5", SPEED: "80" }
  );
  assert.strictEqual(out, "myRobot.move.forward(seconds=0.5, speed=80)");
});

test("buildPythonFromTemplate substitutes a placeholder used in the method name", () => {
  const out = buildPythonFromTemplate(
    "myRobot.move.turn_{DIRECTION}(seconds={SECONDS}, speed={SPEED})",
    { DIRECTION: "left", SECONDS: "0.3", SPEED: "60" }
  );
  assert.strictEqual(out, "myRobot.move.turn_left(seconds=0.3, speed=60)");
});

test("buildPythonFromTemplate throws on an unknown placeholder", () => {
  assert.throws(() => buildPythonFromTemplate("{NOPE}", {}), /unknown placeholder/);
});

test("templatePlaceholders finds every distinct name, no duplicates", () => {
  const names = templatePlaceholders("myRobot.move.{DIRECTION}(seconds={SECONDS}, speed={SPEED})");
  assert.deepStrictEqual(names.sort(), ["DIRECTION", "SECONDS", "SPEED"]);
});

test("every block in blocks.json with a python template has all placeholders covered by args0", () => {
  const catalogue = JSON.parse(
    fs.readFileSync(path.join(__dirname, "../lib/blocks/blocks.json"), "utf8")
  );
  const problems = [];
  for (const entry of catalogue.blocks) {
    if (entry.python === null || entry.python === undefined) continue;
    const placeholders = templatePlaceholders(entry.python);
    const argNames = (entry.json.args0 || []).map((a) => a.name);
    for (const name of placeholders) {
      if (!argNames.includes(name)) {
        problems.push(`${entry.type}: template references {${name}} but args0 has no arg named ${name}`);
      }
    }
  }
  assert.deepStrictEqual(problems, []);
});

test("every statement block's python template renders to a single trimmed line (no stray newlines)", () => {
  const catalogue = JSON.parse(
    fs.readFileSync(path.join(__dirname, "../lib/blocks/blocks.json"), "utf8")
  );
  const sampleValues = {
    SECONDS: "1", SPEED: "60", DIRECTION: "left", COLOR: "255, 0, 0",
    TEXT: "'hi'",
  };
  for (const entry of catalogue.blocks) {
    if (entry.python === null || entry.python === undefined) continue;
    if (entry.shape === "value") continue;
    const code = buildPythonFromTemplate(entry.python, sampleValues);
    assert.ok(!code.includes("\n"), `${entry.type}: template should be one line, got: ${code}`);
    assert.ok(code.trim().length > 0, `${entry.type}: template rendered empty`);
  }
});

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed === 0 ? 0 : 1);
