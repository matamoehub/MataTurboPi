/**
 * Generic Blockly Python generator for the TurboPi block catalogue.
 *
 * Every block in blocks.json carries a declarative "python" template, e.g.
 *   "myRobot.move.forward(seconds={SECONDS}, speed={SPEED})"
 * This module turns those templates into real Blockly.Python generator
 * functions by substituting each {NAME} placeholder with the Blockly code
 * for the block's "NAME" field or input value.
 *
 * Usage (in the browser, after Blockly + the Python generator are loaded):
 *   const catalogue = await fetch('/blocks/lib/blocks.json').then(r => r.json());
 *   registerBlockGenerators(Blockly, catalogue);
 *
 * This file has no Blockly-specific syntax of its own beyond calling into
 * the Blockly object passed to it, so the template-substitution logic
 * (buildPythonFromTemplate) can be unit-tested with plain Node — see
 * common/tests/test_blocks_generators.js.
 */

/**
 * Replace every {NAME} placeholder in `template` with `values[NAME]`.
 * Throws if the template references a name that isn't in `values`, so a
 * typo in a block definition fails loudly instead of emitting broken code.
 *
 * @param {string} template
 * @param {Object<string,string>} values
 * @returns {string}
 */
function buildPythonFromTemplate(template, values) {
  return template.replace(/\{([A-Z_][A-Z0-9_]*)\}/g, (match, name) => {
    if (!(name in values)) {
      throw new Error(`generators.js: template references unknown placeholder {${name}}`);
    }
    return values[name];
  });
}

/**
 * Determine the field/value names referenced in a template, so callers can
 * validate a catalogue entry's json.args0 covers every placeholder it uses.
 * @param {string} template
 * @returns {string[]}
 */
function templatePlaceholders(template) {
  const names = new Set();
  const re = /\{([A-Z_][A-Z0-9_]*)\}/g;
  let m;
  while ((m = re.exec(template)) !== null) {
    names.add(m[1]);
  }
  return Array.from(names);
}

/**
 * Register one Blockly.Python generator per catalogue block that has a
 * "python" template. Blocks with python === null (e.g. when_run) are event
 * roots handled by the page itself, not generated as a statement.
 *
 * @param {Object} Blockly - the global Blockly object (with Blockly.Python already loaded)
 * @param {Object} catalogue - parsed blocks.json
 */
function registerBlockGenerators(Blockly, catalogue) {
  const pyGen = Blockly.Python;
  for (const entry of catalogue.blocks) {
    if (entry.python === null || entry.python === undefined) {
      continue; // event-root or otherwise hand-wired block
    }
    const template = entry.python;
    const isValue = entry.shape === "value";

    pyGen.forBlock = pyGen.forBlock || {};
    pyGen.forBlock[entry.type] = function (block) {
      const values = {};
      for (const arg of (entry.json.args0 || [])) {
        if (arg.type === "field_number" || arg.type === "field_dropdown") {
          values[arg.name] = String(block.getFieldValue(arg.name));
        } else if (arg.type === "input_value") {
          values[arg.name] =
            pyGen.valueToCode(block, arg.name, pyGen.ORDER_NONE) || "''";
        } else if (arg.type === "field_input") {
          // Raw, unescaped substitution -- only safe for a template that
          // places this inside a Python comment (the only current use:
          // note_text's "# {TEXT}"). A field_input landing in actual
          // code would need proper string-literal escaping first.
          values[arg.name] = String(block.getFieldValue(arg.name)).replace(/[\r\n]+/g, " ");
        }
      }
      const code = buildPythonFromTemplate(template, values);
      if (isValue) {
        return [code, pyGen.ORDER_FUNCTION_CALL];
      }
      // STATEMENT_PREFIX (configured by the page) emits a blk(id) marker
      // before this line, using block.id, so the running block can be
      // highlighted while the program executes on the robot.
      return code + "\n";
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { buildPythonFromTemplate, templatePlaceholders, registerBlockGenerators };
}
