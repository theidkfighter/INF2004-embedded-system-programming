'use strict';
// Properties 24, 29 and 30 (spec tasks 14.10, 14.15, 14.16) for the JavaScript
// Laptop_Codec; py/test_codec_params.py does the Python and C halves.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fc = require('fast-check');
const { checkProperty } = require('./prop');
const { codec, wireDump } = require('./cross_helper');

const quiet = (logs) => ({ log: (line) => logs.push(line) });

// ---------------- Property 24 ----------------

/** CRC-16/CCITT-FALSE of the NAMEs + "\n", written out here and not taken from the codec. */
function referenceVersion(names) {
  let crc = 0xFFFF;
  for (const byte of Buffer.from(names.map((n) => n + '\n').join(''), 'latin1')) {
    crc ^= byte << 8;
    for (let i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
  }
  return [names.length, crc];
}

const NAME = fc.stringMatching(/^[A-Z0-9_]{1,24}$/);

// Feature: minimum-bit-data-representation, Property 24: The parameter table version detects table edits
test('P24 parameter table version', () => {
  const dump = wireDump();
  assert.deepStrictEqual([codec.PARAM_COUNT, codec.PARAM_CRC], [dump.params.length, dump.param_table_crc], 'the C table');
  checkProperty({ n: 24, title: 'The parameter table version detects table edits', unit: 'parameter table' },
    fc.property(fc.uniqueArray(NAME, { minLength: 1, maxLength: 60 }), fc.nat(1 << 30), fc.nat(1 << 30), (names, a, b) => {
      const version = codec.paramTableVersion(names);
      assert.deepStrictEqual(version, referenceVersion(names));
      const added = names.slice();
      added.splice(a % (names.length + 1), 0, 'NEW_ENTRY');
      assert.notDeepStrictEqual(codec.paramTableVersion(added), version, 'one name added');
      if (names.length > 1) {
        const removed = names.slice();
        removed.splice(a % names.length, 1);
        assert.notDeepStrictEqual(codec.paramTableVersion(removed), version, 'one name removed');
        const i = a % names.length;
        const j = b % names.length;
        if (names[i] !== names[j]) {
          const swapped = names.slice();
          [swapped[i], swapped[j]] = [swapped[j], swapped[i]];
          assert.notDeepStrictEqual(codec.paramTableVersion(swapped), version, 'two names swapped');
        }
      }
    }), { numRuns: 1000 });
});

// ---------------- Property 29 ----------------

const PARAMS = codec.PARAMS;

/** A value text inside the parameter's range (whole numbers for the packed ones). */
function valueText(index, fraction) {
  const [, kind, low, high] = PARAMS[index];
  if (kind !== 'F32') {
    return String(low + Math.floor(fraction * (high - low + 1)) % (high - low + 1));
  }
  const x = fraction < 0.2 ? low : fraction > 0.8 ? high : low + (high - low) * fraction;
  return String(Math.min(Math.max(Math.round(x * 10000) / 10000, low), high));
}

const validItem = fc.record({
  index: fc.nat(codec.PARAM_COUNT - 1), fraction: fc.double({ min: 0, max: 1, noNaN: true }),
  form: fc.nat(2), filler: fc.nat(3),
});

function linesOf(items) {
  const lines = [];
  const pairs = [];
  for (const it of items) {
    lines.push(...[[], [''], ['# a comment'], ['', '  # note: A=1']][it.filler]);
    const text = valueText(it.index, it.fraction);
    const name = PARAMS[it.index][0];
    lines.push([`${name}=${text}`, `${name} = ${text}`, `${name}=${text}  # kept`][it.form]);
    pairs.push([it.index, codec.f32ToBits(Number(text))]);
  }
  return [lines, pairs];
}

function badLine(index, kind) {
  const [name, , low, high] = PARAMS[index];
  return [['NOT_A_PARAM', 'NOT_A_PARAM=1'], [name, `${name}=abc`], [name, `${name}=${high + 1}`],
    [name, `${name}=${low - 1}`]][kind];
}

// Feature: minimum-bit-data-representation, Property 29: Tool inputs parse to Parameter_IDs or report the bad line
test('P29 valid robot_params.txt content', () => {
  checkProperty({ n: 29, title: 'Tool inputs parse to Parameter_IDs or report the bad line', unit: 'robot_params.txt' },
    fc.property(fc.array(validItem, { maxLength: 20 }), (items) => {
      const [lines, pairs] = linesOf(items);
      const frame = codec.decodeFields('config', codec.encodeConfig(lines));
      assert.deepStrictEqual(frame.pairs, pairs);
      assert.equal(frame.count, pairs.length);
    }), { numRuns: 500 });
});

test('P29 content with a bad line', () => {
  checkProperty({ n: 29, title: 'Tool inputs parse to Parameter_IDs or report the bad line', unit: 'robot_params.txt with a bad line' },
    fc.property(fc.array(validItem, { maxLength: 20 }), fc.nat(codec.PARAM_COUNT - 1), fc.nat(3), fc.nat(1000),
      (items, index, kind, where) => {
        const [lines] = linesOf(items);
        const [name, text] = badLine(index, kind);
        const at = where % (lines.length + 1);
        lines.splice(at, 0, text);
        assert.throws(() => codec.encodeConfig(lines), (e) => {
          assert.ok(e instanceof codec.CodecError);
          assert.deepStrictEqual(e.errors.map((x) => x.slice(0, 2)), [[at + 1, name]]);
          return true;
        });
      }), { numRuns: 500 });
});

// ---------------- Property 30 ----------------

/** Format_Version 1, then the [value, width] fields, as bytes padded with zeros. */
function headerBits(...fields) {
  let bits = (1).toString(2).padStart(4, '0') + fields.map(([v, w]) => v.toString(2).padStart(w, '0')).join('');
  bits += '0'.repeat((8 - bits.length % 8) % 8);
  return Uint8Array.from(bits.match(/.{8}/g).map((b) => parseInt(b, 2)));
}

// Feature: minimum-bit-data-representation, Property 30: Unknown codes are shown numerically
test('P30 unknown Event_Code', () => {
  checkProperty({ n: 30, title: 'Unknown codes are shown numerically', unit: 'event' },
    fc.property(fc.integer({ min: codec.EVENTS.length, max: 63 }), fc.nat(2 ** 24 - 1), (code, t) => {
      const logs = [];
      const shown = codec.decode('is02/event', headerBits([t, 24], [code, 6]), quiet(logs));
      assert.ok(shown);
      assert.equal(shown.event, code);
      assert.equal(shown.t, t);
      assert.ok(logs.some((line) => line.includes(String(code))));
    }), { numRuns: 200 });
});

test('P30 unknown Reason_Code', () => {
  checkProperty({ n: 30, title: 'Unknown codes are shown numerically', unit: 'ack reason' },
    fc.property(fc.integer({ min: codec.REASONS.length, max: 31 }), fc.constantFrom(1, 2, 3, codec.WCMD_UNPARSED),
      fc.nat(2 ** 24 - 1), (reason, cmd, t) => {
        const logs = [];
        const shown = codec.decode('is02/ack', headerBits([t, 24], [cmd, 4], [reason, 5]), quiet(logs));
        assert.ok(shown);
        assert.equal(shown.result, reason);
        assert.ok(logs.some((line) => line.includes(String(reason))));
      }), { numRuns: 200 });
});

test('P30 unknown command code', () => {
  checkProperty({ n: 30, title: 'Unknown codes are shown numerically', unit: 'ack command' },
    fc.property(fc.integer({ min: codec.COMMANDS.length, max: 14 }), fc.nat(codec.REASONS.length - 1),
      fc.nat(2 ** 24 - 1), (cmd, reason, t) => {
        const logs = [];
        const shown = codec.decode('is02/ack', headerBits([t, 24], [cmd, 4], [reason, 5]), quiet(logs));
        assert.ok(shown);
        assert.equal(shown.cmd, cmd);
        assert.ok(logs.some((line) => line.includes(String(cmd))));
      }), { numRuns: 200 });
});
