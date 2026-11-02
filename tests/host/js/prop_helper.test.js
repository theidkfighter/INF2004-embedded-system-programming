'use strict';
// Unit tests for prop.js: the records it writes for run_all.sh and the
// failure report. Records go to a temporary file, not to the suite results.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const fc = require('fast-check');
const { checkProperty } = require('./prop');

function withResults(fn) {
  const saved = process.env.IS02_RESULTS;
  const savedSeed = process.env.IS02_SEED;
  const file = path.join(fs.mkdtempSync(path.join(os.tmpdir(), 'is02-')), 'r.jsonl');
  process.env.IS02_RESULTS = file;
  process.env.IS02_SEED = '424242';
  try {
    fn();
    return fs.readFileSync(file, 'utf8').trim().split('\n').map((l) => JSON.parse(l));
  } finally {
    if (saved === undefined) delete process.env.IS02_RESULTS; else process.env.IS02_RESULTS = saved;
    if (savedSeed === undefined) delete process.env.IS02_SEED; else process.env.IS02_SEED = savedSeed;
  }
}

test('a passing property records cases, seed and no failure', () => {
  const recs = withResults(() => {
    checkProperty({ n: 0, title: 'helper pass', unit: 'u8' },
      fc.property(fc.integer({ min: 0, max: 255 }), (v) => v >= 0), { numRuns: 150 });
  });
  assert.equal(recs.length, 1);
  assert.equal(recs[0].cases, 150);
  assert.equal(recs[0].failed, 0);
  assert.equal(recs[0].seed, '424242');
  assert.equal(recs[0].unit, 'u8');
});

test('a failing property throws with the seed and records the counterexample', () => {
  const recs = withResults(() => {
    assert.throws(
      () => checkProperty({ n: 0, title: 'helper fail', unit: 'u8' },
        fc.property(fc.integer({ min: 0, max: 255 }), (v) => v < 10)),
      /P0 helper fail \| u8 \| seed=424242/);
  });
  assert.equal(recs[0].failed, 1);
  assert.equal(recs[0].input, '[10]');      // shrunk to the smallest failing value
});

test('fewer than 100 runs are raised to 100', () => {
  const recs = withResults(() => {
    checkProperty({ n: 0, title: 'helper min runs' }, fc.property(fc.boolean(), () => true), { numRuns: 5 });
  });
  assert.equal(recs[0].cases, 100);
});
