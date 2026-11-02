'use strict';
// Deliberately failing property for `run_all.sh --selfcheck`
// (Requirements 18.8, 18.9).
//
// The file name matches none of the `node --test` default patterns
// (*.test.js, *-test.js, *_test.js, test-*.js, test.js, test/**), so a normal
// `node --test` run (and so a normal run_all.sh) never runs it.
// `run_all.sh --selfcheck` names it on the command line; test_selfcheck.sh
// then checks that the summary reports the property, the Storage_Unit, the
// shrunk input ([1000]) and the seed, and that run_all.sh exits non-zero.
//
// Do not "fix" the predicate: the failure is the point.
const { test } = require('node:test');
const fc = require('fast-check');
const { checkProperty } = require('./prop');

test('P0 selfcheck (JavaScript)', () => {
  checkProperty(
    { n: 0, title: 'Selfcheck deliberately failing property (JavaScript)', unit: 'selfcheck.u16' },
    // False for 1000..65535; fast-check shrinks the counterexample to [1000].
    fc.property(fc.integer({ min: 0, max: 65535 }), (v) => v < 1000),
    { numRuns: 100 });
});
