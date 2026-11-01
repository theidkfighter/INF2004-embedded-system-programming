'use strict';
/**
 * Host_Test_Suite helper for the fast-check property tests.
 *
 * Runs one property with the seed chosen by run_all.sh (IS02_SEED) and writes
 * one JSON line to the file named by IS02_RESULTS, which run_all.sh turns into
 * the per-property summary (Requirements 18.8, 18.9). Usage in a *.test.js:
 *
 *   // Feature: minimum-bit-data-representation, Property <n>: <title>
 *   const { test } = require('node:test');
 *   const fc = require('fast-check');
 *   const { checkProperty } = require('./prop');
 *
 *   test('P<n> ...', () => {
 *     checkProperty({ n: <n>, title: '<title>', unit: '<payload type>' },
 *                   fc.property(arb, (v) => { ... }), { numRuns: 1000 });
 *   });
 */
const fs = require('node:fs');
const fc = require('fast-check');

const MIN_RUNS = 100;

/** Seed from run_all.sh, or a fresh one when a test file is run on its own. */
function suiteSeed() {
  const s = process.env.IS02_SEED;
  if (s !== undefined && s !== '' && Number.isFinite(Number(s))) {
    return Number(s) | 0;
  }
  return (Date.now() ^ (Math.random() * 0x7fffffff)) | 0;
}

function record(entry) {
  const path = process.env.IS02_RESULTS;
  if (path) {
    fs.appendFileSync(path, JSON.stringify(entry) + '\n');
  }
}

/**
 * Run `property` (fc.property / fc.asyncProperty is not supported here) and
 * throw with the counterexample and seed if it fails.
 * meta: { n, title, unit }; params: extra fast-check parameters (numRuns ...).
 */
function checkProperty(meta, property, params = {}) {
  const seed = suiteSeed();
  const numRuns = Math.max(MIN_RUNS, params.numRuns || MIN_RUNS);
  const details = fc.check(property, { ...params, seed, numRuns });
  const failed = details.failed ? 1 : 0;
  const input = details.failed ? fc.stringify(details.counterexample) : '';
  record({
    runner: 'js',
    kind: 'prop',
    n: meta.n,
    title: meta.title,
    unit: meta.unit || '-',
    cases: details.numRuns,
    failed,
    skipped: false,
    seed: String(seed),
    phase: 'call',
    nodeid: `js:${meta.title}`,
    input,
  });
  if (details.failed) {
    throw new Error(`P${meta.n} ${meta.title} | ${meta.unit || '-'} | seed=${seed}\n` +
                    fc.defaultReportMessage(details));
  }
  return details;
}

module.exports = { checkProperty, suiteSeed, MIN_RUNS };
