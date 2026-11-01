'use strict';
/**
 * Helpers of the cross-codec tests (LT-04): the C side's tables and the
 * generated cases, and a property that replays a list of cases.
 *
 * The JavaScript codec cannot call the C Wire_Codec, so tests/host/py/cross_cases.py
 * does: it writes the cases (records with bytes from the C Wire_Codec) to
 * build/cross/cases_<seed>.json, and loadCases() runs it with the suite seed.
 */
const fs = require('node:fs');
const path = require('node:path');
const cp = require('node:child_process');
const fc = require('fast-check');
const { checkProperty, suiteSeed } = require('./prop');

const HOST = path.join(__dirname, '..');
const BUILD = path.join(HOST, 'build');
const codec = require(path.join(HOST, '..', '..', 'MQTT', 'is02_codec.js'));

function makeTools() {
  cp.execFileSync('make', ['-s', '-C', HOST, 'tools'], { stdio: 'pipe' });
  return BUILD;
}

/** The C side's tables: build/wire_dump as an object. */
function wireDump() {
  makeTools();
  return JSON.parse(cp.execFileSync(path.join(BUILD, 'wire_dump'), { encoding: 'utf8', maxBuffer: 1 << 24 }));
}

/** The cases for the suite seed (made by py/cross_cases.py when not there yet). */
function loadCases() {
  const seed = suiteSeed();
  const file = path.join(BUILD, 'cross', `cases_${seed}.json`);
  if (!fs.existsSync(file)) {
    const python = process.env.PYTHON || 'python3';
    cp.execFileSync(python, [path.join(HOST, 'py', 'cross_cases.py'), '--seed', String(seed), '--out', file],
      { stdio: 'pipe' });
  }
  return JSON.parse(fs.readFileSync(file, 'utf8'));
}

const bytes = (hex) => Uint8Array.from(Buffer.from(hex, 'hex'));
const hex = (data) => Buffer.from(data).toString('hex');

/**
 * One property run per item: the n-th run checks items[n]. check(item) throws on a
 * failure; the message says which item. meta = { n, title, unit }.
 */
function sweep(meta, items, check) {
  let run = 0;
  checkProperty(meta, fc.property(fc.constant(0), () => {
    const index = run++ % items.length;
    try {
      check(items[index]);
    } catch (e) {
      throw new Error(`case ${index} of ${items.length}: ${e.message}`);
    }
  }), { numRuns: items.length });
}

module.exports = { HOST, BUILD, codec, makeTools, wireDump, loadCases, bytes, hex, sweep };
