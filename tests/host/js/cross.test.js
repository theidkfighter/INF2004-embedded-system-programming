'use strict';
// Cross-codec properties for the JavaScript Laptop_Codec (spec tasks 14.11 to 14.14 and
// 16.4): Properties 16, 25, 26, 27 and 28. The bytes come from the C Wire_Codec or were
// read back by it (py/cross_cases.py, see cross_helper.js); py/test_codec_cross.py
// runs the same properties for the Python codec.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { codec, HOST, loadCases, bytes, hex, sweep } = require('./cross_helper');

const cases = loadCases();
const T16 = 'Event detail text can be rebuilt';
const T25 = 'Robot-to-laptop frames round-trip through both Laptop_Codecs';
const T26 = 'Laptop-to-robot frames round-trip into the Wire_Codec';
const T27 = 'Decode then encode is the identity on bytes';
const T28 = 'The three codecs agree on the shared vectors';

const TOPIC = { telemetry: 'is02/telemetry', heartbeat: 'is02/status' };
const quiet = { log: () => {} };

const cm = (code, none) => (code === none ? -1.0 : code / 10);

function expectedTelemetry(r) {
  return {
    seq: r.seq, t: r.t, spd: [r.spd_l / 10, r.spd_r / 10], duty: [r.duty_l / 10, r.duty_r / 10],
    enc: [r.enc_l, r.enc_r], dist: r.dist / 10, faults: r.faults, vbat: r.vbat, rssi: 0 - r.rssi, buf: r.buf,
    hump: { pitch: r.pitch / 10, h: r.hump_h / 10, peak: r.hump_peak / 10, max: r.hump_max / 10 },
    obst: { cm: cm(r.us, 8191), scan: r.scan.map((c) => cm(c, 8191)) },
    imu: { tr: r.turn / 10 },
    line: { l: r.line_l, r: r.line_r, ir: r.ir, irmn: r.ir_min, irmx: r.ir_max, thr: r.thr_valid ? [r.thr_b, r.thr_w] : [0, 0] },
  };
}

function expectedStatus(r) {
  return {
    seq: r.seq, t: r.t, rssi: 0 - r.rssi, buf: r.buf, up: r.up, reconnects: r.reconnects, dropped: r.dropped,
    faults: r.faults, sat: r.sat, broker: r.broker.join('.'), ptab: { count: r.ptab_count, crc: r.ptab_crc },
  };
}

function subset(got, want, where = '') {
  for (const [key, value] of Object.entries(want)) {
    if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
      subset(got[key], value, `${where}${key}.`);
    } else {
      assert.deepStrictEqual(got[key], value, `${where}${key}`);
    }
  }
}

// Feature: minimum-bit-data-representation, Property 25: Robot-to-laptop frames round-trip through both Laptop_Codecs
function roundTrip25(kind) {
  return ({ rec, hex: h }) => {
    assert.deepStrictEqual(codec.decodeFields(kind, bytes(h)), rec);
    if (kind in TOPIC) {
      subset(codec.decode(TOPIC[kind], bytes(h), quiet), kind === 'telemetry' ? expectedTelemetry(rec) : expectedStatus(rec));
    }
  };
}

for (const kind of ['telemetry', 'heartbeat', 'ack']) {
  test(`P25 ${kind}`, () => sweep({ n: 25, title: T25, unit: kind }, cases.p25[kind], roundTrip25(kind)));
}
for (const [name, items] of Object.entries(cases.p25.event)) {
  test(`P25 event ${name}`, () => sweep({ n: 25, title: T25, unit: name }, items, roundTrip25('event')));
}

// Feature: minimum-bit-data-representation, Property 26: Laptop-to-robot frames round-trip into the Wire_Codec
function roundTrip26(kind) {
  return ({ rec, hex: h }) => assert.equal(hex(codec.encodeFields(kind, rec)), h);
}

for (const [code, items] of Object.entries(cases.p26.command)) {
  const name = codec.COMMANDS[Number(code)][0];
  test(`P26 command ${name}`, () => sweep({ n: 26, title: T26, unit: name }, items, roundTrip26('command')));
}
for (const kind of ['map', 'config']) {
  test(`P26 ${kind}`, () => sweep({ n: 26, title: T26, unit: kind }, cases.p26[kind], roundTrip26(kind)));
}

// Feature: minimum-bit-data-representation, Property 27: Decode then encode is the identity on bytes
for (const [kind, items] of Object.entries(cases.p27)) {
  test(`P27 ${kind}`, () => sweep({ n: 27, title: T27, unit: kind }, items,
    (h) => assert.equal(hex(codec.encodeFields(kind, codec.decodeFields(kind, bytes(h)))), h)));
}

// Feature: minimum-bit-data-representation, Property 28: The three codecs agree on the shared vectors
const vectors = JSON.parse(fs.readFileSync(path.join(HOST, 'vectors', 'vectors.json'), 'utf8')).vectors;

test('P28 shared vectors', () => sweep({ n: 28, title: T28, unit: 'shared vectors' }, vectors, (vec) => {
  if (vec.reject) {
    assert.throws(() => codec.decodeFields(vec.kind, bytes(vec.hex)), codec.CodecError, `${vec.kind} ${vec.reject}`);
    return;
  }
  assert.equal(hex(codec.encodeFields(vec.kind, vec.record)), vec.hex, 'encode');
  assert.deepStrictEqual(codec.decodeFields(vec.kind, bytes(vec.hex)), vec.record, 'decode');
}));

// Feature: minimum-bit-data-representation, Property 16: Event detail text can be rebuilt
test('P16 event detail text', () => sweep({ n: 16, title: T16, unit: 'all events' }, cases.p16, (c) => {
  const shown = codec.decode('is02/event', bytes(c.hex), quiet);
  assert.equal(shown.event, c.event);
  assert.equal(shown.detail, c.text);
}));

test('golden Baseline_Build event texts', () => {
  assert.ok(cases.golden.length >= codec.EVENTS.length);
  for (const g of cases.golden) {
    const shown = codec.decode('is02/event', bytes(g.hex), quiet);
    assert.deepStrictEqual([shown.event, shown.detail], [g.event, g.text]);
  }
});
