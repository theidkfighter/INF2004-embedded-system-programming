'use strict';
// Presentation layer of the JavaScript Laptop_Codec against the Baseline_Build JSON (spec
// task 14.17): every baseline key is there, the sentinels show as the baseline showed
// them, and a payload with a wrong version or length is dropped without any field.
// py/test_presentation.py does the same for Python.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const { codec, loadCases, bytes } = require('./cross_helper');

const quiet = (logs) => ({ legacyJson: false, log: (line) => logs.push(line) });
const TOPIC = { telemetry: 'is02/telemetry', heartbeat: 'is02/status', event: 'is02/event', ack: 'is02/ack',
  command: 'is02/cmd', map: 'is02/map', config: 'is02/config' };

// Keys of the Baseline_Build telemetry and heartbeat JSON (bca7964 tsk_telemetry.c)
const TELEMETRY_KEYS = ['seq', 't', 'state', 'spd', 'duty', 'enc', 'dist', 'line.l', 'line.r', 'line.ir', 'line.irmn',
  'line.irmx', 'line.thr', 'bc', 'hump.pitch', 'hump.h', 'hump.peak', 'hump.max', 'obst.cm', 'obst.scan', 'obst.side',
  'imu.ev', 'imu.tr', 'faults', 'vbat', 'rssi', 'buf'];
const STATUS_KEYS = ['seq', 't', 'state', 'rssi', 'buf', 'up', 'reconnects', 'dropped', 'faults', 'broker'];

const has = (obj, key) => key.split('.').reduce((o, k) => (o !== undefined && k in o ? o[k] : undefined), obj) !== undefined;

const cases = loadCases();

test('telemetry has every baseline key', () => {
  for (const { hex } of cases.p25.telemetry.slice(0, 50)) {
    const shown = codec.decode(TOPIC.telemetry, bytes(hex), quiet([]));
    assert.deepStrictEqual(TELEMETRY_KEYS.filter((k) => !has(shown, k)), []);
    assert.equal(typeof shown.state, 'string');
    assert.equal(typeof shown.bc, 'string');
  }
});

test('status has every baseline key', () => {
  for (const { rec, hex } of cases.p25.heartbeat.slice(0, 50)) {
    const shown = codec.decode(TOPIC.heartbeat, bytes(hex), quiet([]));
    assert.deepStrictEqual(STATUS_KEYS.filter((k) => !has(shown, k)), []);
    assert.equal(shown.broker, rec.broker.join('.'));
  }
});

function telemetry(changes) {
  const rec = Object.assign({}, cases.p25.telemetry[3].rec, changes);
  return codec.decode(TOPIC.telemetry, codec.encodeFields('telemetry', rec), quiet([]));
}

test('sentinels show as the baseline showed them', () => {
  let shown = telemetry({ us: 8191, scan: [8191, 0, 4310, 8191, 12], bc: 63, side: 0, thr_valid: 0, thr_b: 0, thr_w: 0 });
  assert.equal(shown.obst.cm, -1.0);
  assert.deepStrictEqual(shown.obst.scan, [-1.0, 0.0, 431.0, -1.0, 1.2]);
  assert.equal(shown.bc, '');
  assert.equal(shown.obst.side, '');
  assert.deepStrictEqual(shown.line.thr, [0, 0]);
  shown = telemetry({ thr_valid: 1, thr_b: 2000, thr_w: 900, bc: 20, side: 2 });
  assert.deepStrictEqual(shown.line.thr, [2000, 900]);
  assert.equal(shown.bc, 'K');
  assert.equal(shown.obst.side, 'R');
});

const RETAINED = ['is02/map', 'is02/config'];

for (const kind of Object.keys(TOPIC)) {
  test(`${kind}: a wrong version or length is dropped without fields`, () => {
    const good = bytes(cases.p27[kind][0]);
    assert.notEqual(codec.decode(TOPIC[kind], good, quiet([])), null);
    const bad = [
      ['bad version', Uint8Array.from([(good[0] & 0x0F) | 0x20, ...good.slice(1)])],
      ['one byte short', good.slice(0, -1)],
      ['one byte long', Uint8Array.from([...good, 0])],
      ['zero length', new Uint8Array(0)],
    ];
    for (const [why, payload] of bad) {
      const logs = [];
      assert.throws(() => codec.decodeFields(kind, payload), codec.CodecError, `${kind} ${why}`);
      const shown = codec.decode(TOPIC[kind], payload, quiet(logs));
      assert.equal(shown, null, `${kind} ${why}`);
      if (why === 'zero length' && RETAINED.includes(TOPIC[kind])) {
        assert.deepStrictEqual(logs, []);      // an empty retained payload is the broker deleting it
      } else {
        assert.equal(logs.length, 1, `${kind} ${why}: ${logs}`);
        assert.ok(logs[0].includes(TOPIC[kind]));
      }
    }
  });
}
