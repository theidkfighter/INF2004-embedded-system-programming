'use strict';
// Drift guard (INTEGRATION.md 1.4): the JavaScript Laptop_Codec's tables against the C
// side (build/wire_dump JSON). py/test_codec_tables.py does the same for Python. A
// difference fails the test and names the codec and the table.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const { codec, wireDump } = require('./cross_helper');

const CODEC = 'JavaScript Laptop_Codec (MQTT/is02_codec.js)';
const dump = wireDump();

function same(what, got, want) {
  assert.deepStrictEqual(got, want, `${CODEC}: ${what} differs from wire_dump`);
}

test('format version', () => same('format_version', codec.FORMAT_VERSION, dump.format_version));

test('event table', () => {
  same('events', codec.EVENTS, dump.events.map((e) => [e.name, e.arg_bits, e.variable]));
  same('event codes', codec.EVENTS.map((_, i) => i), dump.events.map((e) => e.code));
});

test('reason table', () => {
  same('reasons', codec.REASONS, dump.reasons.map((r) => [r.name, r.text]));
  same('RSN_CONFIG_REJECTED', codec.RSN_CONFIG_REJECTED, dump.reasons.find((r) => r.name === 'CONFIG_REJECTED').code);
});

test('command table', () => {
  same('commands', codec.COMMANDS, dump.commands.map((c) => [c.name, c.echo, c.cmd_len, c.ack_len]));
});

test('stop and abort texts', () => {
  same('stop reasons', codec.STOP_REASONS, dump.stop_reasons.map((s) => s.text));
  same('abort reasons', codec.ABORT_REASONS, dump.abort_reasons.map((s) => s.text));
});

test('parameter table', () => {
  const width = { F32: 32, F1: 1, S2: 2, 'B2-1': 2, 'B3+1': 3 };
  same('parameter names and store widths', codec.PARAMS.map((p) => [p[0], width[p[1]]]),
    dump.params.map((p) => [p.name, p.store_width]));
  same('parameter table version', [codec.PARAM_COUNT, codec.PARAM_CRC], [dump.params.length, dump.param_table_crc]);
});

function expand(layout) {
  const out = [];
  let pos = 4;
  for (const field of layout) {
    for (let i = 0; i < (field.length === 2 ? 1 : field[2]); i++) {
      out.push([Math.abs(field[1]), pos]);
      pos += Math.abs(field[1]);
    }
  }
  return [out, pos];
}

for (const [name, layout, bits, length] of [
  ['telemetry', codec.TELEMETRY, codec.TLM_BITS, codec.TLM_LEN],
  ['heartbeat', codec.HEARTBEAT, codec.HB_BITS, codec.HB_LEN],
]) {
  test(`${name} layout`, () => {
    const frame = dump.frames[name];
    const [got, end] = expand(layout);
    same(`${name} field widths and offsets`, got,
      frame.fields.slice(1).filter((f) => f.kind !== 'PAD').map((f) => [f.width, f.offset]));
    same(`${name} bits`, [end, bits, length], [frame.bits, frame.bits, frame.len]);
  });
}

test('map length and header bits', () => {
  same('map length', codec.MAP_LEN, dump.frames.map.len);
  same('event header bits', codec.EVT_HDR_BITS, dump.frames.event_header.bits);
  same('ack header bits', codec.ACK_HDR_BITS, dump.frames.ack_header.bits);
});
