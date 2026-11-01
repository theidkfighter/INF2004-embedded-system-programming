"use strict";
/*
 * Quick self-tests of the JavaScript Laptop_Codec (MQTT/is02_codec.js, tasks 14.3
 * and 14.4): the same hand-made records, hand-built frames, CRC check value and
 * event texts as py/test_codec_basic.py. The table checks against wire_ids.h and
 * app_params.c live in the Python test; the cross-codec vectors are LT-04.
 */

const test = require("node:test");
const assert = require("node:assert/strict");
const path = require("node:path");

const codec = require(path.join(__dirname, "..", "..", "..", "MQTT", "is02_codec.js"));

const TELEMETRY = {
    seq: 32767, t: 16777215, state: 2, spd_l: -1234, spd_r: 1000, duty_l: -1000, duty_r: 455,
    enc_l: -8388607, enc_r: 8388607, dist: 1000000, line_l: -1, line_r: 1, ir: 4095, ir_min: 12,
    ir_max: 3000, thr_valid: 1, thr_b: 2000, thr_w: 900, bc: 20, pitch: -900, hump_h: -15000,
    hump_peak: 15000, hump_max: 1234, us: 8191, scan: [8191, 4310, 0, 123, 8191], side: 2, imu_ev: 7,
    turn: -7200, faults: 0xA005, vbat: 6600, rssi: 128, buf: 64,
};

const HEARTBEAT = {
    seq: 16383, t: 123456, state: 1, rssi: 57, buf: 3, up: 131071, reconnects: 65535,
    dropped: 2, sat: 7, faults: 0x6000, ptab_count: 55, ptab_crc: 0xBEEF, broker: [192, 168, 1, 20],
};

const MAP_ACTS = new Array(26).fill(0);
MAP_ACTS["F".charCodeAt(0) - 65] = 1;
MAP_ACTS["K".charCodeAt(0) - 65] = 3;
MAP_ACTS["R".charCodeAt(0) - 65] = 4;
MAP_ACTS["U".charCodeAt(0) - 65] = 2;
const NO_ACTS = new Array(26).fill(0);
const MAP_TEXT = "F=FWD,K=LEFT,R=RIGHT,U=UTURN";

const LEVELS = { n_levels: 6 };
[[1, 35, 29], [2, 0, 27], [0, 0, 3], [1, 35, 28], [2, 0, 26], [0, 0, 0]].forEach(([res, code, n], i) => {
    LEVELS["res" + i] = res;
    LEVELS["code" + i] = code;
    LEVELS["n" + i] = n;
});
const WINDOW = { n: 29, res: 1, code: 20, rev: 1, thr_b: 1500, thr_w: 900, ovr: 2 };

// [Event name, args, the Baseline_Build detail text]
const EVENT_CASES = [
    ["STATE", { state: 2 }, "LINE_FOLLOWING"],
    ["IMU_OK", { chip: 1, addr: 0x19 }, "LSM303DLHC at 0x19"],
    ["IMU_OK", { chip: 2, addr: 0x1E }, "LSM303D at 0x1E"],
    ["IMU_OK", { chip: 0, addr: 0 }, "none"],
    ["FAULT_IMU_I2C", {}, "I2C1 timeout: bus recovery"],
    ["FAULT_IMU", {}, "3 failed recoveries: driving without the IMU"],
    ["HUMP_START", { valid: 1, h: 0, max: 1234 }, "height 0.0 mm, run max 123.4 mm"],
    ["HUMP_PEAK", { valid: 1, h: 505, max: 1234 }, "height 50.5 mm, run max 123.4 mm"],
    ["HUMP_END", { valid: 1, h: 505, max: 1234 }, "height 50.5 mm, run max 123.4 mm"],
    ["HUMP_END", { valid: 0, h: 0, max: 0 }, "false slope (no height reported)"],
    ["MAP", { map: MAP_ACTS }, MAP_TEXT],
    ["MAP", { map: NO_ACTS }, "cleared"],
    ["BATTERY_LOW", {}, "below VBAT_LOW_MV"],
    ["START_REFUSED", {}, "calibration missing (see console)"],
    ["MAP_WARNING", { map: MAP_ACTS }, MAP_TEXT],
    ["MAP_WARNING", { map: NO_ACTS }, "no barcode map yet (send MAP:...)"],
    ["MISSION_COMPLETE", { t: 95, dist: 1205, bc: 4, humps: 2, max: 305, obst: 1 },
        "t 95s dist 12.05m bc 4 humps 2 max 30.5mm obst 1"],
    ["TURN_SNAP", { not_seen: 0 }, "stopped on the line"],
    ["TURN_SNAP", { not_seen: 1 }, "line not seen, encoder angle kept"],
    ["NAV_ACTION", { letter: 10, action: 3 }, "K -> LEFT"],
    ["NAV_ACTION", { letter: 25, action: 0 }, "Z -> NONE (not in the MQTT map): straight on"],
    ["NAV", { variant: 0, letter: 0, action: 4 }, "RIGHT"],
    ["NAV", { variant: 1, letter: 0, action: 0 }, "turn did not complete"],
    ["NAV", { variant: 2, letter: 10, action: 3 }, "K stored for the next junction (LEFT)"],
    ["NAV", { variant: 3, letter: 2, action: 0 }, "barcode fault: going straight (C)"],
    ["EMERGENCY_STOP", { reason: 1 }, "remote E-stop (MQTT)"],
    ["ERROR", { reason: 9 }, "fault reported (see faults in telemetry)"],
    ["RECOVERED", {}, "re-check passed"],
    ["RECOVERY_REFUSED", { why: 0b11 }, "obstacle closer than 8 cm. IMU INT1 still active. "],
    ["RECOVERY_REFUSED", { why: (1 << 2) | (1 << 14) }, "MOTOR_MAPPING_CONFIRMED, IMU_FORWARD_SIGN"],
    ["IMPACT", {}, "while not driving"],
    ["BARCODE", { code: 20, rev: 0 }, "K (read forwards)"],
    ["BARCODE", { code: 20, rev: 1 }, "K (read backwards)"],
    ["BARCODE_UNKNOWN", { code: 7, rev: 0 }, "7 (read forwards)"],
    ["BARCODE_FAULT", {}, "3 ambiguous barcodes: going straight (C)"],
    ["BARCODE_FAILED", {}, "window did not decode"],
    ["LINE_FOUND", {}, "search"],
    ["LINE_LOST", {}, "search found nothing"],
    ["LINE_SEARCH", { right: 0 }, "left first"],
    ["LINE_SEARCH", { right: 1 }, "right first"],
    ["FINE_SCAN", { first: -60, count: 3, cm: [41, 511, 40] }, "-60deg+2: 41 -1 40"],
    ["SIDE_FOLLOW", { n60: 12, seen: 9, near: 225, limit: 330, leg: 412, x: 180 },
        "60deg pings 12 seen 9 near 22.5cm (<=33.0) leg 412 x 180"],
    ["SIDE_FOLLOW", { n60: 3, seen: 0, near: 8191, limit: 330, leg: 40, x: 0 },
        "60deg pings 3 seen 0 near -1.0cm (<=33.0) leg 40 x 0"],
    ["SIDE_LOOK", { d90: 305, d60: 8191, gone: 1 }, "pivot look 90deg 30.5 cm, 60deg -1.0 cm: side gone"],
    ["SIDE_LOOK", { d90: 305, d60: 400, gone: 0 }, "pivot look 90deg 30.5 cm, 60deg 40.0 cm: side still there"],
    ["BYPASS_EXTEND", { variant: 0, cm: 25 }, "obstacle still beside the car (25 cm): 100 mm further"],
    ["BYPASS_EXTEND", { variant: 1, cm: 511 }, "return blocked (-1 cm): back to the turn, 100 mm further"],
    ["BYPASS_SIDESTEP", { mm: 60 }, "too close beside the obstacle: back, then 60 mm further out"],
    ["BYPASS_RETURN", {}, "line not seen crossing the original path: back to it, then search"],
    ["OBSTACLE_PROFILE", { r: 22, w: 30, off: -4, clr_l: 12, clr_r: -3, side: 1 },
        "r 22cm w 30cm off -4 clrL 12 clrR -3 side L"],
    ["OBSTACLE_PROFILE", { r: 511, w: 0, off: 0, clr_l: 0, clr_r: 0, side: 0 },
        "r -1cm w 0cm off 0 clrL 0 clrR 0 side -"],
    ["AVOIDANCE_ABORT", { reason: 10 }, "too many attempts"],
    ["OBST_REVERSE_CAP", { variant: 0, n: 90, used: 300, cap: 350, total: 700 },
        "standoff 90 mm refused: 300 of 350 mm used, 700 mm total"],
    ["OBST_REVERSE_CAP", { variant: 1, n: 0, used: 300, cap: 350, total: 700 },
        "rescan skipped: 300 of 350 mm used, 700 mm total"],
    ["OBST_REVERSE_CAP", { variant: 2, n: 50, used: 300, cap: 350, total: 700 },
        "rescan reverse cut to 50 mm: 300 of 350 mm used, 700 mm total"],
    ["OBSTACLE_BACKOFF", { need: 75, right: 0 }, "reversing 75 mm: a 45 deg turn would clip the left corner"],
    ["OBSTACLE_BACKOFF", { need: 75, right: 1 }, "reversing 75 mm: a 45 deg turn would clip the right corner"],
    ["OBSTACLE_RESCAN", { blocked: 0 }, "no room: reversing 100 mm and scanning again"],
    ["OBSTACLE_RESCAN", { blocked: 1 }, "blocked: reversing 100 mm and scanning again"],
    ["OBSTACLE_TIE", { dl: 200, dr: 34, side: 2 }, "tie: 90 deg look L 200 cm, R 34 cm -> side R"],
    ["OBSTACLE_CLEARED", {}, "coarse sweep clear: resume line following"],
    ["BYPASS_COMPLETE", { right: 1, search: 0 }, "passed on the right"],
    ["BYPASS_COMPLETE", { right: 0, search: 0 }, "passed on the left"],
    ["BYPASS_COMPLETE", { right: 1, search: 1 }, "passed on the right; on the path, searching for the line"],
    ["BYPASS_COMPLETE", { right: 0, search: 1 }, "passed on the left; on the path, searching for the line"],
    ["BYPASS_BLOCKED", { right: 1, no_end: 0, cm: 18, deg: -30 }, "right side: something at 18 cm, -30 deg; backing out"],
    ["BYPASS_BLOCKED", { right: 0, no_end: 1, cm: 0, deg: 0 }, "left side: obstacle side did not end in 1 m; backing out"],
    ["BYPASS_RETRY", { right: 1, wider: 0 }, "trying the right side (same scan)"],
    ["BYPASS_RETRY", { right: 0, wider: 0 }, "trying the left side (same scan)"],
    ["BYPASS_RETRY", { right: 1, wider: 1 }, "right side again, 100 mm wider"],
    ["BYPASS_RETRY", { right: 0, wider: 1 }, "left side again, 100 mm wider"],
    ["OBSTACLE", { cm: 198, us: 412 }, "at 19.8 cm, STOP issued in 412 us"],
    ["BC_DBG", { variant: 0 }, "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close"],
    ["BC_DBG", { ...WINDOW, variant: 1 }, "n=29 decoded K (backwards), thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 1, n: 5, res: 0 }, "n=5 stray mark (fewer than 9), thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 1, n: 9, res: 0 }, "n=9 closed, thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 1, res: 2 }, "n=29 NOT decoded, thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 2, rev: 0, level: 5, conflict: 0 },
        "n=29 decoded K (forwards) at adapt, thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 2, n: 5, res: 0, level: 0, conflict: 0 },
        "n=5 stray mark (fewer than 9), thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 2, res: 2, level: 0, conflict: 1 },
        "n=29 NOT decoded: levels disagree, thr 1500/900, ovr 2"],
    ["BC_DBG", { ...WINDOW, variant: 2, res: 2, level: 0, conflict: 0 }, "n=29 NOT decoded, thr 1500/900, ovr 2"],
    ["BC_DBG", { ...LEVELS, variant: 3 }, "levels 50%:Z29 35%:x27 65%:-3 25%:Z28 75%:x26 adapt:-0"],
    ["BC_DBG", { variant: 4, part: 2, count: 3, w: [12, 345, 4095] }, "w2 (0.1mm, bar first): 12 345 4095"],
    ["BC_DBG", { variant: 4, part: 1, count: 0, w: [] }, "w1 (0.1mm, bar first):"],
];

const bytes = (...values) => Uint8Array.from(values);
const bigBytes = (value, length) => {
    const out = new Uint8Array(length);
    for (let i = length - 1; i >= 0; i--) {
        out[i] = Number(value & 0xFFn);
        value >>= 8n;
    }
    return out;
};
const toBig = (data) => data.reduce((acc, b) => (acc << 8n) | BigInt(b), 0n);

function eventFrame(name, args, t = 1000) {
    return codec.encodeFields("event", { t, code: codec.EVC[name], args });
}

function roundTrip(kind, record) {
    const payload = codec.encodeFields(kind, record);
    const decoded = codec.decodeFields(kind, payload);
    assert.deepEqual(decoded, record);
    assert.deepEqual(codec.encodeFields(kind, decoded), payload);
    assert.equal(payload[0] >> 4, codec.FORMAT_VERSION);
    return payload;
}

function layoutBits(layout, upto) {
    let bits = 4;
    for (const field of layout) {
        if (field[0] === upto) {
            break;
        }
        bits += Math.abs(field[1]) * (field.length === 3 ? field[2] : 1);
    }
    return bits;
}

function logged() {
    const lines = [];
    return [lines, (line) => lines.push(line)];
}

function throwsCodec(fn, pattern) {
    assert.throws(fn, (e) => {
        assert.ok(e instanceof codec.CodecError, `CodecError expected, got ${e}`);
        if (pattern) {
            assert.match(e.message, pattern);
        }
        return true;
    });
}

// ---------------- tables ----------------

test("CRC-16/CCITT-FALSE check value", () => {
    assert.equal(codec.crc16CcittFalse("123456789"), 0x29B1);
    assert.equal(codec.crc16CcittFalse(bytes(0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39)), 0x29B1);
});

test("table sizes and the parameter table version", () => {
    assert.equal(codec.EVENTS.length, 45);
    assert.equal(codec.PARAMS.length, 55);
    assert.deepEqual(codec.paramTableVersion(), [55, codec.PARAM_CRC]);
    const names = codec.PARAMS.map((p) => p[0]);
    [names[0], names[1]] = [names[1], names[0]];
    assert.notEqual(codec.paramTableVersion(names)[1], codec.PARAM_CRC);
});

test("packed parameters match D15", () => {
    const packed = {};
    codec.PARAMS.forEach((p, i) => { if (p[1] !== "F32") { packed[i] = p[1]; } });
    assert.deepEqual(packed, {
        0: "S2", 1: "F1", 2: "F1", 3: "S2", 4: "S2", 7: "S2", 8: "F1", 10: "B2-1", 11: "S2",
        33: "F1", 34: "F1", 36: "F1", 39: "B3+1", 41: "F1", 49: "F1", 51: "F1",
    });
});

// ---------------- layouts and lengths ----------------

test("telemetry offsets match D8", () => {
    const offsets = {
        seq: 4, t: 19, state: 43, spd_l: 46, duty_l: 76, enc_l: 98, dist: 146, line_l: 166,
        ir: 170, thr_valid: 206, thr_b: 207, bc: 231, pitch: 237, hump_h: 248, hump_peak: 263,
        us: 291, scan: 304, side: 369, imu_ev: 371, turn: 374, faults: 388, vbat: 404, rssi: 417, buf: 425,
    };
    for (const [name, offset] of Object.entries(offsets)) {
        assert.equal(layoutBits(codec.TELEMETRY, name), offset, name);
    }
    assert.equal(layoutBits(codec.TELEMETRY), codec.TLM_BITS);
});

test("heartbeat offsets match D9", () => {
    const offsets = {
        seq: 4, t: 18, state: 42, rssi: 45, buf: 53, up: 60, reconnects: 77, dropped: 93,
        sat: 109, faults: 125, ptab_count: 141, ptab_crc: 147, broker: 163,
    };
    for (const [name, offset] of Object.entries(offsets)) {
        assert.equal(layoutBits(codec.HEARTBEAT, name), offset, name);
    }
    assert.equal(layoutBits(codec.HEARTBEAT), codec.HB_BITS);
});

test("fixed event argument bits match the table", () => {
    for (const [name, bits, variable] of codec.EVENTS) {
        if (!variable) {
            const sum = (codec.EVENT_ARGS[name] || []).reduce((s, f) => s + Math.abs(f[1]) * (f.length === 3 ? f[2] : 1), 0);
            assert.equal(sum, bits, name);
        }
    }
});

test("variable event lengths", () => {
    assert.equal(eventFrame("BC_DBG", { variant: 4, part: 1, count: 16, w: new Array(16).fill(1) }).length, codec.EVT_MAX_LEN);
    assert.equal(eventFrame("FINE_SCAN", { first: 0, count: 1, cm: [1] }).length, codec.evtLen(20));
    assert.equal(eventFrame("BC_DBG", { ...WINDOW, variant: 1 }).length, codec.evtLen(59));
    assert.equal(eventFrame("BC_DBG", { ...WINDOW, variant: 2, level: 0, conflict: 0 }).length, codec.evtLen(63));
    assert.equal(eventFrame("BC_DBG", { ...LEVELS, variant: 3 }).length, codec.evtLen(96));
});

test("config lengths", () => {
    assert.equal(codec.cfgLen(0), 2);
    assert.equal(codec.cfgLen(55), 263);
    assert.equal(codec.cfgAckLen(55), 48);
});

// ---------------- round trips, one per payload kind ----------------

test("round trip telemetry", () => {
    assert.equal(roundTrip("telemetry", TELEMETRY).length, codec.TLM_LEN);
});

test("round trip heartbeat", () => {
    assert.equal(roundTrip("heartbeat", HEARTBEAT).length, codec.HB_LEN);
});

EVENT_CASES.forEach(([name, args, text], i) => {
    test(`event ${name}-${i}: round trip and detail text`, () => {
        roundTrip("event", { t: 77, code: codec.EVC[name], args });
        assert.deepEqual(codec.decode("is02/event", eventFrame(name, args)), { t: 1000, event: name, detail: text, args });
    });
});

const ACK_CASES = [
    ["ESTOP", { pwm_off_us: 812 }, 7],
    ["RESUME", {}, 5],
    ["START", {}, 5],
    ["STOP", {}, 5],
    ["BARCODE", { letter: 31 }, 6],
    ["MAP", { map: MAP_ACTS, n: 4 }, 15],
    ["MAP_CLEAR", { map: NO_ACTS, n: 0 }, 15],
    ["GET_MAP", { map: MAP_ACTS, n: 4 }, 15],
    ["PING", { ping: 65535 }, 7],
    ["SET", { param: 63 }, 6],
    ["CONFIG", { applied: 54, n_rej: 1, rej: [17] }, codec.cfgAckLen(1)],
    ["CONFIG", { applied: 0, n_rej: 55, rej: Array.from({ length: 55 }, (_, i) => i) }, 48],
];

ACK_CASES.forEach(([name, args, length], i) => {
    test(`round trip ack ${name}-${i}`, () => {
        assert.equal(roundTrip("ack", { t: 5, cmd: codec.WCMD[name], reason: 0, args }).length, length);
    });
});

test("round trip ack unparsed and GET:PARAMS", () => {
    assert.equal(roundTrip("ack", { t: 5, cmd: codec.WCMD_UNPARSED, reason: 12, args: {} }).length, 5);
    const bits = codec.f32ToBits(0.35);
    const args = { part: 1, count: 4, values: [[0, 3], [1, 1], [10, 2], [23, bits]] };
    const payload = roundTrip("ack", { t: 5, cmd: codec.WCMD.GET_PARAMS, reason: 0, args });
    assert.equal(payload.length, codec.frameBytes(37 + 7 + (6 + 2) + (6 + 1) + (6 + 2) + (6 + 32)));
});

test("the longest GET:PARAMS part is 63 bytes", () => {
    const values = [5, 6, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20].map((i) => [i, 0]);
    const args = { part: 1, count: 12, values };
    const payload = roundTrip("ack", { t: 0, cmd: codec.WCMD.GET_PARAMS, reason: 0, args });
    assert.equal(payload.length, codec.GETP_MAX_LEN);
});

test("round trip commands", () => {
    const cases = [
        [{ code: 0 }, 1], [{ code: 1 }, 1], [{ code: 2 }, 1], [{ code: 3 }, 1], [{ code: 6 }, 1], [{ code: 7 }, 1],
        [{ code: 10 }, 1], [{ code: 4, letter: 25 }, 2], [{ code: 8, ping: 65535 }, 3],
        [{ code: 9, param: 54, value: 0x7FC00000 }, 6],
        [{ ...codec.mapBody({ fwd: 5, uturn: 20 }), code: 5 }, 4],
    ];
    for (const [record, length] of cases) {
        assert.equal(roundTrip("command", record).length, length);
    }
});

test("round trip map and config", () => {
    assert.equal(roundTrip("map", codec.mapBody({ fwd: 5, left: 10, right: 17, uturn: 20 })).length, codec.MAP_LEN);
    assert.deepEqual(roundTrip("map", codec.mapBody({})), bytes(0x10, 0, 0, 0));
    assert.equal(roundTrip("config", { count: 0, pairs: [] }).length, 2);
    const pairs = Array.from({ length: 55 }, (_, i) => [i, 0x3F800000 + i]);
    assert.equal(roundTrip("config", { count: 55, pairs }).length, 263);
});

test("bad version or length throws", () => {
    const cases = [
        ["telemetry", Uint8Array.from([0x20, ...new Array(53).fill(0)])],       // version 2
        ["telemetry", Uint8Array.from([0x10, ...new Array(52).fill(0)])],       // 53 bytes
        ["heartbeat", Uint8Array.from([0x10, ...new Array(25).fill(0)])],       // 26 bytes
        ["event", bytes(0x10)],                                                 // no header
        ["event", Uint8Array.from([...codec.encodeFields("event", { t: 0, code: 0, args: { state: 1 } }), 0])],
        ["ack", bytes(0x10, 0, 0, 0)],
        ["command", bytes()],
        ["command", new TextEncoder().encode("ESTOP")],                         // baseline text: version 4
        ["command", bytes(0x1B)],                                               // unassigned code 11
        ["command", bytes(0x18, 0)],                                            // PING one byte short
        ["map", bytes(0x10, 0, 0)],
        ["config", bytes(0x10, 0, 0)],                                          // 0 pairs, 3 bytes
        ["config", bytes(0x1F, 0xC0)],                                          // count 63
    ];
    for (const [kind, payload] of cases) {
        throwsCodec(() => codec.decodeFields(kind, payload));
    }
    throwsCodec(() => codec.decodeFields("nope", bytes(0x10)), /unknown payload kind 'nope'/);
});

test("FINE_SCAN count must be 1 to 11", () => {
    throwsCodec(() => eventFrame("FINE_SCAN", { first: 0, count: 0, cm: [] }));
    throwsCodec(() => eventFrame("FINE_SCAN", { first: 0, count: 12, cm: new Array(12).fill(0) }));
});

test("a field too wide or missing throws", () => {
    throwsCodec(() => codec.encodeFields("telemetry", { ...TELEMETRY, seq: 32768 }), /seq: 32768 does not fit 15 bits/);
    throwsCodec(() => codec.encodeFields("telemetry", { ...TELEMETRY, spd_l: 20000 }),
        /spd_l: 20000 does not fit a signed 15 bits/);
    const missing = { ...TELEMETRY };
    delete missing.seq;
    throwsCodec(() => codec.encodeFields("telemetry", missing), /telemetry: record has no field 'seq'/);
});

// ---------------- hand-built frames: the bit order itself ----------------

test("command bytes by hand", () => {
    assert.deepEqual(codec.encodeCommandText("ESTOP"), bytes(0x10));
    assert.deepEqual(codec.encodeCommandText("RESUME"), bytes(0x11));
    assert.deepEqual(codec.encodeCommandText("START"), bytes(0x12));
    assert.deepEqual(codec.encodeCommandText("STOP"), bytes(0x13));
    assert.deepEqual(codec.encodeCommandText("MAP:CLEAR"), bytes(0x16));
    assert.deepEqual(codec.encodeCommandText("GET:MAP"), bytes(0x17));
    assert.deepEqual(codec.encodeCommandText("GET:PARAMS"), bytes(0x1A));
    assert.deepEqual(codec.encodeCommandText("PING:7"), bytes(0x18, 0x00, 0x07));
    assert.deepEqual(codec.encodeCommandText("PING"), bytes(0x18, 0x00, 0x00));
    assert.deepEqual(codec.encodeCommandText("BARCODE:K"), bytes(0x14, 0b01010_000));
});

test("SET bytes by hand", () => {
    const view = new DataView(new ArrayBuffer(4));
    view.setFloat32(0, 0.35, false);
    const value = BigInt(view.getUint32(0, false));
    const bits = (1n << 44n) | (9n << 40n) | (23n << 34n) | (value << 2n);
    assert.deepEqual(codec.encodeCommandText("SET:LINE_STEER=0.35"), bigBytes(bits, 6));
});

test("map bytes by hand", () => {
    let body = 0n;
    for (const letter of [5n, 10n, 17n, 20n]) {                         // F, K, R, U
        body = (body << 6n) | 0b100000n | letter;
    }
    assert.deepEqual(codec.encodeMap(MAP_TEXT), bigBytes(((1n << 24n) | body) << 4n, 4));
    assert.deepEqual(codec.encodeCommandText("MAP:" + MAP_TEXT), Uint8Array.from([0x15, ...bigBytes(body, 3)]));
});

test("heartbeat bits by hand", () => {
    const value = toBig(codec.encodeFields("heartbeat", HEARTBEAT));
    assert.equal(value & 0b11111n, 0n);                                 // 5 padding bits
    assert.equal((value >> 5n) & 0xFFFFFFFFn, 0xC0A80114n);             // broker
    assert.equal((value >> 37n) & 0xFFFFn, 0xBEEFn);                    // CRC
});

test("event header by hand", () => {
    const payload = eventFrame("OBSTACLE", { cm: 198, us: 412 }, 0x123456);
    const value = toBig(payload);
    const bits = BigInt(payload.length * 8);
    assert.equal(value >> (bits - 28n), 0x1123456n);
    assert.equal((value >> (bits - 34n)) & 0x3Fn, 44n);
});

// ---------------- presentation ----------------

test("decode telemetry", () => {
    const m = codec.decode("is02/telemetry", codec.encodeFields("telemetry", TELEMETRY));
    assert.deepEqual(m, {
        seq: 32767, t: 16777215, state: "LINE_FOLLOWING",
        spd: [-123.4, 100], duty: [-100, 45.5], enc: [-8388607, 8388607], dist: 100000,
        line: { l: -1, r: 1, ir: 4095, irmn: 12, irmx: 3000, thr: [2000, 900] },
        bc: "K",
        hump: { pitch: -90, h: -1500, peak: 1500, max: 123.4 },
        obst: { cm: -1, scan: [-1, 431, 0, 12.3, -1], side: "R" },
        imu: { ev: "SUDDEN_IMPACT", tr: -720 },
        faults: 0xA005, vbat: 6600, rssi: -128, buf: 64,
    });
});

test("decode telemetry sentinels", () => {
    const rec = { ...TELEMETRY, thr_valid: 0, thr_b: 0, thr_w: 0, bc: 63, side: 0, rssi: 0 };
    const m = codec.decode("is02/telemetry", codec.encodeFields("telemetry", rec));
    assert.deepEqual(m.line.thr, [0, 0]);
    assert.equal(m.bc, "");
    assert.equal(m.obst.side, "");
    assert.ok(Object.is(m.rssi, 0));                                    // not -0
});

test("decode status", () => {
    assert.deepEqual(codec.decode("is02/status", codec.encodeFields("heartbeat", HEARTBEAT)), {
        seq: 16383, t: 123456, state: "IDLE", rssi: -57, buf: 3, up: 131071,
        reconnects: 65535, dropped: 2, faults: 0x6000, broker: "192.168.1.20", sat: 7,
        ptab: { count: 55, crc: 0xBEEF },
    });
});

test("every event has a text", () => {
    const named = new Set([...codec.FIXED_DETAIL.keys(), ...codec.EVENT_TEXT.keys()]);
    assert.deepEqual(new Set(codec.EVENTS.map((e) => e[0])), named);
    assert.deepEqual(new Set(EVENT_CASES.map((c) => c[0])), named);
});

const ACK_DECODE_CASES = [
    ["ESTOP", { pwm_off_us: 812 }, "ESTOP", 7, { pwm_off_us: 812 }],
    ["RESUME", {}, "RESUME", 8, {}],
    ["START", {}, "START", 2, {}],
    ["STOP", {}, "STOP", 3, {}],
    ["BARCODE", { letter: 10 }, "BARCODE:K", 1, {}],
    ["MAP", { map: MAP_ACTS, n: 4 }, "MAP", 0, { map: MAP_TEXT, n: 4 }],
    ["MAP_CLEAR", { map: NO_ACTS, n: 0 }, "MAP:CLEAR", 5, { map: "", n: 0 }],
    ["GET_MAP", { map: MAP_ACTS, n: 4 }, "GET:MAP", 0, { map: MAP_TEXT, n: 4 }],
    ["PING", { ping: 7 }, "PING:7", 4, {}],
    ["SET", { param: 23 }, "SET:LINE_STEER", 0, {}],
];

ACK_DECODE_CASES.forEach(([name, args, cmd, result, extras]) => {
    test(`decode ack ${name}`, () => {
        const payload = codec.encodeFields("ack", { t: 9, cmd: codec.WCMD[name], reason: result, args });
        assert.deepEqual(codec.decode("is02/ack", payload), { t: 9, cmd, result: codec.REASONS[result][1], ...extras });
    });
});

test("decode ack GET:PARAMS", () => {
    const values = [[0, 3], [1, 1], [10, 0], [23, codec.f32ToBits(0.35)], [39, 5], [5, codec.f32ToBits(-1.0)]];
    const args = { part: 2, count: 6, values };
    const m = codec.decode("is02/ack", codec.encodeFields("ack", { t: 1, cmd: 10, reason: 0, args }));
    assert.deepEqual(m, {
        t: 1, cmd: "GET:PARAMS", result: "ok", part: 2,
        params: {
            MOTOR_MAPPING_CONFIRMED: -1, ENC_R_REVERSED: 1, IMU_FORWARD_AXIS: -1,
            LINE_STEER: 0.35, BARCODE_LEVELS: 6, SERVO_MIN_US: -1,
        },
    });
});

test("decode ack CONFIG and unparsed", () => {
    const ok = { t: 1, cmd: 11, reason: 0, args: { applied: 55, n_rej: 0, rej: [] } };
    assert.deepEqual(codec.decode("is02/ack", codec.encodeFields("ack", ok)),
        { t: 1, cmd: "CONFIG", result: "ok", applied: 55, rejected: 0 });
    const bad = { t: 1, cmd: 11, reason: 26, args: { applied: 53, n_rej: 2, rej: [23, 0] } };
    assert.deepEqual(codec.decode("is02/ack", codec.encodeFields("ack", bad)),
        { t: 1, cmd: "CONFIG", result: "LINE_STEER MOTOR_MAPPING_CONFIRMED", applied: 53, rejected: 2 });
    const unparsed = codec.encodeFields("ack", { t: 1, cmd: 15, reason: 16, args: {} });
    assert.deepEqual(codec.decode("is02/ack", unparsed), { t: 1, cmd: "", result: "nack: unknown format version" });
});

test("decode the laptop-to-robot topics", () => {
    assert.deepEqual(codec.decode("is02/cmd", codec.encodeCommandText("PING:7")), { cmd: "PING:7" });
    assert.deepEqual(codec.decode("is02/map", codec.encodeMap(MAP_TEXT)), { map: MAP_TEXT, n: 4 });
    assert.deepEqual(codec.decode("is02/config", codec.encodeConfig(["LINE_STEER=0.5"])),
        { count: 1, params: { LINE_STEER: 0.5 } });
    assert.deepEqual(codec.decode("is02/cmd", codec.encodeCommandText("SET:LINE_STEER=0.5")),
        { cmd: "SET:LINE_STEER", value: 0.5 });
    assert.deepEqual(codec.decode("is02/cmd", codec.encodeCommandText("MAP:" + MAP_TEXT)), { cmd: "MAP", map: MAP_TEXT });
});

// ---------------- bad payloads ----------------

test("a bad version is logged and dropped", () => {
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/telemetry", Uint8Array.from([0x20, ...new Array(53).fill(0)]), { log }), null);
    assert.equal(lines.length, 1);
    assert.match(lines[0], /^is02\/telemetry: dropped, version 2, length 54: /);
});

test("a bad length is logged and dropped", () => {
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/status", bytes(0x10, 0, 0, 0), { log }), null);
    assert.match(lines[0], /^is02\/status: dropped, version 1, length 4: /);
});

test("an empty payload on a normal topic is logged; on a retained topic it is silent", () => {
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/event", bytes(), { log }), null);
    assert.match(lines[0], /^is02\/event: dropped, version none, length 0: /);
    const [quiet, log2] = logged();
    assert.equal(codec.decode("is02/map", bytes(), { log: log2 }), null);
    assert.equal(codec.decode("is02/config", bytes(), { log: log2 }), null);
    assert.deepEqual(quiet, []);
});

test("an unknown topic is logged", () => {
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/other", bytes(1, 2), { log }), null);
    assert.equal(lines[0], "is02/other: no decoder for this topic, length 2");
});

test("an unknown Event_Code shows the number and keeps the bytes", () => {
    const [lines, log] = logged();
    const payload = bigBytes((((1n << 24n | 500n) << 6n | 45n) << 6n), 5);
    assert.deepEqual(codec.decode("is02/event", payload, { log }), { t: 500, event: 45, detail: "", args: {} });
    assert.equal(lines[0], "unknown Event_Code 45");
    assert.deepEqual(codec.encodeFields("event", codec.decodeFields("event", payload)), payload);
});

test("an unknown Reason_Code shows the number", () => {
    const [lines, log] = logged();
    const payload = codec.encodeFields("ack", { t: 1, cmd: 2, reason: 28, args: {} });
    assert.deepEqual(codec.decode("is02/ack", payload, { log }), { t: 1, cmd: "START", result: 28 });
    assert.equal(lines[0], "unknown Reason_Code 28");
});

test("an unknown command code shows the number and keeps the bytes", () => {
    const [lines, log] = logged();
    const good = codec.encodeFields("ack", { t: 1, cmd: 2, reason: 12, args: {} });
    const payload = Uint8Array.from([good[0], good[1], good[2], (good[3] & 0xF0) | 13, good[4]]);
    assert.deepEqual(codec.decode("is02/ack", payload, { log }), { t: 1, cmd: 13, result: "nack: unknown command" });
    assert.equal(lines[0], "unknown command code 13");
    assert.deepEqual(codec.encodeFields("ack", codec.decodeFields("ack", payload)), payload);
});

[
    ["EMERGENCY_STOP", { reason: 12 }, "12"],
    ["AVOIDANCE_ABORT", { reason: 11 }, "11"],
    ["NAV_ACTION", { letter: 0, action: 6 }, "A -> 6"],
    ["BARCODE", { code: 50, rev: 0 }, "50 (read forwards)"],
    ["MAP", { map: [7, ...new Array(25).fill(0)] }, "A=7"],
].forEach(([name, args, text]) => {
    test(`an unknown code inside a ${name} event shows the number`, () => {
        const [lines, log] = logged();
        assert.equal(codec.decode("is02/event", eventFrame(name, args), { log }).detail, text);
        assert.ok(lines.length > 0);
    });
});

test("the default log is console.warn", (t) => {
    const calls = [];
    t.mock.method(console, "warn", (line) => calls.push(line));
    assert.equal(codec.decode("is02/event", bytes()), null);
    assert.equal(calls.length, 1);
    assert.match(calls[0], /is02\/event/);
});

// ---------------- legacy JSON ----------------

test("legacy JSON passes through only when asked", () => {
    const baseline = { seq: 3, t: 100, state: "IDLE", spd: [0, 0] };
    const text = new TextEncoder().encode(JSON.stringify(baseline));
    assert.deepEqual(codec.decode("is02/telemetry", text, { legacyJson: true }), baseline);
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/telemetry", text, { log }), null);
    assert.match(lines[0], /version 7/);                                // '{' = 0x7B
});

test("legacy mode still decodes binary; invalid JSON is dropped", () => {
    const m = codec.decode("is02/status", codec.encodeFields("heartbeat", HEARTBEAT), { legacyJson: true });
    assert.equal(m.broker, "192.168.1.20");
    const [lines, log] = logged();
    assert.equal(codec.decode("is02/telemetry", new TextEncoder().encode("{not json"), { legacyJson: true, log }), null);
    assert.ok(lines.length > 0);
});

// ---------------- user input ----------------

test("every baseline command text", () => {
    const cases = [
        ["ESTOP", 0], ["RESUME", 1], ["START", 2], ["STOP", 3], ["BARCODE:A", 4],
        ["MAP:F=FWD,K=LEFT,R=RIGHT,U=UTURN", 5], ["MAP:CLEAR", 6], ["GET:MAP", 7], ["PING:7", 8],
        ["SET:LINE_STEER=0.35", 9], ["GET:PARAMS", 10], ["  STOP\n", 3],
    ];
    for (const [text, code] of cases) {
        assert.equal(codec.decodeFields("command", codec.encodeCommandText(text)).code, code, text);
    }
});

test("SET keeps the value", () => {
    const rec = codec.decodeFields("command", codec.encodeCommandText("SET:BARCODE_LEVELS=6"));
    assert.equal(rec.param, 39);
    assert.equal(codec.f32FromBits(rec.value), 6);
});

test("bad command text names the argument", () => {
    const cases = [
        ["FLY", ["argument FLY: unknown command"]],
        ["PING:Dabc", ["argument PING:Dabc: PING takes a number 0..65535"]],
        ["PING:65536", ["PING:65536"]],
        ["BARCODE:k", ["argument BARCODE:k: use BARCODE:<A-Z>"]],
        ["BARCODE:7", ["BARCODE:7"]],
        ["SET:NOPE=1", ["argument SET:NOPE=1: NOPE: unknown parameter"]],
        ["SET:LINE_STEER", ["LINE_STEER: expected NAME=VALUE"]],
        ["SET:LINE_STEER=abc", ["LINE_STEER: bad number 'abc'"]],
        ["SET:LINE_STEER=nan", ["LINE_STEER"]],
        ["SET:LINE_STEER=inf", ["LINE_STEER"]],
        ["SET:LINE_STEER=1e-1", ["LINE_STEER"]],
        ["SET:LINE_STEER=1.5", ["LINE_STEER: 1.5 out of range 0.0..1.0"]],
        ["SET:LINE_BASE_MM_S=200", ["LINE_BASE_MM_S: 200 out of range 30.0..150.0"]],
        ["SET:BARCODE_DET_HYS=0.5", ["out of range 0.005..0.3"]],
        ["SET:BARCODE_LEVELS=2.5", ["BARCODE_LEVELS: 2.5 is not a whole number"]],
        ["SET:SERVO_MAX_US=1e40", ["SERVO_MAX_US"]],
        ["SET:SERVO_MAX_US=" + "9".repeat(60), ["SERVO_MAX_US: " + "9".repeat(60) + " out of range 1500.0..2800.0"]],
        ["MAP:", ["map: empty map"]],
    ];
    for (const [text, words] of cases) {
        throwsCodec(() => codec.encodeCommandText(text));
        try {
            codec.encodeCommandText(text);
        } catch (e) {
            for (const word of words) {
                assert.ok(e.message.includes(word), `${text}: ${e.message} lacks ${word}`);
            }
        }
    }
});

test("float32 limits are inclusive", () => {
    assert.ok(codec.encodeCommandText("SET:SPEED_KP=0.1"));
    assert.ok(codec.encodeCommandText("SET:SPEED_FF_MM_S_PER_PCT=0.1"));
    assert.ok(codec.encodeCommandText("SET:BARCODE_DET_HYS=0.005"));
});

test("encodeMap accepts every form", () => {
    const want = codec.encodeMap(MAP_TEXT);
    const forms = [
        "F=FWD;K=LEFT R=RIGHT\nU=UTURN",
        "MAP:" + MAP_TEXT,
        ["F=FWD", "K=LEFT", "R=RIGHT", "U=UTURN"],
        [["F", "FWD"], ["K", "LEFT"], ["R", "RIGHT"], ["U", "UTURN"]],
        "f=forward,k=l,r=right,u=u-turn",
        "F=STRAIGHT,K=LEFT,R=R,U=BACK",
        "F,U,K,R",                                                      // positional: FWD,UTURN,LEFT,RIGHT
    ];
    for (const form of forms) {
        assert.deepEqual(codec.encodeMap(form), want, String(form));
    }
});

test("encodeMap partial and clear", () => {
    const rec = codec.decodeFields("map", codec.encodeMap("K=LEFT"));
    assert.deepEqual([rec.left_on, rec.left, rec.fwd_on, rec.fwd], [1, 10, 0, 0]);
    assert.deepEqual(codec.encodeMap("CLEAR"), bytes(0x10, 0, 0, 0));
});

test("encodeMap errors", () => {
    const cases = [
        ["", "empty map"], [[], "empty map"], ["7=FWD", "letters must be A-Z"], ["K=JUMP", "unknown movement"],
        ["K=LEFT,X", "do not mix"], ["K,L,M", "exactly 4 letters"], ["K,L,M,N,O", "exactly 4 letters"],
        ["KLM", "L=ACTION"], ["K,K,M,N", "given twice"], ["K=LEFT,K=RIGHT", "given twice"],
        ["K=LEFT,Q=LEFT", "one letter per movement"],
    ];
    for (const [pairs, word] of cases) {
        throwsCodec(() => codec.encodeMap(pairs), new RegExp(word));
    }
    throwsCodec(() => codec.encodeMap("K=LEFT,Q=LEFT"), /map: Q: LEFT already has letter K: /);
});

test("encodeConfig lines, text and separators", () => {
    const payload = codec.encodeConfig(["# comment", "", "LINE_STEER=0.35", "  SPEED_KP = 0.02  # tuned", "SET:TURN_SNAP=1"]);
    const rec = codec.decodeFields("config", payload);
    assert.deepEqual(rec.pairs.map((p) => p[0]), [23, 20, 33]);
    assert.equal(codec.f32FromBits(rec.pairs[2][1]), 1);
    assert.equal(payload.length, codec.cfgLen(3));
    assert.deepEqual(codec.encodeConfig("LINE_STEER=0.35\nSPEED_KP=0.02\n"), codec.encodeConfig(["LINE_STEER=0.35;SPEED_KP=0.02"]));
    assert.deepEqual(codec.encodeConfig(["# nothing"]), bytes(0x10, 0x00));
});

test("encodeConfig reports every bad line", () => {
    const lines = ["LINE_STEER=0.35", "", "NOPE=1", "SPEED_KP=abc", "TURN_SNAP", "LINE_STEER=9"];
    assert.throws(() => codec.encodeConfig(lines), (e) => {
        assert.ok(e instanceof codec.CodecError);
        assert.deepEqual(e.errors.map(([line, name]) => [line, name]),
            [[3, "NOPE"], [4, "SPEED_KP"], [5, "TURN_SNAP"], [6, "LINE_STEER"]]);
        assert.deepEqual(e.errors[0], [3, "NOPE", "unknown parameter"]);
        assert.ok(e.message.includes("line 3: NOPE") && e.message.includes("line 6: LINE_STEER"));
        return true;
    });
    throwsCodec(() => codec.encodeConfig(new Array(56).fill("LINE_STEER=0.5")), /at most 55/);
});

// ---------------- jbFixed ----------------

test("jbFixed", () => {
    const cases = [
        [0.0, 1, "0.0"],
        [12.34, 1, "12.3"],
        [12.35, 1, "12.4"],                     // float32 12.35 is 12.3500004
        [0.25, 1, "0.3"],                       // exact half: away from zero
        [-0.25, 1, "-0.3"],
        [-0.04, 1, "0.0"],                      // no "-0.0"
        [2.5, 0, "3"],
        [-1.0, 0, "-1"],
        [0.35, 4, "0.3500"],
        [0.0001, 4, "0.0001"],
        [5.0e9, 1, "400000000.0"],              // capped inside uint32
    ];
    for (const [value, decimals, text] of cases) {
        assert.equal(codec.jbFixed(value, decimals), text, `${value}/${decimals}`);
    }
    assert.equal(codec.jbFixed(NaN, 1), "null");
});

test("require() and the browser global both give the codec", () => {
    assert.equal(typeof codec.decode, "function");
    const vm = require("node:vm");
    const sandbox = { TextEncoder, TextDecoder, console };
    sandbox.self = sandbox;
    vm.createContext(sandbox);
    vm.runInContext(require("node:fs").readFileSync(path.join(__dirname, "..", "..", "..", "MQTT", "is02_codec.js"), "utf8"), sandbox);
    assert.equal(typeof sandbox.IS02.decode, "function");
});

test("readPayload tells JSON, text and binary apart in both modes", () => {
    const text = (s) => new TextEncoder().encode(s);
    assert.deepEqual(codec.readPayload("is02/ack", text('{"cmd": "PING:1"}')),
        { decoded: { cmd: "PING:1" }, text: '{"cmd": "PING:1"}' });
    assert.deepEqual(codec.readPayload("is02/cmd", text("PING:1")), { decoded: null, text: "PING:1" });
    assert.deepEqual(codec.readPayload("is02/map", text("X=FWD,K=LEFT")), { decoded: null, text: "X=FWD,K=LEFT" });
    assert.deepEqual(codec.readPayload("is02/config", new Uint8Array(0)), { decoded: null, text: "" });
    const map = codec.readPayload("is02/map", codec.encodeMap("X=FWD,K=LEFT,Q=RIGHT,M=UTURN"));
    assert.deepEqual(map.decoded, { map: "K=LEFT,M=UTURN,Q=RIGHT,X=FWD", n: 4 });
    assert.deepEqual(JSON.parse(map.text), map.decoded);
    const lines = [];
    const log = (line) => lines.push(line);
    assert.deepEqual(codec.readPayload("is02/map", new Uint8Array([0x10]), { log }), { decoded: null, text: null });
    assert.deepEqual(codec.readPayload("is02/telemetry", text("hello"), { log }), { decoded: null, text: null });
    assert.deepEqual(codec.readPayload("is02/telemetry", text("{bad"), { log }), { decoded: null, text: "{bad" });
    assert.equal(lines.length, 3);
    assert.ok(lines[1].includes("version 6"));
});

test("ptabMismatch names both tables, and a JSON heartbeat says nothing", () => {
    assert.equal(codec.ptabMismatch({}), null);
    assert.equal(codec.ptabMismatch({ ptab: { count: codec.PARAM_COUNT, crc: codec.PARAM_CRC } }), null);
    const hex = codec.PARAM_CRC.toString(16).toUpperCase().padStart(4, "0");
    assert.equal(codec.ptabMismatch({ ptab: { count: 3, crc: 0xBEEF } }),
        `parameter table mismatch: robot 3 / 0xBEEF, tool ${codec.PARAM_COUNT} / 0x${hex}`);
});
