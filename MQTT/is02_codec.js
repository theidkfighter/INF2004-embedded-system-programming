/*
 * IS-02 Laptop_Codec (JavaScript): the binary MQTT payloads of the robot.
 *
 * A port of MQTT/is02_codec.py: the same bytes and the same decoded objects.
 * Two layers, the same as the Python codec:
 *
 *     IS02.decodeFields(kind, payload) -> record     raw field codes; encodeFields
 *     IS02.encodeFields(kind, record)  -> Uint8Array gives the input bytes back
 *
 *     IS02.decode(topic, payload, {legacyJson, log}) -> object   the Baseline_Build JSON shape
 *     IS02.encodeCommandText(text)   -> Uint8Array   "ESTOP", "PING:7", "SET:LINE_STEER=0.35", ...
 *     IS02.encodeMap(pairs)          -> Uint8Array   the Map_Frame
 *     IS02.encodeConfig(lines)       -> Uint8Array   the Config_Frame
 *
 *     IS02.readPayload(topic, payload, {log}) -> {decoded, text}   for the tools: JSON, text or binary
 *     IS02.ptabMismatch(status)      -> warning or null    from a decoded heartbeat
 *
 * The kinds are telemetry, heartbeat, event, ack, command, map and config.
 * Record and object field names are the Python ones (spd_l, n_rej, ...).
 *
 * The code tables and frame lengths are copies of app_program/wire_ids.h, and
 * PARAMS is PARAMS[] of app_program/app_params.c in the same order: change them
 * together with is02_codec.py. No build step and no packages: the dashboard
 * loads this file with <script src="is02_codec.js"> from file://, and Node
 * loads it with require().
 */
(function (root, factory) {
    "use strict";
    if (typeof module === "object" && module.exports) {
        module.exports = factory();
    } else {
        root.IS02 = factory();
    }
}(typeof self !== "undefined" ? self : this, function () {
    "use strict";

    var FORMAT_VERSION = 1;                 // WIRE_FORMAT_VERSION

    class CodecError extends Error {
        /* errors lists [line number, NAME, reason] for each bad encodeConfig line. */
        constructor(message, errors) {
            super(message);
            this.name = "CodecError";
            this.errors = errors || [];
        }
    }

    /* A record without a field the layout needs (a Python KeyError). */
    class MissingField extends Error {
        constructor(name) {
            super(name);
            this.field = name;
        }
    }

    // ---------------- Python-style text, so the messages match ----------------

    function pyFloat(x) {
        return Number.isInteger(x) ? x.toFixed(1) : String(x);
    }

    function pyRepr(x) {
        if (x === null || x === undefined) {
            return "None";
        }
        if (typeof x === "boolean") {
            return x ? "True" : "False";
        }
        if (typeof x === "string") {
            var quote = x.indexOf("'") >= 0 && x.indexOf('"') < 0 ? '"' : "'";
            var body = x.replace(/\\/g, "\\\\").replace(/\n/g, "\\n").replace(/\r/g, "\\r").replace(/\t/g, "\\t");
            if (quote === "'") {
                body = body.replace(/'/g, "\\'");
            }
            body = body.replace(/[\x00-\x1f\x7f]/g, function (c) {
                return "\\x" + c.charCodeAt(0).toString(16).padStart(2, "0");
            });
            return quote + body + quote;
        }
        if (typeof x === "number") {
            if (Number.isNaN(x)) {
                return "nan";
            }
            return Number.isFinite(x) ? String(x) : (x < 0 ? "-inf" : "inf");
        }
        if (Array.isArray(x)) {
            return "[" + x.map(pyRepr).join(", ") + "]";
        }
        return String(x);
    }

    // ---------------- code tables (wire_ids.h) ----------------

    // [NAME, largest argument bits, variable length]
    var EVENTS = [
        ["STATE", 3, false], ["IMU_OK", 9, false], ["FAULT_IMU_I2C", 0, false], ["FAULT_IMU", 0, false],
        ["HUMP_START", 29, false], ["HUMP_PEAK", 29, false], ["HUMP_END", 29, false], ["MAP", 78, false],
        ["BATTERY_LOW", 0, false], ["START_REFUSED", 0, false], ["MAP_WARNING", 78, false],
        ["MISSION_COMPLETE", 59, false], ["TURN_SNAP", 1, false], ["NAV_ACTION", 8, false], ["NAV", 10, false],
        ["EMERGENCY_STOP", 4, false], ["ERROR", 4, false], ["RECOVERED", 0, false], ["RECOVERY_REFUSED", 15, false],
        ["IMPACT", 0, false], ["BC_DBG", 203, true], ["BARCODE", 7, false], ["BARCODE_UNKNOWN", 7, false],
        ["BARCODE_FAULT", 0, false], ["BARCODE_FAILED", 0, false], ["LINE_FOUND", 0, false], ["LINE_LOST", 0, false],
        ["LINE_SEARCH", 1, false], ["FINE_SCAN", 110, true], ["SIDE_FOLLOW", 64, false], ["SIDE_LOOK", 27, false],
        ["BYPASS_EXTEND", 10, false], ["BYPASS_SIDESTEP", 10, false], ["BYPASS_RETURN", 0, false],
        ["OBSTACLE_PROFILE", 51, false], ["AVOIDANCE_ABORT", 4, false], ["OBST_REVERSE_CAP", 44, false],
        ["OBSTACLE_BACKOFF", 9, false], ["OBSTACLE_RESCAN", 1, false], ["OBSTACLE_TIE", 20, false],
        ["OBSTACLE_CLEARED", 0, false], ["BYPASS_COMPLETE", 2, false], ["BYPASS_BLOCKED", 19, false],
        ["BYPASS_RETRY", 2, false], ["OBSTACLE", 29, false]
    ];
    var EVC = {};
    EVENTS.forEach(function (e, code) { EVC[e[0]] = code; });

    // [NAME, text]; CONFIG_REJECTED has no fixed text, it lists the rejected NAMEs
    var REASONS = [
        ["OK", "ok"],
        ["QUEUED", "queued"],
        ["STARTING", "starting"],
        ["ENDING_RUN", "ending the run"],
        ["PONG", "pong"],
        ["CLEARED", "cleared"],
        ["OK_NOT_4", "ok (not 4 letters)"],
        ["MOTORS_OFF", "motors off (press BTN1 on the robot to recover)"],
        ["RESUME_REFUSED", "refused: clear the hazard and press BTN1 on the robot"],
        ["BAD_LETTER", "nack: use BARCODE:<A-Z>"],
        ["ONLY_LINE_FOLLOW", "nack: only while LINE_FOLLOWING"],
        ["ONLY_FROM_IDLE", "nack: only from IDLE"],
        ["UNKNOWN_COMMAND", "nack: unknown command"],
        ["UNKNOWN_PARAM", "nack: unknown parameter"],
        ["BAD_NUMBER", "nack: bad number"],
        ["OUT_OF_RANGE", "nack: out of range"],
        ["BAD_VERSION", "nack: unknown format version"],
        ["BAD_LENGTH", "nack: bad length"],
        ["MAP_LETTERS", "nack: letters must be A-Z"],
        ["MAP_MOVEMENT", "nack: unknown movement (use FWD, LEFT, RIGHT, UTURN)"],
        ["MAP_POS_COUNT", "nack: positional form takes exactly 4 letters"],
        ["MAP_SYNTAX", "nack: expected L=ACTION or a single letter"],
        ["MAP_MIXED", "nack: do not mix the two forms"],
        ["MAP_POS_4", "nack: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT"],
        ["MAP_EMPTY", "nack: empty map"],
        ["MAP_DUP_LETTER", "nack: the same letter was given twice"],
        ["CONFIG_REJECTED", ""],
        ["BAD_COUNT", "nack: count larger than the parameter table"]
    ];
    var RSN_CONFIG_REJECTED = 26;

    // [NAME, cmd echo, Command_Frame length, Ack_Frame length]; length 0 = not on
    // is02/cmd for a command, variable for an ack
    var COMMANDS = [
        ["ESTOP", "ESTOP", 1, 7], ["RESUME", "RESUME", 1, 5], ["START", "START", 1, 5], ["STOP", "STOP", 1, 5],
        ["BARCODE", "BARCODE", 2, 6], ["MAP", "MAP", 4, 15], ["MAP_CLEAR", "MAP:CLEAR", 1, 15],
        ["GET_MAP", "GET:MAP", 1, 15], ["PING", "PING", 3, 7], ["SET", "SET", 6, 6],
        ["GET_PARAMS", "GET:PARAMS", 1, 0], ["CONFIG", "CONFIG", 0, 0]
    ];
    var WCMD = {};
    COMMANDS.forEach(function (c, code) { WCMD[c[0]] = code; });
    var WCMD_UNPARSED = 15;

    var STOP_REASONS = [
        "", "remote E-stop (MQTT)", "BTN2 stop", "collision (IMU)", "obstacle closer than 8 cm",
        "watchdog: a control task stopped running", "calibration missing",
        "IMU fault (3 failed I2C recoveries)", "battery critical", "fault reported (see faults in telemetry)"
    ];

    var ABORT_REASONS = [
        "bypass leg failed (timeout, E-stop or proximity)", "realign failed",
        "reverse onto the original path failed", "reverse limit reached",
        "reverse failed (E-stop, proximity or timeout)", "profile failed (no echo in the fine scan)",
        "no room on either side", "no bypass plan", "backing out failed (E-stop, proximity or timeout)",
        "blocked on both sides", "too many attempts"
    ];

    var STATES = ["INITIALISING", "IDLE", "LINE_FOLLOWING", "MANEUVER", "OBSTACLE_AVOIDANCE",
        "MISSION_COMPLETE", "EMERGENCY_STOP", "ERROR"];
    var MOTION_EVENTS = ["STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING", "TURNING",
        "CLIMBING_HUMP", "DESCENDING_HUMP", "SUDDEN_IMPACT"];
    var ACTIONS = ["NONE", "FWD", "UTURN", "LEFT", "RIGHT"];    // nav_action_t
    var ACT_NONE = 0;
    var C39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    var C39_NONE = 63;
    var BC_LEVELS = ["50%", "35%", "65%", "25%", "75%", "adapt"];
    var IMU_CHIPS = [null, "LSM303DLHC", "LSM303D"];            // hal_imu_chip_t
    var SIDES = ["", "L", "R"];
    var BARCODE_MIN_ELEMENTS = 9;
    // config_check() order: bits 2..14 of RECOVERY_REFUSED
    var CONFIG_ITEMS = ["MOTOR_MAPPING_CONFIRMED", "ENC_DIRECTION_CONFIRMED", "ENC_A_EDGES_PER_REV",
        "WHEEL_DIAMETER_MM", "TRACK_WIDTH_MM", "SPEED_FF_DEADBAND_PCT", "SPEED_FF_MM_S_PER_PCT",
        "LINE_DO_BLACK_HIGH", "IR_WHITE_ADC", "IR_BLACK_ADC", "SERVO_CALIBRATED", "IMU_FORWARD_AXIS",
        "IMU_FORWARD_SIGN"];

    // ---------------- frame lengths (wire_ids.h) ----------------

    var TLM_BITS = 432, TLM_LEN = 54;
    var HB_BITS = 195, HB_LEN = 25;
    var MAP_LEN = 4;
    var EVT_HDR_BITS = 34;
    var EVT_MAX_LEN = 30;
    var ACK_HDR_BITS = 37;
    var ACK_UNPARSED_LEN = 5;
    var GETP_PER_PART = 12;
    var GETP_MAX_LEN = 63;
    var FINE_SCAN_MAX = 11;
    var BC_WIDTHS_MAX = 16;

    /* Length in bytes of a frame of this many bits (WIRE_BYTES). */
    function frameBytes(bits) {
        return Math.floor((bits + 7) / 8);
    }

    function evtLen(argBits) {
        return frameBytes(EVT_HDR_BITS + argBits);
    }

    function cfgLen(count) {
        return frameBytes(10 + 38 * count);
    }

    function cfgAckLen(rejected) {
        return frameBytes(ACK_HDR_BITS + 12 + 6 * rejected);
    }

    // ---------------- Parameter_IDs (app_params.c, design D15) ----------------

    // Parameter_Store representation -> code width in a GET:PARAMS part. F32 stays
    // a float; the others are packed in g_params.bits.
    var PARAM_KINDS = new Map([["F32", 32], ["F1", 1], ["S2", 2], ["B2-1", 2], ["B3+1", 3]]);

    // [NAME, representation, min, max], the index is the Parameter_ID
    var PARAMS = [
        ["MOTOR_MAPPING_CONFIRMED", "S2", -1.0, 1.0],
        ["ENC_R_REVERSED", "F1", 0.0, 1.0],
        ["ENC_L_REVERSED", "F1", 0.0, 1.0],
        ["ENC_DIRECTION_CONFIRMED", "S2", -1.0, 1.0],
        ["LINE_DO_BLACK_HIGH", "S2", -1.0, 1.0],
        ["SERVO_MIN_US", "F32", 300.0, 1500.0],
        ["SERVO_MAX_US", "F32", 1500.0, 2800.0],
        ["SERVO_CALIBRATED", "S2", -1.0, 1.0],
        ["SERVO_LOW_ANGLE_IS_RIGHT", "F1", 0.0, 1.0],
        ["SERVO_TRIM_DEG", "F32", -45.0, 45.0],
        ["IMU_FORWARD_AXIS", "B2-1", -1.0, 2.0],
        ["IMU_FORWARD_SIGN", "S2", -1.0, 1.0],
        ["ROBOT_HALF_WIDTH_MM", "F32", 40.0, 200.0],
        ["ENC_A_EDGES_PER_REV", "F32", 10.0, 20000.0],
        ["WHEEL_DIAMETER_MM", "F32", 20.0, 150.0],
        ["TRACK_WIDTH_MM", "F32", 50.0, 300.0],
        ["SPEED_FF_DEADBAND_PCT", "F32", 0.0, 90.0],
        ["SPEED_FF_MM_S_PER_PCT", "F32", 0.1, 100.0],
        ["IR_WHITE_ADC", "F32", 0.0, 4095.0],
        ["IR_BLACK_ADC", "F32", 0.0, 4095.0],
        ["SPEED_KP", "F32", 0.0, 0.1],
        ["SPEED_KI", "F32", 0.0, 1.0],
        ["LINE_BASE_MM_S", "F32", 30.0, 150.0],         // LINE_BASE_SAFE_MIN_MM_S..MAX
        ["LINE_STEER", "F32", 0.0, 1.0],
        ["LINE_STEER_RAMP", "F32", 0.0, 20.0],
        ["LINE_HOLD_STEER", "F32", 0.0, 1.0],
        ["LINE_HOLD_MM", "F32", 0.0, 300.0],
        ["LINE_LOST_MM", "F32", 50.0, 3000.0],
        ["LINE_CURVE_SLOW", "F32", 0.0, 0.8],
        ["LINE_MIN_WHEEL_MM_S", "F32", 0.0, 80.0],
        ["LINE_BIAS_KI", "F32", 0.0, 10.0],
        ["LINE_BIAS_DECAY_MM", "F32", 0.0, 3000.0],
        ["OBST_MAX_TURN_DPS", "F32", 0.0, 720.0],
        ["TURN_SNAP", "F1", 0.0, 1.0],
        ["BARCODE_AT_JUNCTION", "F1", 0.0, 1.0],
        ["BARCODE_HYS", "F32", 0.01, 0.30],
        ["BARCODE_DETECT", "F1", 0.0, 1.0],
        ["BARCODE_DET_HYS", "F32", 0.005, 0.30],
        ["BARCODE_SENSOR_Y_MM", "F32", -150.0, 150.0],
        ["BARCODE_LEVELS", "B3+1", 1.0, 6.0],
        ["BARCODE_ADAPT_SWING", "F32", 0.05, 0.60],
        ["BARCODE_DEBUG", "F1", 0.0, 1.0],
        ["TURN_MM_S", "F32", 30.0, 250.0],              // SPEED_MAX_MM_S
        ["BYPASS_MM_S", "F32", 30.0, 250.0],
        ["OBSTACLE_STOP_CM", "F32", 10.0, 60.0],
        ["OBST_MARGIN_MM", "F32", 0.0, 300.0],
        ["OBST_DEPTH_MM", "F32", 20.0, 600.0],
        ["OBST_LEG_STOP_CM", "F32", 0.0, 40.0],
        ["OBST_REVERSE_MAX_MM", "F32", 0.0, 1000.0],
        ["OBST_LEGACY", "F1", 0.0, 1.0],
        ["OBST_MIN_WIDTH_MM", "F32", 0.0, 600.0],
        ["OBST_TRUST_MISS", "F1", 0.0, 1.0],
        ["OBST_EDGE_PAD_MM", "F32", 0.0, 100.0],
        ["ROBOT_LEFT_HALF_MM", "F32", 0.0, 250.0],
        ["IR_TO_AXLE_MM", "F32", 0.0, 200.0]
    ];
    var PARAM_IDS = {};
    var PARAM_ID_MAP = new Map();
    PARAMS.forEach(function (p, i) {
        PARAM_IDS[p[0]] = i;
        PARAM_ID_MAP.set(p[0], i);
    });
    var PARAM_NOT_FOUND = 63;               // SET ack of a legacy text with an unknown name

    /* CRC-16/CCITT-FALSE: polynomial 0x1021, start 0xFFFF, no reflection, no final XOR.
       data: bytes, or an ASCII string. */
    function crc16CcittFalse(data) {
        var crc = 0xFFFF;
        for (var i = 0; i < data.length; i++) {
            var byte = typeof data === "string" ? data.charCodeAt(i) : data[i];
            crc ^= byte << 8;
            for (var bit = 0; bit < 8; bit++) {
                crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
            }
        }
        return crc;
    }

    /* [count, CRC] of the parameter table: the NAMEs in ID order, each followed by a newline. */
    function paramTableVersion(names) {
        names = names === undefined || names === null ? PARAMS.map(function (p) { return p[0]; }) : Array.from(names);
        return [names.length, crc16CcittFalse(names.map(function (n) { return n + "\n"; }).join(""))];
    }

    var PARAM_VERSION = paramTableVersion();
    var PARAM_COUNT = PARAM_VERSION[0];
    var PARAM_CRC = PARAM_VERSION[1];

    // ---------------- bitstream (design D7) ----------------

    /* Most significant bit first; signed fields are two's complement of their width. */
    class BitReader {
        constructor(data) {
            this.data = data;
            this.bits = data.length * 8;
            this.pos = 0;
        }

        get(width) {
            var end = this.pos + width;
            if (end > this.bits) {
                throw new CodecError("payload too short: needs more than " + Math.floor(this.bits / 8) + " bytes");
            }
            var value = 0;
            for (var p = this.pos; p < end; p++) {
                value = value * 2 + ((this.data[p >> 3] >> (7 - (p & 7))) & 1);
            }
            this.pos = end;
            return value;
        }

        getSigned(width) {
            var code = this.get(width);
            return code >= Math.pow(2, width - 1) ? code - Math.pow(2, width) : code;
        }
    }

    class BitWriter {
        constructor() {
            this.bits = [];
        }

        put(name, code, width) {
            if (typeof code !== "number" || !Number.isInteger(code) || code < 0 || code >= Math.pow(2, width)) {
                throw new CodecError(name + ": " + pyRepr(code) + " does not fit " + width + " bits");
            }
            for (var i = width - 1; i >= 0; i--) {
                this.bits.push(Math.floor(code / Math.pow(2, i)) % 2);
            }
        }

        putSigned(name, code, width) {
            var half = Math.pow(2, width - 1);
            if (typeof code !== "number" || !Number.isInteger(code) || code < -half || code >= half) {
                throw new CodecError(name + ": " + pyRepr(code) + " does not fit a signed " + width + " bits");
            }
            this.put(name, code < 0 ? code + Math.pow(2, width) : code, width);
        }

        toBytes() {
            var out = new Uint8Array(Math.ceil(this.bits.length / 8));
            for (var i = 0; i < this.bits.length; i++) {
                out[i >> 3] |= this.bits[i] << (7 - (i & 7));
            }
            return out;
        }
    }

    // ---------------- frame layouts (design D8 to D14) ----------------

    // A layout lists [name, width] in stream order. A negative width is a signed
    // field. A third item makes the field a list: of that many values, or of as
    // many as the earlier field it names.

    var TELEMETRY = [
        ["seq", 15], ["t", 24], ["state", 3], ["spd_l", -15], ["spd_r", -15], ["duty_l", -11], ["duty_r", -11],
        ["enc_l", -24], ["enc_r", -24], ["dist", 20], ["line_l", -2], ["line_r", -2], ["ir", 12], ["ir_min", 12],
        ["ir_max", 12], ["thr_valid", 1], ["thr_b", 12], ["thr_w", 12], ["bc", 6], ["pitch", -11], ["hump_h", -15],
        ["hump_peak", 14], ["hump_max", 14], ["us", 13], ["scan", 13, 5], ["side", 2], ["imu_ev", 3], ["turn", -14],
        ["faults", 16], ["vbat", 13], ["rssi", 8], ["buf", 7]
    ];

    var HEARTBEAT = [
        ["seq", 14], ["t", 24], ["state", 3], ["rssi", 8], ["buf", 7], ["up", 17], ["reconnects", 16],
        ["dropped", 16], ["sat", 16], ["faults", 16], ["ptab_count", 6], ["ptab_crc", 16], ["broker", 8, 4]
    ];

    var MAP78 = [["map", 3, 26]];
    var HUMP_ARGS = [["valid", 1], ["h", 14], ["max", 14]];
    var BARCODE_ARGS = [["code", 6], ["rev", 1]];

    // Arguments per Event_Code; BC_DBG goes through BC_DBG_ARGS
    var EVENT_ARGS = {
        STATE: [["state", 3]],
        IMU_OK: [["chip", 2], ["addr", 7]],
        HUMP_START: HUMP_ARGS,
        HUMP_PEAK: HUMP_ARGS,
        HUMP_END: HUMP_ARGS,
        MAP: MAP78,
        MAP_WARNING: MAP78,
        MISSION_COMPLETE: [["t", 17], ["dist", 14], ["bc", 6], ["humps", 4], ["max", 14], ["obst", 4]],
        TURN_SNAP: [["not_seen", 1]],
        NAV_ACTION: [["letter", 5], ["action", 3]],
        NAV: [["variant", 2], ["letter", 5], ["action", 3]],
        EMERGENCY_STOP: [["reason", 4]],
        ERROR: [["reason", 4]],
        RECOVERY_REFUSED: [["why", 15]],
        BARCODE: BARCODE_ARGS,
        BARCODE_UNKNOWN: BARCODE_ARGS,
        LINE_SEARCH: [["right", 1]],
        FINE_SCAN: [["first", -7], ["count", 4], ["cm", 9, "count"]],
        SIDE_FOLLOW: [["n60", 8], ["seen", 8], ["near", 13], ["limit", 13], ["leg", 11], ["x", 11]],
        SIDE_LOOK: [["d90", 13], ["d60", 13], ["gone", 1]],
        BYPASS_EXTEND: [["variant", 1], ["cm", 9]],
        BYPASS_SIDESTEP: [["mm", 10]],
        OBSTACLE_PROFILE: [["r", 9], ["w", 10], ["off", -10], ["clr_l", -10], ["clr_r", -10], ["side", 2]],
        AVOIDANCE_ABORT: [["reason", 4]],
        OBST_REVERSE_CAP: [["variant", 2], ["n", 10], ["used", 10], ["cap", 10], ["total", 12]],
        OBSTACLE_BACKOFF: [["need", 8], ["right", 1]],
        OBSTACLE_RESCAN: [["blocked", 1]],
        OBSTACLE_TIE: [["dl", 9], ["dr", 9], ["side", 2]],
        BYPASS_COMPLETE: [["right", 1], ["search", 1]],
        BYPASS_BLOCKED: [["right", 1], ["no_end", 1], ["cm", 9], ["deg", -8]],
        BYPASS_RETRY: [["right", 1], ["wider", 1]],
        OBSTACLE: [["cm", 13], ["us", 16]]
    };

    var BC_WINDOW = [["n", 7], ["res", 2], ["code", 6], ["rev", 1], ["thr_b", 12], ["thr_w", 12], ["ovr", 16]];
    var BC_LEVEL_ARGS = [];
    for (var level = 0; level < 6; level++) {
        BC_LEVEL_ARGS.push(["res" + level, 2], ["code" + level, 6], ["n" + level, 7]);
    }
    // variant U3 comes first, then:
    var BC_DBG_ARGS = new Map([
        [0, []],                                                            // off
        [1, BC_WINDOW],                                                     // window
        [2, BC_WINDOW.concat([["level", 3], ["conflict", 1]])],             // pass
        [3, [["n_levels", 3]].concat(BC_LEVEL_ARGS)],                       // levels
        [4, [["part", 3], ["count", 5], ["w", 12, "count"]]]                // widths
    ]);
    var RES_NONE = 0, RES_DECODED = 1;                                      // RES2; 2 = not decoded

    var MAP_ACK = MAP78.concat([["n", 5]]);
    // Arguments per command code, except GET:PARAMS
    var ACK_ARGS = new Map([
        [WCMD.ESTOP, [["pwm_off_us", 16]]],
        [WCMD.RESUME, []],
        [WCMD.START, []],
        [WCMD.STOP, []],
        [WCMD.BARCODE, [["letter", 5]]],
        [WCMD.MAP, MAP_ACK],
        [WCMD.MAP_CLEAR, MAP_ACK],
        [WCMD.GET_MAP, MAP_ACK],
        [WCMD.PING, [["ping", 16]]],
        [WCMD.SET, [["param", 6]]],
        [WCMD.CONFIG, [["applied", 6], ["n_rej", 6], ["rej", 6, "n_rej"]]],
        [WCMD_UNPARSED, []]
    ]);

    // forward, left, right, U-turn: present F1, letter U5
    var MAP_BODY = [["fwd_on", 1], ["fwd", 5], ["left_on", 1], ["left", 5],
        ["right_on", 1], ["right", 5], ["uturn_on", 1], ["uturn", 5]];
    var MAP_SLOTS = new Map([[1, "fwd"], [3, "left"], [4, "right"], [2, "uturn"]]);  // nav_action_t -> movement
    var MAP_ORDER = ["fwd", "left", "right", "uturn"];

    var COMMAND_ARGS = new Map([
        [WCMD.ESTOP, []],
        [WCMD.RESUME, []],
        [WCMD.START, []],
        [WCMD.STOP, []],
        [WCMD.BARCODE, [["letter", 5]]],
        [WCMD.MAP, MAP_BODY],
        [WCMD.MAP_CLEAR, []],
        [WCMD.GET_MAP, []],
        [WCMD.PING, [["ping", 16]]],
        [WCMD.SET, [["param", 6], ["value", 32]]],                          // value: F32 bit pattern
        [WCMD.GET_PARAMS, []]
    ]);

    /* The field a record lacks (a Python KeyError). */
    function need(rec, name) {
        if (rec === null || rec === undefined || rec[name] === undefined) {
            throw new MissingField(name);
        }
        return rec[name];
    }

    function readLayout(r, layout, rec) {
        layout.forEach(function (field) {
            var name = field[0], width = field[1];
            var get = width > 0 ? r.get.bind(r) : r.getSigned.bind(r);
            if (field.length === 2) {
                rec[name] = get(Math.abs(width));
            } else {
                var count = typeof field[2] === "string" ? rec[field[2]] : field[2];
                var values = [];
                for (var i = 0; i < count; i++) {
                    values.push(get(Math.abs(width)));
                }
                rec[name] = values;
            }
        });
        return rec;
    }

    function writeLayout(w, layout, rec) {
        layout.forEach(function (field) {
            var name = field[0], width = field[1];
            var put = (width > 0 ? w.put : w.putSigned).bind(w);
            if (field.length === 2) {
                put(name, need(rec, name), Math.abs(width));
                return;
            }
            var count = typeof field[2] === "string" ? need(rec, field[2]) : field[2];
            var values = need(rec, name);
            if (values.length !== count) {
                throw new CodecError(name + ": " + values.length + " values, expected " + count);
            }
            values.forEach(function (code) {
                put(name, code, Math.abs(width));
            });
        });
    }

    // ---------------- field layer ----------------

    /* The frame length must be exactly what its fields imply. */
    function checkEnd(kind, r, data) {
        var want = frameBytes(r.pos);
        if (data.length !== want) {
            throw new CodecError(kind + ": " + data.length + " bytes, expected " + want);
        }
    }

    /* The two array counts that set a frame's length. */
    function checkEventCounts(code, args) {
        if (code === EVC.FINE_SCAN && !(args.count >= 1 && args.count <= FINE_SCAN_MAX)) {
            throw new CodecError("FINE_SCAN: count " + args.count + " outside 1.." + FINE_SCAN_MAX);
        }
        if (code === EVC.BC_DBG && args.variant === 4 && args.count > BC_WIDTHS_MAX) {
            throw new CodecError("BC_DBG: count " + args.count + " above " + BC_WIDTHS_MAX);
        }
    }

    function bcDbgLayout(variant) {
        if (!BC_DBG_ARGS.has(variant)) {
            throw new CodecError("BC_DBG: unknown variant " + variant);
        }
        return BC_DBG_ARGS.get(variant);
    }

    function eventLayout(code) {
        return EVENT_ARGS[EVENTS[code][0]] || [];
    }

    function bytesToHex(data) {
        return Array.from(data, function (b) { return b.toString(16).padStart(2, "0"); }).join("");
    }

    function hexToBytes(text) {
        var hex = text.replace(/\s+/g, "");
        if (hex.length % 2 || /[^0-9a-fA-F]/.test(hex)) {
            throw new CodecError("non-hexadecimal number found in raw: " + pyRepr(text));
        }
        var out = new Uint8Array(hex.length / 2);
        for (var i = 0; i < out.length; i++) {
            out[i] = parseInt(hex.substr(2 * i, 2), 16);
        }
        return out;
    }

    function decodeEvent(data) {
        var r = new BitReader(data);
        r.get(4);
        var rec = { t: r.get(24), code: r.get(6) };
        if (rec.code >= EVENTS.length) {
            rec.raw = bytesToHex(data);         // unknown arguments: kept so the bytes survive
            return rec;
        }
        var args = {};
        if (rec.code === EVC.BC_DBG) {
            args.variant = r.get(3);
            readLayout(r, bcDbgLayout(args.variant), args);
        } else {
            readLayout(r, eventLayout(rec.code), args);
        }
        checkEventCounts(rec.code, args);
        checkEnd("event", r, data);
        rec.args = args;
        return rec;
    }

    function encodeEvent(rec) {
        if (rec.raw !== undefined) {
            return hexToBytes(rec.raw);
        }
        var code = need(rec, "code");
        if (!(code >= 0 && code < EVENTS.length)) {
            throw new CodecError("unknown Event_Code " + code);
        }
        var args = rec.args === undefined ? {} : rec.args;
        var w = new BitWriter();
        w.put("version", FORMAT_VERSION, 4);
        w.put("t", need(rec, "t"), 24);
        w.put("code", code, 6);
        if (code === EVC.BC_DBG) {
            w.put("variant", need(args, "variant"), 3);
            writeLayout(w, bcDbgLayout(args.variant), args);
        } else {
            writeLayout(w, eventLayout(code), args);
        }
        checkEventCounts(code, args);
        return w.toBytes();
    }

    function getpCodeWidth(paramId) {
        if (paramId >= PARAMS.length) {
            throw new CodecError("GET:PARAMS: unknown Parameter_ID " + paramId);
        }
        return PARAM_KINDS.get(PARAMS[paramId][1]);
    }

    function decodeAck(data) {
        var r = new BitReader(data);
        r.get(4);
        var rec = { t: r.get(24), cmd: r.get(4), reason: r.get(5) };
        var args = {};
        if (rec.cmd === WCMD.GET_PARAMS) {
            args.part = r.get(3);
            args.count = r.get(4);
            if (args.count > GETP_PER_PART) {
                throw new CodecError("GET:PARAMS: count " + args.count + " above " + GETP_PER_PART);
            }
            args.values = [];
            for (var i = 0; i < args.count; i++) {
                var paramId = r.get(6);
                args.values.push([paramId, r.get(getpCodeWidth(paramId))]);
            }
        } else if (ACK_ARGS.has(rec.cmd)) {
            readLayout(r, ACK_ARGS.get(rec.cmd), args);
        } else {
            rec.raw = bytesToHex(data);         // unknown arguments: kept so the bytes survive
            return rec;
        }
        checkEnd("ack", r, data);
        rec.args = args;
        return rec;
    }

    function encodeAck(rec) {
        if (rec.raw !== undefined) {
            return hexToBytes(rec.raw);
        }
        var args = rec.args === undefined ? {} : rec.args;
        var w = new BitWriter();
        w.put("version", FORMAT_VERSION, 4);
        w.put("t", need(rec, "t"), 24);
        w.put("cmd", need(rec, "cmd"), 4);
        w.put("reason", need(rec, "reason"), 5);
        if (rec.cmd === WCMD.GET_PARAMS) {
            w.put("part", need(args, "part"), 3);
            w.put("count", need(args, "count"), 4);
            var values = need(args, "values");
            if (values.length !== args.count) {
                throw new CodecError("GET:PARAMS: " + values.length + " values, count " + args.count);
            }
            values.forEach(function (pair) {
                w.put("param", pair[0], 6);
                w.put("value", pair[1], getpCodeWidth(pair[0]));
            });
        } else if (ACK_ARGS.has(rec.cmd)) {
            writeLayout(w, ACK_ARGS.get(rec.cmd), args);
        } else {
            throw new CodecError("unknown command code " + rec.cmd);
        }
        return w.toBytes();
    }

    function decodeCommand(data) {
        var r = new BitReader(data);
        r.get(4);
        var rec = { code: r.get(4) };
        if (!COMMAND_ARGS.has(rec.code)) {
            throw new CodecError("unassigned command code " + rec.code);
        }
        readLayout(r, COMMAND_ARGS.get(rec.code), rec);
        checkEnd("command", r, data);
        return rec;
    }

    function encodeCommand(rec) {
        var code = need(rec, "code");
        if (!COMMAND_ARGS.has(code)) {
            throw new CodecError("unassigned command code " + code);
        }
        var w = new BitWriter();
        w.put("version", FORMAT_VERSION, 4);
        w.put("code", code, 4);
        writeLayout(w, COMMAND_ARGS.get(code), rec);
        return w.toBytes();
    }

    function decodeConfig(data) {
        var r = new BitReader(data);
        r.get(4);
        var count = r.get(6);
        if (count > PARAM_COUNT) {
            throw new CodecError("config: count " + count + " above " + PARAM_COUNT);
        }
        var rec = { count: count, pairs: [] };
        for (var i = 0; i < count; i++) {
            var paramId = r.get(6);
            rec.pairs.push([paramId, r.get(32)]);       // value: F32 bit pattern
        }
        checkEnd("config", r, data);
        return rec;
    }

    function encodeConfigRecord(rec) {
        var count = need(rec, "count");
        var pairs = need(rec, "pairs");
        if (!(count >= 0 && count <= PARAM_COUNT) || pairs.length !== count) {
            throw new CodecError("config: count " + count + ", " + pairs.length + " pairs, at most " + PARAM_COUNT);
        }
        var w = new BitWriter();
        w.put("version", FORMAT_VERSION, 4);
        w.put("count", count, 6);
        pairs.forEach(function (pair) {
            w.put("param", pair[0], 6);
            w.put("value", pair[1], 32);
        });
        return w.toBytes();
    }

    function decodeFixed(kind, layout, data) {
        var r = new BitReader(data);
        r.get(4);
        var rec = readLayout(r, layout, {});
        checkEnd(kind, r, data);
        return rec;
    }

    function encodeFixed(layout, rec) {
        var w = new BitWriter();
        w.put("version", FORMAT_VERSION, 4);
        writeLayout(w, layout, rec);
        return w.toBytes();
    }

    var DECODERS = new Map([
        ["telemetry", function (data) { return decodeFixed("telemetry", TELEMETRY, data); }],
        ["heartbeat", function (data) { return decodeFixed("heartbeat", HEARTBEAT, data); }],
        ["event", decodeEvent],
        ["ack", decodeAck],
        ["command", decodeCommand],
        ["map", function (data) { return decodeFixed("map", MAP_BODY, data); }],
        ["config", decodeConfig]
    ]);

    var ENCODERS = new Map([
        ["telemetry", function (rec) { return encodeFixed(TELEMETRY, rec); }],
        ["heartbeat", function (rec) { return encodeFixed(HEARTBEAT, rec); }],
        ["event", encodeEvent],
        ["ack", encodeAck],
        ["command", encodeCommand],
        ["map", function (rec) { return encodeFixed(MAP_BODY, rec); }],
        ["config", encodeConfigRecord]
    ]);

    /* Bytes of any kind a MQTT client hands over (a string is taken as UTF-8). */
    function toBytes(payload) {
        if (typeof payload === "string") {
            return new TextEncoder().encode(payload);
        }
        if (payload instanceof Uint8Array) {
            return payload;
        }
        if (ArrayBuffer.isView(payload)) {
            return new Uint8Array(payload.buffer, payload.byteOffset, payload.byteLength);
        }
        if (payload instanceof ArrayBuffer) {
            return new Uint8Array(payload);
        }
        return Uint8Array.from(payload);
    }

    /* Payload bytes -> record of raw field codes. Throws CodecError on a bad version or length. */
    function decodeFields(kind, payload) {
        if (!DECODERS.has(kind)) {
            throw new CodecError("unknown payload kind " + pyRepr(kind));
        }
        var data = toBytes(payload);
        if (data.length === 0) {
            throw new CodecError(kind + ": empty payload");
        }
        if (data[0] >> 4 !== FORMAT_VERSION) {
            throw new CodecError(kind + ": unknown Format_Version " + (data[0] >> 4));
        }
        return DECODERS.get(kind)(data);
    }

    /* Record of raw field codes -> payload bytes. */
    function encodeFields(kind, record) {
        if (!ENCODERS.has(kind)) {
            throw new CodecError("unknown payload kind " + pyRepr(kind));
        }
        try {
            return ENCODERS.get(kind)(record);
        } catch (e) {
            if (e instanceof MissingField) {
                throw new CodecError(kind + ": record has no field " + pyRepr(e.field));
            }
            throw e;
        }
    }

    // ---------------- presentation helpers ----------------

    /* value as the nearest float32 (a float32 store on the robot). */
    function f32(value) {
        return Math.fround(value);
    }

    function f32FromBits(bits) {
        var view = new DataView(new ArrayBuffer(4));
        view.setUint32(0, bits, false);
        return view.getFloat32(0, false);
    }

    function f32ToBits(value) {
        var view = new DataView(new ArrayBuffer(4));
        view.setFloat32(0, value, false);
        return view.getUint32(0, false);
    }

    /* A fixed-point code as decimal text, without going through a float. */
    function fixedText(code, decimals) {
        var sign = code < 0 ? "-" : "";
        var mag = Math.abs(code);
        if (decimals === 0) {
            return sign + mag;
        }
        var scale = Math.pow(10, decimals);
        return sign + Math.floor(mag / scale) + "." + String(mag % scale).padStart(decimals, "0");
    }

    /* The text jb_fixed() prints for a float32 value, computed in float32 as jfmt.c does. */
    function jbFixed(value, decimals) {
        value = f32(value);
        if (Number.isNaN(value)) {
            return "null";
        }
        var scale = Math.pow(10, decimals);
        var mag = Math.min(Math.abs(value), f32(4.0e9 / scale));
        var total = Math.trunc(f32(f32(mag * scale) + 0.5));
        var sign = value < 0.0 && total > 0 ? "-" : "";
        return sign + fixedText(total, decimals);
    }

    function cm9Text(code) {
        return code === 511 ? "-1" : String(code);
    }

    function cm13Text(code) {
        return code === 8191 ? "-1.0" : fixedText(code, 1);
    }

    function cm13Value(code) {
        return code === 8191 ? -1.0 : code / 10;
    }

    /* table[code], or the number itself (logged) when the table has no such code. */
    function lookup(table, code, what, emit) {
        if (code >= 0 && code < table.length && table[code] !== null && table[code] !== undefined) {
            return table[code];
        }
        emit("unknown " + what + " " + code);
        return code;
    }

    function letterText(code, emit) {
        return code < 26 ? String.fromCharCode(65 + code) : String(lookup([], code, "letter", emit));
    }

    /* Code 39 index -> character; the number for an index no table lists. */
    function bcChar(code, emit) {
        return lookup(C39_CHARS, code, "barcode index", emit);
    }

    /* The text of nav_map_describe(): 26 action codes, letter A first. */
    function describeMap(acts, emit) {
        var parts = [];
        acts.forEach(function (act, i) {
            if (act !== ACT_NONE) {
                parts.push(String.fromCharCode(65 + i) + "=" + lookup(ACTIONS, act, "action", emit));
            }
        });
        return parts.join(",");
    }

    // ---------------- event detail texts (design D10) ----------------

    var FIXED_DETAIL = new Map([
        ["FAULT_IMU_I2C", "I2C1 timeout: bus recovery"],
        ["FAULT_IMU", "3 failed recoveries: driving without the IMU"],
        ["BATTERY_LOW", "below VBAT_LOW_MV"],
        ["START_REFUSED", "calibration missing (see console)"],
        ["RECOVERED", "re-check passed"],
        ["IMPACT", "while not driving"],
        ["BARCODE_FAULT", "3 ambiguous barcodes: going straight (C)"],
        ["BARCODE_FAILED", "window did not decode"],
        ["LINE_FOUND", "search"],
        ["LINE_LOST", "search found nothing"],
        ["BYPASS_RETURN", "line not seen crossing the original path: back to it, then search"],
        ["OBSTACLE_CLEARED", "coarse sweep clear: resume line following"]
    ]);

    function sideWord(right) {
        return right ? "right" : "left";
    }

    /* Side letter, or "-" for no side, as the obstacle events print it. */
    function sideChar(code, emit) {
        return lookup(SIDES, code, "side", emit) || "-";
    }

    function textImuOk(a, emit) {
        if (a.chip === 0) {
            return "none";
        }
        return lookup(IMU_CHIPS, a.chip, "IMU chip", emit) + " at 0x" + a.addr.toString(16).toUpperCase().padStart(2, "0");
    }

    function textHump(a, emit) {
        return "height " + fixedText(a.h, 1) + " mm, run max " + fixedText(a.max, 1) + " mm";
    }

    function textHumpEnd(a, emit) {
        return a.valid ? textHump(a, emit) : "false slope (no height reported)";
    }

    function textMission(a, emit) {
        return "t " + a.t + "s dist " + fixedText(a.dist, 2) + "m bc " + a.bc + " humps " + a.humps +
            " max " + fixedText(a.max, 1) + "mm obst " + a.obst;
    }

    function textNavAction(a, emit) {
        var text = letterText(a.letter, emit) + " -> " + lookup(ACTIONS, a.action, "action", emit);
        return text + (a.action === ACT_NONE ? " (not in the MQTT map): straight on" : "");
    }

    function textNav(a, emit) {
        var action = lookup(ACTIONS, a.action, "action", emit);
        return [String(action),
            "turn did not complete",
            letterText(a.letter, emit) + " stored for the next junction (" + action + ")",
            "barcode fault: going straight (C)"][a.variant];
    }

    function textRefused(a, emit) {
        var text = (a.why & 1) ? "obstacle closer than 8 cm. " : "";
        text += (a.why & 2) ? "IMU INT1 still active. " : "";
        return text + CONFIG_ITEMS.filter(function (name, i) { return (a.why >> (i + 2)) & 1; }).join(", ");
    }

    /* The " decoded K (forwards)" part shared by the window and pass texts. */
    function bcOutcome(a, emit) {
        return " decoded " + bcChar(a.code, emit) + (a.rev ? " (backwards)" : " (forwards)");
    }

    function bcTail(a) {
        return ", thr " + a.thr_b + "/" + a.thr_w + ", ovr " + a.ovr;
    }

    function textBcWindow(a, emit) {
        var outcome;
        if (a.res === RES_DECODED) {
            outcome = bcOutcome(a, emit);
        } else if (a.res === RES_NONE) {
            outcome = a.n < BARCODE_MIN_ELEMENTS ? " stray mark (fewer than 9)" : " closed";
        } else {
            outcome = " NOT decoded";
        }
        return "n=" + a.n + outcome + bcTail(a);
    }

    function textBcPass(a, emit) {
        var outcome;
        if (a.res === RES_DECODED) {
            outcome = bcOutcome(a, emit) + " at " + lookup(BC_LEVELS, a.level, "barcode level", emit);
        } else if (a.res === RES_NONE) {
            outcome = " stray mark (fewer than 9)";
        } else {
            outcome = a.conflict ? " NOT decoded: levels disagree" : " NOT decoded";
        }
        return "n=" + a.n + outcome + bcTail(a);
    }

    function textBcLevels(a, emit) {
        var parts = ["levels"];
        for (var i = 0; i < Math.min(a.n_levels, BC_LEVELS.length); i++) {
            var mark;
            if (a["res" + i] === RES_DECODED) {
                mark = bcChar(a["code" + i], emit);
            } else {
                mark = a["res" + i] === RES_NONE ? "-" : "x";
            }
            parts.push(BC_LEVELS[i] + ":" + mark + a["n" + i]);
        }
        return parts.join(" ");
    }

    function textBcWidths(a, emit) {
        return "w" + a.part + " (0.1mm, bar first):" + a.w.map(function (w) { return " " + w; }).join("");
    }

    function textBcDbg(a, emit) {
        if (a.variant === 0) {
            return "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close";
        }
        return [null, textBcWindow, textBcPass, textBcLevels, textBcWidths][a.variant](a, emit);
    }

    function textBarcode(a, emit) {
        return bcChar(a.code, emit) + " (read " + (a.rev ? "backwards" : "forwards") + ")";
    }

    function textSideFollow(a, emit) {
        return "60deg pings " + a.n60 + " seen " + a.seen + " near " + cm13Text(a.near) + "cm (<=" +
            fixedText(a.limit, 1) + ") leg " + a.leg + " x " + a.x;
    }

    function textSideLook(a, emit) {
        return "pivot look 90deg " + cm13Text(a.d90) + " cm, 60deg " + cm13Text(a.d60) + " cm: side " +
            (a.gone ? "gone" : "still there");
    }

    function textExtend(a, emit) {
        if (a.variant) {
            return "return blocked (" + cm9Text(a.cm) + " cm): back to the turn, 100 mm further";
        }
        return "obstacle still beside the car (" + cm9Text(a.cm) + " cm): 100 mm further";
    }

    function textProfile(a, emit) {
        return "r " + cm9Text(a.r) + "cm w " + a.w + "cm off " + a.off + " clrL " + a.clr_l + " clrR " + a.clr_r +
            " side " + sideChar(a.side, emit);
    }

    function textReverseCap(a, emit) {
        var starts = ["standoff " + a.n + " mm refused: ", "rescan skipped: ", "rescan reverse cut to " + a.n + " mm: "];
        var start = lookup(starts, a.variant, "OBST_REVERSE_CAP variant", emit);
        return start + a.used + " of " + a.cap + " mm used, " + a.total + " mm total";
    }

    function textComplete(a, emit) {
        return "passed on the " + sideWord(a.right) + (a.search ? "; on the path, searching for the line" : "");
    }

    function textBlocked(a, emit) {
        var start = sideWord(a.right) + " side: ";
        if (a.no_end) {
            return start + "obstacle side did not end in 1 m; backing out";
        }
        return start + "something at " + cm9Text(a.cm) + " cm, " + a.deg + " deg; backing out";
    }

    function textRetry(a, emit) {
        if (a.wider) {
            return sideWord(a.right) + " side again, 100 mm wider";
        }
        return "trying the " + sideWord(a.right) + " side (same scan)";
    }

    var EVENT_TEXT = new Map([
        ["STATE", function (a, emit) { return STATES[a.state]; }],
        ["IMU_OK", textImuOk],
        ["HUMP_START", textHump],
        ["HUMP_PEAK", textHump],
        ["HUMP_END", textHumpEnd],
        ["MAP", function (a, emit) { return describeMap(a.map, emit) || "cleared"; }],
        ["MAP_WARNING", function (a, emit) { return describeMap(a.map, emit) || "no barcode map yet (send MAP:...)"; }],
        ["MISSION_COMPLETE", textMission],
        ["TURN_SNAP", function (a, emit) {
            return a.not_seen ? "line not seen, encoder angle kept" : "stopped on the line";
        }],
        ["NAV_ACTION", textNavAction],
        ["NAV", textNav],
        ["EMERGENCY_STOP", function (a, emit) { return String(lookup(STOP_REASONS, a.reason, "stop reason", emit)); }],
        ["ERROR", function (a, emit) { return String(lookup(STOP_REASONS, a.reason, "stop reason", emit)); }],
        ["RECOVERY_REFUSED", textRefused],
        ["BC_DBG", textBcDbg],
        ["BARCODE", textBarcode],
        ["BARCODE_UNKNOWN", textBarcode],
        ["LINE_SEARCH", function (a, emit) { return a.right ? "right first" : "left first"; }],
        ["FINE_SCAN", function (a, emit) {
            return a.first + "deg+2:" + a.cm.map(function (c) { return " " + cm9Text(c); }).join("");
        }],
        ["SIDE_FOLLOW", textSideFollow],
        ["SIDE_LOOK", textSideLook],
        ["BYPASS_EXTEND", textExtend],
        ["BYPASS_SIDESTEP", function (a, emit) {
            return "too close beside the obstacle: back, then " + a.mm + " mm further out";
        }],
        ["OBSTACLE_PROFILE", textProfile],
        ["AVOIDANCE_ABORT", function (a, emit) { return String(lookup(ABORT_REASONS, a.reason, "abort reason", emit)); }],
        ["OBST_REVERSE_CAP", textReverseCap],
        ["OBSTACLE_BACKOFF", function (a, emit) {
            return "reversing " + a.need + " mm: a 45 deg turn would clip the " + sideWord(a.right) + " corner";
        }],
        ["OBSTACLE_RESCAN", function (a, emit) {
            return (a.blocked ? "blocked" : "no room") + ": reversing 100 mm and scanning again";
        }],
        ["OBSTACLE_TIE", function (a, emit) {
            return "tie: 90 deg look L " + a.dl + " cm, R " + a.dr + " cm -> side " + sideChar(a.side, emit);
        }],
        ["BYPASS_COMPLETE", textComplete],
        ["BYPASS_BLOCKED", textBlocked],
        ["BYPASS_RETRY", textRetry],
        ["OBSTACLE", function (a, emit) { return "at " + cm13Text(a.cm) + " cm, STOP issued in " + a.us + " us"; }]
    ]);

    // ---------------- presentation layer ----------------

    /* -x, but 0 stays +0 (Python -0 is 0). */
    function neg(x) {
        return 0 - x;
    }

    function presentTelemetry(rec, emit) {
        var bc = rec.bc === C39_NONE ? "" : bcChar(rec.bc, emit);
        return {
            seq: rec.seq,
            t: rec.t,
            state: STATES[rec.state],
            spd: [rec.spd_l / 10, rec.spd_r / 10],
            duty: [rec.duty_l / 10, rec.duty_r / 10],
            enc: [rec.enc_l, rec.enc_r],
            dist: rec.dist / 10,
            line: {
                l: rec.line_l, r: rec.line_r, ir: rec.ir, irmn: rec.ir_min, irmx: rec.ir_max,
                thr: rec.thr_valid ? [rec.thr_b, rec.thr_w] : [0, 0]
            },
            bc: bc,
            hump: {
                pitch: rec.pitch / 10, h: rec.hump_h / 10,
                peak: rec.hump_peak / 10, max: rec.hump_max / 10
            },
            obst: {
                cm: cm13Value(rec.us), scan: rec.scan.map(cm13Value),
                side: lookup(SIDES, rec.side, "side", emit)
            },
            imu: { ev: MOTION_EVENTS[rec.imu_ev], tr: rec.turn / 10 },
            faults: rec.faults,
            vbat: rec.vbat,
            rssi: neg(rec.rssi),
            buf: rec.buf
        };
    }

    function presentStatus(rec, emit) {
        return {
            seq: rec.seq,
            t: rec.t,
            state: STATES[rec.state],
            rssi: neg(rec.rssi),
            buf: rec.buf,
            up: rec.up,
            reconnects: rec.reconnects,
            dropped: rec.dropped,
            faults: rec.faults,
            broker: rec.broker.join("."),
            sat: rec.sat,
            ptab: { count: rec.ptab_count, crc: rec.ptab_crc }
        };
    }

    function presentEvent(rec, emit) {
        if (rec.code >= EVENTS.length) {
            emit("unknown Event_Code " + rec.code);
            return { t: rec.t, event: rec.code, detail: "", args: {} };
        }
        var name = EVENTS[rec.code][0];
        var text = FIXED_DETAIL.get(name);
        if (text === undefined) {
            text = EVENT_TEXT.get(name)(rec.args, emit);
        }
        return { t: rec.t, event: name, detail: text, args: rec.args };
    }

    /* Parameter_Store code -> the value, as GET:PARAMS printed it (jb_fixed, 4 decimals). */
    function paramValue(paramId, code) {
        var kind = PARAMS[paramId][1];
        if (kind === "F32") {
            var text = jbFixed(f32FromBits(code), 4);
            return text === "null" ? null : Number(text);
        }
        if (kind === "S2") {
            return code >= 2 ? code - 4 : code;
        }
        return code + (kind === "F1" ? 0 : kind === "B2-1" ? -1 : 1);
    }

    function paramName(paramId, emit) {
        return lookup(PARAMS.map(function (p) { return p[0]; }), paramId, "Parameter_ID", emit);
    }

    /* The Baseline_Build "cmd" text. */
    function ackEcho(code, args, emit) {
        if (code === WCMD_UNPARSED) {
            return "";
        }
        if (!(code >= 0 && code < COMMANDS.length)) {
            emit("unknown command code " + code);
            return code;
        }
        var echo = COMMANDS[code][1];
        if (code === WCMD.PING) {
            return echo + ":" + args.ping;
        }
        if (code === WCMD.BARCODE) {
            return echo + ":" + letterText(args.letter, emit);
        }
        if (code === WCMD.SET) {
            return echo + ":" + paramName(args.param, emit);
        }
        return echo;
    }

    function presentAck(rec, emit) {
        var args = rec.args === undefined ? {} : rec.args;
        var code = rec.cmd;
        var result;
        if (code === WCMD.CONFIG && rec.reason === RSN_CONFIG_REJECTED) {
            result = args.rej.map(function (i) { return String(paramName(i, emit)); }).join(" ");
        } else if (rec.reason >= 0 && rec.reason < REASONS.length) {
            result = REASONS[rec.reason][1];
        } else {
            emit("unknown Reason_Code " + rec.reason);
            result = rec.reason;
        }
        var out = { t: rec.t, cmd: ackEcho(code, args, emit), result: result };
        if (code === WCMD.ESTOP) {
            out.pwm_off_us = args.pwm_off_us;
        } else if (code === WCMD.MAP || code === WCMD.MAP_CLEAR || code === WCMD.GET_MAP) {
            out.map = describeMap(args.map, emit);
            out.n = args.n;
        } else if (code === WCMD.GET_PARAMS) {
            out.part = args.part;
            out.params = {};
            args.values.forEach(function (pair) {
                out.params[PARAMS[pair[0]][0]] = paramValue(pair[0], pair[1]);
            });
        } else if (code === WCMD.CONFIG) {
            out.applied = args.applied;
            out.rejected = args.n_rej;
        }
        return out;
    }

    /* A map body as "F=FWD,K=LEFT,...", letter A first (nav_map_describe order). */
    function slotsText(rec, emit) {
        var words = { fwd: "FWD", left: "LEFT", right: "RIGHT", uturn: "UTURN" };
        var pairs = MAP_ORDER.filter(function (slot) { return rec[slot + "_on"]; })
            .map(function (slot) { return [rec[slot], words[slot]]; });
        pairs.sort(function (x, y) {
            return x[0] - y[0] || (x[1] < y[1] ? -1 : x[1] > y[1] ? 1 : 0);
        });
        return pairs.map(function (p) { return letterText(p[0], emit) + "=" + p[1]; }).join(",");
    }

    function presentCommand(rec, emit) {
        var out = { cmd: ackEcho(rec.code, rec, emit) };
        if (rec.code === WCMD.MAP) {
            out.map = slotsText(rec, emit);
        } else if (rec.code === WCMD.SET) {
            out.value = f32FromBits(rec.value);
        }
        return out;
    }

    function presentMap(rec, emit) {
        var n = 0;
        MAP_ORDER.forEach(function (slot) { n += rec[slot + "_on"]; });
        return { map: slotsText(rec, emit), n: n };
    }

    function presentConfig(rec, emit) {
        var params = {};
        rec.pairs.forEach(function (pair) {
            params[paramName(pair[0], emit)] = f32FromBits(pair[1]);
        });
        return { count: rec.count, params: params };
    }

    // topic -> [payload kind, presentation]
    var TOPICS = new Map([
        ["is02/telemetry", ["telemetry", presentTelemetry]],
        ["is02/status", ["heartbeat", presentStatus]],
        ["is02/event", ["event", presentEvent]],
        ["is02/ack", ["ack", presentAck]],
        ["is02/cmd", ["command", presentCommand]],
        ["is02/map", ["map", presentMap]],
        ["is02/config", ["config", presentConfig]]
    ]);
    var RETAINED_TOPICS = ["is02/map", "is02/config"];
    // Topics the tools fill with baseline text while legacy mode is on (flag R7)
    var TEXT_TOPICS = ["is02/cmd", "is02/map", "is02/config"];

    /* The Baseline_Build JSON shape of a payload, or null if it was dropped.
     *
     * options.log: a function taking one line of text for bad versions, lengths
     * and unknown codes (default console.warn). An empty retained payload is the
     * broker deleting it, so it returns null without a log line.
     * options.legacyJson: a payload starting with "{" is parsed as baseline JSON.
     */
    function decode(topic, payload, options) {
        options = options || {};
        var emit = options.log || function (line) { console.warn(line); };
        var data = toBytes(payload);
        if (!TOPICS.has(topic)) {
            emit(topic + ": no decoder for this topic, length " + data.length);
            return null;
        }
        if (options.legacyJson && data[0] === 0x7B) {
            try {
                return JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(data));
            } catch (e) {
                emit(topic + ": dropped, invalid JSON: " + e.message);
                return null;
            }
        }
        if (data.length === 0 && RETAINED_TOPICS.indexOf(topic) >= 0) {
            return null;
        }
        var entry = TOPICS.get(topic);
        var rec;
        try {
            rec = decodeFields(entry[0], data);
        } catch (e) {
            if (!(e instanceof CodecError)) {
                throw e;
            }
            var version = data.length ? data[0] >> 4 : "none";
            emit(topic + ": dropped, version " + version + ", length " + data.length + ": " + e.message);
            return null;
        }
        return entry[1](rec, emit);
    }

    /* What the tools do with a received payload, in both modes: {decoded, text}.
     *
     * "{" is baseline JSON, anything else binary (flag R7). On TEXT_TOPICS and on a
     * topic with no decoder, a payload without our Format_Version is text, and
     * decoded is null. text is the payload as received, or the decoded JSON of a
     * binary frame, or null if the frame was dropped.
     */
    function readPayload(topic, payload, options) {
        var data = toBytes(payload);
        var received = new TextDecoder("utf-8").decode(data);
        if (!TOPICS.has(topic) || (TEXT_TOPICS.indexOf(topic) >= 0 &&
                (data.length === 0 || data[0] >> 4 !== FORMAT_VERSION))) {
            return { decoded: null, text: received };
        }
        var log = options && options.log;
        var decoded = decode(topic, data, { legacyJson: true, log: log });
        if (data[0] === 0x7B) {
            return { decoded: decoded, text: received };
        }
        return { decoded: decoded, text: decoded === null ? null : JSON.stringify(decoded) };
    }

    /* The warning for a decoded heartbeat whose parameter table differs from PARAMS, else null.
     * A baseline JSON heartbeat has no "ptab", so it gives null.
     */
    function ptabMismatch(status) {
        var ptab = status.ptab;
        if (ptab === undefined || (ptab.count === PARAM_COUNT && ptab.crc === PARAM_CRC)) {
            return null;
        }
        var hex = function (crc) { return "0x" + crc.toString(16).toUpperCase().padStart(4, "0"); };
        return "parameter table mismatch: robot " + ptab.count + " / " + hex(ptab.crc) +
            ", tool " + PARAM_COUNT + " / " + hex(PARAM_CRC);
    }

    // ---------------- user input -> frames ----------------

    // the robot's words (nav_map.c), compared without case, "-" and "_"
    var MOVES = new Map([["FWD", 1], ["FORWARD", 1], ["STRAIGHT", 1], ["F", 1], ["UTURN", 2], ["U", 2],
        ["BACK", 2], ["LEFT", 3], ["L", 3], ["RIGHT", 4], ["R", 4]]);
    var POSITIONAL = [1, 2, 3, 4];          // MAP:F,U,K,R gives FWD, UTURN, LEFT, RIGHT
    var NUMBER = /^[+-]?(\d+\.?\d*|\.\d+)$/;  // what jf_parse_float accepts: no exponent

    /* Map text -> Map {movement: letter code}; empty for CLEAR. Same rules as nav_map_parse(). */
    function parseMap(text) {
        text = text.replace(/^[ \t\r\n,;]+|[ \t\r\n,;]+$/g, "").replace(/^MAP:/, "").replace(/^[ \t\r\n,;]+/, "");
        var slots = new Map();
        if (text.slice(0, 5).toUpperCase() === "CLEAR") {
            return slots;
        }
        var pairs = [];
        var nPos = 0;
        var nNamed = 0;
        text.split(/[,; \t\r\n]+/).forEach(function (item) {
            if (!item) {
                return;
            }
            var letter = item[0].toUpperCase();
            if (!/^[A-Z]$/.test(letter)) {
                throw new CodecError("map: " + item + ": letters must be A-Z");
            }
            var action;
            if (item.length >= 3 && item[1] === "=") {
                action = MOVES.get(item.slice(2).replace(/-/g, "").replace(/_/g, "").toUpperCase());
                if (action === undefined) {
                    throw new CodecError("map: " + item + ": unknown movement (use FWD, LEFT, RIGHT, UTURN)");
                }
                nNamed++;
            } else if (item.length === 1) {
                if (nPos >= 4) {
                    throw new CodecError("map: " + item + ": positional form takes exactly 4 letters");
                }
                action = POSITIONAL[nPos];
                nPos++;
            } else {
                throw new CodecError("map: " + item + ": expected L=ACTION or a single letter");
            }
            pairs.push([letter, action]);
        });
        if (nPos && nNamed) {
            throw new CodecError("map: do not mix the two forms");
        }
        if (nPos && nPos !== 4) {
            throw new CodecError("map: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT");
        }
        if (!pairs.length) {
            throw new CodecError("map: empty map");
        }
        var seen = new Set();
        pairs.forEach(function (pair) {
            var letter = pair[0], action = pair[1];
            if (seen.has(letter)) {
                throw new CodecError("map: " + letter + ": the same letter was given twice");
            }
            seen.add(letter);
            var slot = MAP_SLOTS.get(action);
            if (slots.has(slot)) {
                throw new CodecError("map: " + letter + ": " + ACTIONS[action] + " already has letter " +
                    String.fromCharCode(65 + slots.get(slot)) +
                    ": the Map_Frame holds one letter per movement");
            }
            slots.set(slot, letter.charCodeAt(0) - 65);
        });
        return slots;
    }

    function mapBody(slots) {
        if (!(slots instanceof Map)) {
            slots = new Map(Object.keys(slots).map(function (k) { return [k, slots[k]]; }));
        }
        var rec = {};
        MAP_ORDER.forEach(function (slot) {
            rec[slot + "_on"] = slots.has(slot) ? 1 : 0;
            rec[slot] = slots.has(slot) ? slots.get(slot) : 0;
        });
        return rec;
    }

    /* The Map_Frame. pairs: map text ("K=LEFT,Q=RIGHT") or a list of "K=LEFT" or ["K", "LEFT"]. */
    function encodeMap(pairs) {
        if (typeof pairs !== "string") {
            pairs = Array.from(pairs, function (p) { return typeof p === "string" ? p : p[0] + "=" + p[1]; }).join(",");
        }
        return encodeFields("map", mapBody(parseMap(pairs)));
    }

    /* "NAME=VALUE" -> [name, Parameter_ID, F32 bits]. CodecError names the NAME. */
    function parseParam(item) {
        if (item.startsWith("SET:")) {
            item = item.slice(4);
        }
        var cut = item.indexOf("=");
        var name = (cut < 0 ? item : item.slice(0, cut)).trim();
        var value = (cut < 0 ? "" : item.slice(cut + 1)).trim();
        if (cut < 0) {
            throw new CodecError(name + ": expected NAME=VALUE");
        }
        if (!PARAM_ID_MAP.has(name)) {
            throw new CodecError(name + ": unknown parameter");
        }
        if (!NUMBER.test(value)) {
            throw new CodecError(name + ": bad number " + pyRepr(value));
        }
        var paramId = PARAM_ID_MAP.get(name);
        var number = Number(value);
        var kind = PARAMS[paramId][1], low = PARAMS[paramId][2], high = PARAMS[paramId][3];
        var range = name + ": " + value + " out of range " + pyFloat(low) + ".." + pyFloat(high);
        var store = f32(number);
        if (Number.isFinite(number) && !Number.isFinite(store)) {
            throw new CodecError(range);
        }
        if (store < f32(low) || store > f32(high)) {    // the robot compares float32 values
            throw new CodecError(range);
        }
        if (kind !== "F32" && number !== Math.trunc(number)) {
            throw new CodecError(name + ": " + value + " is not a whole number");
        }
        return [name, paramId, f32ToBits(number)];
    }

    var SIMPLE_COMMANDS = new Map([["ESTOP", "ESTOP"], ["RESUME", "RESUME"], ["START", "START"], ["STOP", "STOP"],
        ["GET:MAP", "GET_MAP"], ["GET:PARAMS", "GET_PARAMS"]]);

    /* A Baseline_Build command text -> its Command_Frame. CodecError names the argument. */
    function encodeCommandText(text) {
        var cmd = text.trim();
        if (SIMPLE_COMMANDS.has(cmd)) {
            return encodeFields("command", { code: WCMD[SIMPLE_COMMANDS.get(cmd)] });
        }
        if (cmd === "PING" || cmd.startsWith("PING:")) {
            var number = cmd.slice(5);
            if (cmd !== "PING" && !(/^[0-9]+$/.test(number) && Number(number) < 65536)) {
                throw new CodecError("argument " + cmd + ": PING takes a number 0..65535");
            }
            return encodeFields("command", { code: WCMD.PING, ping: Number(number || 0) });
        }
        if (cmd.startsWith("BARCODE:")) {
            var letter = cmd.slice(8);
            if (!/^[A-Z]$/.test(letter)) {
                throw new CodecError("argument " + cmd + ": use BARCODE:<A-Z>");
            }
            return encodeFields("command", { code: WCMD.BARCODE, letter: letter.charCodeAt(0) - 65 });
        }
        if (cmd.startsWith("SET:")) {
            var parsed;
            try {
                parsed = parseParam(cmd);
            } catch (e) {
                if (e instanceof CodecError) {
                    throw new CodecError("argument " + cmd + ": " + e.message);
                }
                throw e;
            }
            return encodeFields("command", { code: WCMD.SET, param: parsed[1], value: parsed[2] });
        }
        if (cmd.startsWith("MAP:")) {
            var slots = parseMap(cmd);
            if (!slots.size) {
                return encodeFields("command", { code: WCMD.MAP_CLEAR });
            }
            return encodeFields("command", Object.assign(mapBody(slots), { code: WCMD.MAP }));
        }
        throw new CodecError("argument " + cmd + ": unknown command");
    }

    /* Python str.splitlines(): no empty last line after a final line break. */
    function splitLines(text) {
        var lines = text.split(new RegExp("\r\n|[\n\r\v\f\x1c-\x1e\x85\u2028\u2029]"));
        if (lines.length && lines[lines.length - 1] === "") {
            lines.pop();
        }
        return lines;
    }

    /* NAME=VALUE lines -> [[Parameter_ID, F32 bits], ...].
     *
     * Blank lines and # comments are ignored, and one line may hold several items
     * split by ; or , as on the robot. Every bad line is listed in the CodecError,
     * with its line number (from 1) and NAME.
     */
    function parseConfig(lines) {
        if (typeof lines === "string") {
            lines = splitLines(lines);
        }
        var pairs = [];
        var errors = [];
        Array.from(lines).forEach(function (raw, index) {
            raw.split("#")[0].split(/[;,]/).forEach(function (item) {
                item = item.trim();
                if (!item) {
                    return;
                }
                try {
                    var parsed = parseParam(item);
                    pairs.push([parsed[1], parsed[2]]);
                } catch (e) {
                    if (!(e instanceof CodecError)) {
                        throw e;
                    }
                    var cut = e.message.indexOf(": ");
                    errors.push([index + 1, e.message.slice(0, cut), e.message.slice(cut + 2)]);
                }
            });
        });
        if (errors.length) {
            throw new CodecError(errors.map(function (e) {
                return "line " + e[0] + ": " + e[1] + ": " + e[2];
            }).join("\n"), errors);
        }
        if (pairs.length > PARAM_COUNT) {
            throw new CodecError(pairs.length + " values: the Config_Frame holds at most " + PARAM_COUNT);
        }
        return pairs;
    }

    /* robot_params.txt lines (or its whole text) -> the Config_Frame. */
    function encodeConfig(lines) {
        var pairs = parseConfig(lines);
        return encodeFields("config", { count: pairs.length, pairs: pairs });
    }

    return {
        FORMAT_VERSION: FORMAT_VERSION,
        CodecError: CodecError,
        EVENTS: EVENTS,
        EVC: EVC,
        REASONS: REASONS,
        RSN_CONFIG_REJECTED: RSN_CONFIG_REJECTED,
        COMMANDS: COMMANDS,
        WCMD: WCMD,
        WCMD_UNPARSED: WCMD_UNPARSED,
        STOP_REASONS: STOP_REASONS,
        ABORT_REASONS: ABORT_REASONS,
        STATES: STATES,
        MOTION_EVENTS: MOTION_EVENTS,
        ACTIONS: ACTIONS,
        C39_CHARS: C39_CHARS,
        BC_LEVELS: BC_LEVELS,
        CONFIG_ITEMS: CONFIG_ITEMS,
        TLM_BITS: TLM_BITS,
        TLM_LEN: TLM_LEN,
        HB_BITS: HB_BITS,
        HB_LEN: HB_LEN,
        MAP_LEN: MAP_LEN,
        EVT_HDR_BITS: EVT_HDR_BITS,
        EVT_MAX_LEN: EVT_MAX_LEN,
        ACK_HDR_BITS: ACK_HDR_BITS,
        ACK_UNPARSED_LEN: ACK_UNPARSED_LEN,
        GETP_PER_PART: GETP_PER_PART,
        GETP_MAX_LEN: GETP_MAX_LEN,
        FINE_SCAN_MAX: FINE_SCAN_MAX,
        BC_WIDTHS_MAX: BC_WIDTHS_MAX,
        frameBytes: frameBytes,
        evtLen: evtLen,
        cfgLen: cfgLen,
        cfgAckLen: cfgAckLen,
        PARAMS: PARAMS,
        PARAM_IDS: PARAM_IDS,
        PARAM_NOT_FOUND: PARAM_NOT_FOUND,
        PARAM_COUNT: PARAM_COUNT,
        PARAM_CRC: PARAM_CRC,
        crc16CcittFalse: crc16CcittFalse,
        paramTableVersion: paramTableVersion,
        TELEMETRY: TELEMETRY,
        HEARTBEAT: HEARTBEAT,
        EVENT_ARGS: EVENT_ARGS,
        FIXED_DETAIL: FIXED_DETAIL,
        EVENT_TEXT: EVENT_TEXT,
        decodeFields: decodeFields,
        encodeFields: encodeFields,
        f32: f32,
        f32FromBits: f32FromBits,
        f32ToBits: f32ToBits,
        jbFixed: jbFixed,
        mapBody: mapBody,
        decode: decode,
        readPayload: readPayload,
        ptabMismatch: ptabMismatch,
        encodeCommandText: encodeCommandText,
        encodeMap: encodeMap,
        encodeConfig: encodeConfig
    };
}));
