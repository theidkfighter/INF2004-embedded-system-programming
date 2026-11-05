/**
 * @file    wire.h
 * @brief   Wire_Codec: compact binary MQTT payloads (Change Requirement 1).
 *
 * Every payload is an MSB-first bitstream: bit 0 is the most significant bit
 * of byte 0. Byte 0 bits 7..4 always hold the Format_Version
 * (WIRE_FORMAT_VERSION), so a text payload ('A'..'Z' = 0x41..0x5A) always
 * reads as an unknown version (4 or 5). Unused bits after the last field are
 * 0, and a decoder rejects a set padding bit.
 *
 * Field notation: Uw unsigned, Sw two's complement, Ew enum code, Mw value
 * modulo 2^w (wraps), F32 IEEE-754 float bits.
 *
 * Telemetry_Frame (is02/telemetry, 432 bits = 54 bytes, no padding):
 *   ver U4 | seq U15 | t M24 | state E3 | spd_l S15 | spd_r S15 (0.1 mm/s)
 *   | duty_l S11 | duty_r S11 (0.1 %) | enc_l S24 | enc_r S24 | dist U20 (0.1 mm)
 *   | line_l S2 | line_r S2 | ir U12 | ir_min U12 | ir_max U12
 *   | thr_valid U1 | thr_b U12 | thr_w U12 (both 0 when thr_valid = 0)
 *   | bc U6 (0 none, else ASCII - 32) | side U2 (0 none, 1 L, 2 R)
 *   | pitch S11 (0.1 deg) | hump_h S15 | hump_peak U14 | hump_max U14 (0.1 mm)
 *   | us U13 | scan U13 x 5 (0.1 cm, 8191 = no echo) | imu_ev E3
 *   | rssi U8 (-dBm) | turn S14 (0.1 deg/s) | faults U16 | vbat U13 (mV)
 *   | buf U7 (Frame_Ring fill)
 *
 * Heartbeat_Frame (is02/status, 195 bits + 5 padding = 25 bytes):
 *   ver U4 | seq U14 | state E3 | rssi U8 | t M24 | uptime_s U17 (saturates)
 *   | reconnects U16 | dropped U16 | sat U16 | faults U16 | param_crc U16
 *   | buf U7 | n_params U6 | ip 4 x U8
 *
 * Event_Frame (is02/event): ver U4 | t M24 | Event_Code E6 | arguments
 *   (layout per code in evt.h; the codec only checks the bit count).
 *
 * Ack_Frame (is02/ack): ver U4 | t M24 | cmd E4 | reason E5 | arguments:
 *   ESTOP pwm_off_us U16; BARCODE letter U5; MAP, MAP_CLEAR, GET_MAP
 *   26 x action E3 + count U5; PING U16; SET param_id U6;
 *   GET_PARAMS part U3, count U4, count x (id U6, value w bits);
 *   CONFIG applied U6, n_rej U6, n_rej x id U6; others none.
 *
 * Command_Frame (is02/cmd): ver U4 | cmd E4 | arguments:
 *   BARCODE letter U5 (0 = 'A') + 3 zero bits; MAP 4 x (present U1,
 *   letter U5) in the order FORWARD, LEFT, RIGHT, UTURN (D13); PING U16;
 *   SET param_id U6, value F32, 2 zero bits; others none.
 * Map_Frame (is02/map, retained): ver U4 | the 24-bit MAP body.
 * Config_Frame (is02/config, retained): ver U4 | count U6 |
 *   count x (param_id U6, value F32).
 *
 * Plain C (standard headers only): used by the firmware and the
 * Host_Test_Suite.
 */
#ifndef WIRE_H
#define WIRE_H

#include <stdbool.h>
#include <stdint.h>

#include "wire_ids.h"

/* ---- decode status ------------------------------------------------------- */
typedef enum
{
    WIRE_OK = 0,
    WIRE_BAD_VERSION,       /* byte 0 bits 7..4 are not WIRE_FORMAT_VERSION */
    WIRE_BAD_CODE,          /* unknown event / command code */
    WIRE_BAD_LENGTH,        /* payload length does not match its content */
    WIRE_BAD_LETTER,        /* letter outside A..Z */
    WIRE_BAD_DUP_LETTER,    /* the same letter twice in a map */
    WIRE_BAD_COUNT,         /* a count outside its range */
    WIRE_BAD_VALUE          /* any other field outside its range, set padding bit */
} wire_status_t;

#define WIRE_EVT_ARG_BYTES   (26u)    /* WIRE_EVT_ARG_CAP_BITS / 8 */
#define WIRE_MAP_LETTERS     (26u)
#define WIRE_CFG_REJ_MAX     (61u)
#define WIRE_PARAMS_MAX      (61u)

/* ---- bit writer / reader ------------------------------------------------- */
typedef struct
{
    uint8_t *p;
    uint32_t cap_bits;
    uint32_t pos;
    bool     b_overflow;      /* a put did not fit; pos is unchanged */
} bw_t;

typedef struct
{
    const uint8_t *p;
    uint32_t       len_bits;
    uint32_t       pos;
    bool           b_short;   /* a get ran past the end; it returned 0 */
} br_t;

void     bw_init(bw_t *p_w, uint8_t *p_buf, uint32_t cap_bytes);   /* zero-fills */
void     bw_put(bw_t *p_w, uint32_t value, uint32_t width);        /* low width bits */
void     bw_copy(bw_t *p_w, const uint8_t *p_src, uint32_t n_bits);    /* MSB-first bits */
uint32_t bw_bytes(const bw_t *p_w);                                  /* bytes used */
void     br_init(br_t *p_r, const uint8_t *p_buf, uint32_t len_bytes);
uint32_t br_get(br_t *p_r, uint32_t width);
int32_t  br_get_s(br_t *p_r, uint32_t width);                        /* sign-extended */

/* ---- frame contents (decoded form) --------------------------------------- */
typedef struct
{
    uint16_t seq;
    uint32_t t_ms;
    uint8_t  state;
    uint8_t  thr_valid;
    int16_t  spd[2];          /* 0.1 mm/s, [left, right] */
    int16_t  duty[2];         /* 0.1 % */
    int32_t  enc[2];
    uint32_t dist;            /* 0.1 mm */
    int8_t   line_l;
    int8_t   line_r;
    uint16_t ir;
    uint16_t ir_min;
    uint16_t ir_max;
    uint16_t thr_b;
    uint16_t thr_w;
    uint8_t  bc;
    uint8_t  side;
    int16_t  pitch;           /* 0.1 deg */
    int16_t  hump_h;          /* 0.1 mm */
    uint16_t hump_peak;
    uint16_t hump_max;
    uint16_t us;              /* 0.1 cm, 8191 = no echo */
    uint16_t scan[5];
    uint8_t  imu_ev;
    uint8_t  rssi;            /* -dBm */
    int16_t  turn;            /* 0.1 deg/s */
    uint16_t faults;
    uint16_t vbat;            /* mV */
    uint8_t  buf;
} wire_tlm_t;

typedef struct
{
    uint16_t seq;
    uint8_t  state;
    uint8_t  rssi;
    uint32_t t_ms;
    uint32_t uptime_s;
    uint16_t reconnects;
    uint16_t dropped;
    uint16_t sat;
    uint16_t faults;
    uint16_t param_crc;
    uint8_t  buf;
    uint8_t  n_params;
    uint8_t  ip[4];
} wire_hb_t;

typedef struct
{
    uint8_t code;                       /* evt_code_t */
    uint8_t nbits;                      /* argument bits used */
    uint8_t args[WIRE_EVT_ARG_BYTES];   /* MSB-first, padding 0 */
} wire_evt_t;

typedef struct
{
    uint32_t code;            /* value bits (F32 bits or packed code) */
    uint8_t  id;              /* Parameter_ID */
    uint8_t  w;               /* value width: 32, or 1..3 for packed parameters */
} wire_pval_t;

typedef struct
{
    uint32_t t_ms;
    uint8_t  cmd;             /* wire_cmd_code_t */
    uint8_t  reason;          /* wire_reason_t */
    union
    {
        uint16_t pwm_off_us;
        uint16_t ping;
        uint8_t  letter;
        uint8_t  param_id;
        struct
        {
            uint8_t act[WIRE_MAP_LETTERS];   /* nav_action_t per letter A..Z */
            uint8_t n;
        } map;
        struct
        {
            uint8_t     part;
            uint8_t     count;
            wire_pval_t v[WIRE_GETP_PER_PART];
        } getp;
        struct
        {
            uint8_t applied;
            uint8_t n_rej;
            uint8_t rej[WIRE_CFG_REJ_MAX];
        } cfg;
    } a;
} wire_ack_t;

/* Decoded Command_Frame (laptop -> robot). */
typedef struct
{
    uint8_t  cmd;             /* wire_cmd_code_t */
    uint8_t  letter;          /* BARCODE: 0..25 */
    uint16_t ping;
    uint8_t  param_id;        /* SET */
    uint32_t value_bits;      /* SET: F32 bits */
    uint8_t  map_present[4];  /* MAP: FORWARD, LEFT, RIGHT, UTURN (D13) */
    uint8_t  map_letter[4];
} wire_cmd_t;

/* ---- codec ---------------------------------------------------------------- */
/* Encoders return the payload length in bytes, or 0 when a field is out of
   range or the buffer is too small. Decoders return a wire_status_t. */
uint32_t wire_enc_telemetry(const wire_tlm_t *p_t, uint8_t *p_out);          /* WIRE_TLM_LEN */
int      wire_dec_telemetry(const uint8_t *p, uint32_t len, wire_tlm_t *p_t);
uint32_t wire_enc_heartbeat(const wire_hb_t *p_h, uint8_t *p_out);           /* WIRE_HB_LEN */
int      wire_dec_heartbeat(const uint8_t *p, uint32_t len, wire_hb_t *p_h);

/* Argument bit count of an event from its arguments (variable events read
   their count field). avail = argument bits present. */
int      wire_evt_arg_bits(uint32_t code, const uint8_t *p_args, uint32_t avail, uint32_t *p_bits);
uint32_t wire_enc_event(uint32_t t_ms, const wire_evt_t *p_e, uint8_t *p_out, uint32_t cap);
int      wire_dec_event(const uint8_t *p, uint32_t len, uint32_t *p_t_ms, wire_evt_t *p_e);

/* p_pbits(param_id) gives a GET:PARAMS value width (NULL or 0 = 32 bits). */
typedef uint32_t (*wire_pbits_fn)(uint32_t param_id);
uint32_t wire_enc_ack(const wire_ack_t *p_a, uint8_t *p_out, uint32_t cap);
int      wire_dec_ack(const uint8_t *p, uint32_t len, wire_pbits_fn p_pbits, wire_ack_t *p_a);

int      wire_dec_cmd(const uint8_t *p, uint32_t len, wire_cmd_t *p_c);
uint32_t wire_enc_cmd(const wire_cmd_t *p_c, uint8_t *p_out, uint32_t cap);
int      wire_dec_map(const uint8_t *p, uint32_t len, wire_cmd_t *p_c);       /* is02/map */

/* Config_Frame: calls p_apply(param_id, value_bits) for each pair in order. */
typedef void (*wire_cfg_fn)(uint32_t param_id, uint32_t value_bits, void *p_ctx);
int      wire_dec_config(const uint8_t *p, uint32_t len, wire_cfg_fn p_apply, void *p_ctx);
/* Inverse encoders (host tests and tools): Map_Frame, Config_Frame. */
uint32_t wire_enc_map(const wire_cmd_t *p_c, uint8_t *p_out, uint32_t cap);
uint32_t wire_enc_config(uint32_t n, const uint8_t *p_ids, const uint32_t *p_bits, uint8_t *p_out, uint32_t cap);

#endif /* WIRE_H */
