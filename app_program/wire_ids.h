/**
 * @file    wire_ids.h
 * @brief   Wire protocol identifiers: Format_Version, Event_Codes, Reason_Codes,
 *          command codes, stop reasons, abort reasons and fixed frame lengths.
 *
 * Single source of the code tables of the Protocol_Specification (design
 * D2, D7 to D14). Each table is an X-macro, so the enum, the name or text
 * lookup and the compile-time checks are all generated from one list:
 *
 *   WIRE_EVENT_TABLE(X)   X(code, NAME, arg_bits_max, b_variable)   D10
 *   WIRE_REASON_TABLE(X)  X(code, NAME, "Baseline_Build ack text")  D11
 *   WIRE_CMD_TABLE(X)     X(code, NAME, "cmd echo", cmd_len, ack_len) D11, D12
 *   WIRE_STOP_TABLE(X)    X(code, NAME, "Baseline_Build reason")    D2
 *   WIRE_ABORT_TABLE(X)   X(code, NAME, "Baseline_Build reason")    D10
 *
 * Codes are numbered 0, 1, 2, ... in table order (checked below), so no
 * numeric value is used twice within one code category (Requirement 16.3).
 * The only code outside a table is WCMD_UNPARSED = 15.
 *
 * Changing a code value, a text's meaning or a frame length is a layout
 * change: WIRE_FORMAT_VERSION must then go up by exactly 1 (Requirement 16.4).
 *
 * Plain C header (C standard headers only): used by the task code, the
 * Pico SDK driver code and the Host_Test_Suite (Requirement 9.5).
 */
#ifndef WIRE_IDS_H
#define WIRE_IDS_H

#include <stdint.h>

/* No <stddef.h>: micro T-Kernel's headers define size_t themselves and the
   task code includes this header after them. */
#ifndef NULL
#define NULL ((void *)0)
#endif

/* ---- Format_Version and header field widths (D7) ------------------------ */

/* Byte 0 bits 7..4 of all seven payload types (Requirement 16.1).
   Versions 4 and 5 are never assigned: Baseline_Build text commands start
   with 'A'..'Z' (0x41..0x5A) and must always read as an unknown version. */
#define WIRE_FORMAT_VERSION     (1u)
#define WIRE_VER_BITS           (4u)
#define WIRE_TIME_BITS          (24u)    /* robot time, M24 ms */
#define WIRE_EVT_CODE_BITS      (6u)     /* Event_Code E6 */
#define WIRE_CMD_CODE_BITS      (4u)     /* command code E4 */
#define WIRE_REASON_BITS        (5u)     /* Reason_Code E5 */
#define WIRE_STOP_REASON_BITS   (4u)     /* stop reason E4 (RS4) */
#define WIRE_ABORT_BITS         (4u)     /* abort reason E4 (AB4) */

/* Bytes needed for a bit count; padding bits are 0 (D7). */
#define WIRE_BYTES(bits)        (((bits) + 7u) / 8u)

/* ---- fixed frame lengths (D8, D9, D13, D14) ------------------------------ */

#define WIRE_TLM_BITS           (432u)   /* Telemetry_Frame, 0 padding bits */
#define WIRE_TLM_LEN            (54u)
#define WIRE_HB_BITS            (195u)   /* Heartbeat_Frame, 5 padding bits */
#define WIRE_HB_LEN             (25u)
#define WIRE_MAP_BODY_BITS      (24u)    /* 4 x (present F1, letter U5) */
#define WIRE_MAP_BITS           (WIRE_VER_BITS + WIRE_MAP_BODY_BITS)
#define WIRE_MAP_LEN            (4u)

/* Config_Frame: version U4, count U6, count x (Parameter_ID U6, value F32) */
#define WIRE_CFG_HDR_BITS       (10u)
#define WIRE_CFG_PAIR_BITS      (38u)
#define WIRE_CFG_LEN(n)         WIRE_BYTES(WIRE_CFG_HDR_BITS + (WIRE_CFG_PAIR_BITS * (uint32_t)(n)))

/* ---- Event_Frame (D10) --------------------------------------------------- */

/* Header: version U4, robot time M24, Event_Code E6; arguments from bit 34. */
#define WIRE_EVT_HDR_BITS       (34u)
#define WIRE_EVT_LEN(arg_bits)  WIRE_BYTES(WIRE_EVT_HDR_BITS + (uint32_t)(arg_bits))
/* Argument capacity of an Inter_Task_Message event body (EVT_ARG_BYTES = 26). */
#define WIRE_EVT_ARG_CAP_BITS   (208u)
#define WIRE_EVT_MAX_LEN        (30u)    /* 16-width BC_DBG part: 34 + 203 bits */

/* Array limits of the two variable-length details (Requirements 8.9, 12.8). */
#define WIRE_FINE_SCAN_MAX      (11u)    /* readings per FINE_SCAN event, count U4 1..11 */
#define WIRE_BC_WIDTHS_MAX      (16u)    /* widths per BC_DBG part, count U5 0..16 */
#define WIRE_BC_PARTS_MAX       (6u)     /* BC_DBG width parts per window, part U3 1..6 */

/* Largest argument bit counts of the variable events:
   FINE_SCAN: first S7, count U4, count x CM9
   BC_DBG v4: variant U3, part U3, count U5, count x U12 */
#define WIRE_FINE_SCAN_ARG_MAX  (7u + 4u + (9u * WIRE_FINE_SCAN_MAX))
#define WIRE_BC_DBG_ARG_MAX     (3u + 3u + 5u + (12u * WIRE_BC_WIDTHS_MAX))

/* X(code, NAME, arg_bits_max, b_variable). NAME is the Baseline_Build
   post_event name; arg_bits_max is the fixed argument bit count, or the
   largest one for the two variable events (b_variable = 1). */
#define WIRE_EVENT_TABLE(X) \
    X( 0, STATE,             3u, 0) \
    X( 1, IMU_OK,            9u, 0) \
    X( 2, FAULT_IMU_I2C,     0u, 0) \
    X( 3, FAULT_IMU,         0u, 0) \
    X( 4, HUMP_START,       29u, 0) \
    X( 5, HUMP_PEAK,        29u, 0) \
    X( 6, HUMP_END,         29u, 0) \
    X( 7, MAP,              78u, 0) \
    X( 8, BATTERY_LOW,       0u, 0) \
    X( 9, START_REFUSED,     0u, 0) \
    X(10, MAP_WARNING,      78u, 0) \
    X(11, MISSION_COMPLETE, 59u, 0) \
    X(12, TURN_SNAP,         1u, 0) \
    X(13, NAV_ACTION,        8u, 0) \
    X(14, NAV,              10u, 0) \
    X(15, EMERGENCY_STOP,    4u, 0) \
    X(16, ERROR,             4u, 0) \
    X(17, RECOVERED,         0u, 0) \
    X(18, RECOVERY_REFUSED, 15u, 0) \
    X(19, IMPACT,            0u, 0) \
    X(20, BC_DBG,           WIRE_BC_DBG_ARG_MAX, 1) \
    X(21, BARCODE,           7u, 0) \
    X(22, BARCODE_UNKNOWN,   7u, 0) \
    X(23, BARCODE_FAULT,     0u, 0) \
    X(24, BARCODE_FAILED,    0u, 0) \
    X(25, LINE_FOUND,        0u, 0) \
    X(26, LINE_LOST,         0u, 0) \
    X(27, LINE_SEARCH,       1u, 0) \
    X(28, FINE_SCAN,        WIRE_FINE_SCAN_ARG_MAX, 1) \
    X(29, SIDE_FOLLOW,      64u, 0) \
    X(30, SIDE_LOOK,        27u, 0) \
    X(31, BYPASS_EXTEND,    10u, 0) \
    X(32, BYPASS_SIDESTEP,  10u, 0) \
    X(33, BYPASS_RETURN,     0u, 0) \
    X(34, OBSTACLE_PROFILE, 51u, 0) \
    X(35, AVOIDANCE_ABORT,   4u, 0) \
    X(36, OBST_REVERSE_CAP, 44u, 0) \
    X(37, OBSTACLE_BACKOFF,  9u, 0) \
    X(38, OBSTACLE_RESCAN,   1u, 0) \
    X(39, OBSTACLE_TIE,     20u, 0) \
    X(40, OBSTACLE_CLEARED,  0u, 0) \
    X(41, BYPASS_COMPLETE,   2u, 0) \
    X(42, BYPASS_BLOCKED,   19u, 0) \
    X(43, BYPASS_RETRY,      2u, 0) \
    X(44, OBSTACLE,         29u, 0)

#define WIRE_X_EVC_ENUM(c, n, b, v)  EVC_##n = (c),
typedef enum
{
    WIRE_EVENT_TABLE(WIRE_X_EVC_ENUM)
    EVC_COUNT                                   /* 45 */
} evt_code_t;
#undef WIRE_X_EVC_ENUM

/* ---- Ack_Frame Reason_Codes (D11) ---------------------------------------- */

/* Header: version U4, robot time M24, command code E4, Reason_Code E5. */
#define WIRE_ACK_HDR_BITS       (37u)

/* X(code, NAME, text). The text is the Baseline_Build ack result, except the
   codes marked (new). RSN_CONFIG_REJECTED has no fixed text: the Laptop_Codec
   lists the names of the rejected Parameter_IDs, space separated. */
#define WIRE_REASON_TABLE(X) \
    X( 0, OK,                "ok") \
    X( 1, QUEUED,            "queued") \
    X( 2, STARTING,          "starting") \
    X( 3, ENDING_RUN,        "ending the run") \
    X( 4, PONG,              "pong") \
    X( 5, CLEARED,           "cleared") \
    X( 6, OK_NOT_4,          "ok (not 4 letters)") \
    X( 7, MOTORS_OFF,        "motors off (press BTN1 on the robot to recover)") \
    X( 8, RESUME_REFUSED,    "refused: clear the hazard and press BTN1 on the robot") \
    X( 9, BAD_LETTER,        "nack: use BARCODE:<A-Z>") \
    X(10, ONLY_LINE_FOLLOW,  "nack: only while LINE_FOLLOWING") \
    X(11, ONLY_FROM_IDLE,    "nack: only from IDLE") \
    X(12, UNKNOWN_COMMAND,   "nack: unknown command") \
    X(13, UNKNOWN_PARAM,     "nack: unknown parameter") \
    X(14, BAD_NUMBER,        "nack: bad number") \
    X(15, OUT_OF_RANGE,      "nack: out of range") \
    X(16, BAD_VERSION,       "nack: unknown format version")                 /* (new) */ \
    X(17, BAD_LENGTH,        "nack: bad length")                             /* (new) */ \
    X(18, MAP_LETTERS,       "nack: letters must be A-Z") \
    X(19, MAP_MOVEMENT,      "nack: unknown movement (use FWD, LEFT, RIGHT, UTURN)") \
    X(20, MAP_POS_COUNT,     "nack: positional form takes exactly 4 letters") \
    X(21, MAP_SYNTAX,        "nack: expected L=ACTION or a single letter") \
    X(22, MAP_MIXED,         "nack: do not mix the two forms") \
    X(23, MAP_POS_4,         "nack: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT") \
    X(24, MAP_EMPTY,         "nack: empty map") \
    X(25, MAP_DUP_LETTER,    "nack: the same letter was given twice") \
    X(26, CONFIG_REJECTED,   "") \
    X(27, BAD_COUNT,         "nack: count larger than the parameter table")  /* (new) */

#define WIRE_X_RSN_ENUM(c, n, t)  RSN_##n = (c),
typedef enum
{
    WIRE_REASON_TABLE(WIRE_X_RSN_ENUM)
    RSN_COUNT                                   /* 28 */
} wire_reason_t;
#undef WIRE_X_RSN_ENUM

/* ---- command codes (D11, D12) -------------------------------------------- */

/* Command_Frame byte 0: Format_Version U4, command code E4. */
#define WIRE_CMD_HDR_BITS       (8u)

/* X(code, NAME, echo, cmd_len, ack_len).
   echo:    the Baseline_Build "cmd" text the Laptop_Codec rebuilds (PING,
            BARCODE and SET add ":<number>", ":<letter>", ":<NAME>").
   cmd_len: Command_Frame length in bytes on is02/cmd; 0 = not accepted on
            is02/cmd (CONFIG is the Ack_Frame code of a Config_Frame).
   ack_len: Ack_Frame length in bytes; 0 = variable (GET:PARAMS parts, CONFIG).
   Codes 0..10 are the 11 Baseline_Build commands (Requirement 13.2). */
#define WIRE_CMD_TABLE(X) \
    X( 0, ESTOP,      "ESTOP",      1u,  7u) \
    X( 1, RESUME,     "RESUME",     1u,  5u) \
    X( 2, START,      "START",      1u,  5u) \
    X( 3, STOP,       "STOP",       1u,  5u) \
    X( 4, BARCODE,    "BARCODE",    2u,  6u) \
    X( 5, MAP,        "MAP",        4u, 15u) \
    X( 6, MAP_CLEAR,  "MAP:CLEAR",  1u, 15u) \
    X( 7, GET_MAP,    "GET:MAP",    1u, 15u) \
    X( 8, PING,       "PING",       3u,  7u) \
    X( 9, SET,        "SET",        6u,  6u) \
    X(10, GET_PARAMS, "GET:PARAMS", 1u,  0u) \
    X(11, CONFIG,     "CONFIG",     0u,  0u)

#define WIRE_X_CMD_ENUM(c, n, e, lc, la)  WCMD_##n = (c),
typedef enum
{
    WIRE_CMD_TABLE(WIRE_X_CMD_ENUM)
    WCMD_TABLE_COUNT,                           /* 12: codes 0..11 */
    WCMD_UNPARSED = 15                          /* ack of a rejected frame, no arguments */
} wire_cmd_code_t;
#undef WIRE_X_CMD_ENUM

#define WIRE_CMD_FRAME_COUNT    (11u)    /* codes 0..10 accepted on is02/cmd */
#define WIRE_ACK_UNPARSED_LEN   (5u)

/* GET:PARAMS reply parts: part U3 1..5, count U4 0..12, count x (ID U6,
   value: F32 for float parameters, 1/2/3-bit code for packed ones). */
#define WIRE_GETP_PER_PART      (12u)
#define WIRE_GETP_MAX_LEN       WIRE_BYTES(WIRE_ACK_HDR_BITS + 3u + 4u + (WIRE_GETP_PER_PART * (6u + 32u)))
/* CONFIG ack: applied U6, rejected count U6, rejected IDs U6 each (0..60). */
#define WIRE_CFG_ACK_LEN(n_rej) WIRE_BYTES(WIRE_ACK_HDR_BITS + 6u + 6u + (6u * (uint32_t)(n_rej)))

/* ---- stop reasons (D2, Requirement 2.9) ---------------------------------- */

/* X(code, NAME, text). The text is the Baseline_Build g_stop_reason string. */
#define WIRE_STOP_TABLE(X) \
    X(0, NONE,            "") \
    X(1, REMOTE_ESTOP,    "remote E-stop (MQTT)") \
    X(2, BTN2,            "BTN2 stop") \
    X(3, COLLISION,       "collision (IMU)") \
    X(4, OBSTACLE,        "obstacle closer than 8 cm") \
    X(5, WATCHDOG,        "watchdog: a control task stopped running") \
    X(6, CALIBRATION,     "calibration missing") \
    X(7, IMU_FAULT,       "IMU fault (3 failed I2C recoveries)") \
    X(8, BATTERY_CRIT,    "battery critical") \
    X(9, FAULT_REPORTED,  "fault reported (see faults in telemetry)")

#define WIRE_X_SR_ENUM(c, n, t)  SR_##n = (c),
typedef enum
{
    WIRE_STOP_TABLE(WIRE_X_SR_ENUM)
    STOP_REASON_COUNT                           /* 10; codes 10..15 never stored */
} stop_reason_t;
#undef WIRE_X_SR_ENUM

/* ---- avoidance abort reasons (D10 AB4) ----------------------------------- */

/* X(code, NAME, text). The text is the Baseline_Build abort_avoidance() reason. */
#define WIRE_ABORT_TABLE(X) \
    X( 0, LEG_FAILED,      "bypass leg failed (timeout, E-stop or proximity)") \
    X( 1, REALIGN,         "realign failed") \
    X( 2, REVERSE_PATH,    "reverse onto the original path failed") \
    X( 3, REVERSE_LIMIT,   "reverse limit reached") \
    X( 4, REVERSE_FAILED,  "reverse failed (E-stop, proximity or timeout)") \
    X( 5, PROFILE_FAILED,  "profile failed (no echo in the fine scan)") \
    X( 6, NO_ROOM,         "no room on either side") \
    X( 7, NO_PLAN,         "no bypass plan") \
    X( 8, BACKOUT_FAILED,  "backing out failed (E-stop, proximity or timeout)") \
    X( 9, BLOCKED_BOTH,    "blocked on both sides") \
    X(10, TOO_MANY,        "too many attempts")

#define WIRE_X_ABR_ENUM(c, n, t)  ABR_##n = (c),
typedef enum
{
    WIRE_ABORT_TABLE(WIRE_X_ABR_ENUM)
    ABORT_REASON_COUNT                          /* 11 */
} abort_reason_t;
#undef WIRE_X_ABR_ENUM

/* ---- lookups (NULL for a code no table lists) ---------------------------- */

#define WIRE_X_CASE_NAME(c, n, b, v)      case (c): return #n;
#define WIRE_X_CASE_ABITS(c, n, b, v)     case (c): return (b);
#define WIRE_X_CASE_TEXT(c, n, t)         case (c): return (t);
#define WIRE_X_CASE_ECHO(c, n, e, lc, la) case (c): return (e);
#define WIRE_X_CASE_CLEN(c, n, e, lc, la) case (c): return (lc);
#define WIRE_X_CASE_ALEN(c, n, e, lc, la) case (c): return (la);

/* Baseline_Build event name, for example "EMERGENCY_STOP". */
static inline const char *wire_evt_name(uint32_t code)
{
    switch (code)
    {
        WIRE_EVENT_TABLE(WIRE_X_CASE_NAME)
        default: return NULL;
    }
}

/* Fixed argument bits, or the largest for BC_DBG and FINE_SCAN; 0 if unknown. */
static inline uint32_t wire_evt_arg_bits_max(uint32_t code)
{
    switch (code)
    {
        WIRE_EVENT_TABLE(WIRE_X_CASE_ABITS)
        default: return 0u;
    }
}

static inline const char *wire_reason_text(uint32_t code)
{
    switch (code)
    {
        WIRE_REASON_TABLE(WIRE_X_CASE_TEXT)
        default: return NULL;
    }
}

/* Baseline_Build "cmd" echo text; "" for WCMD_UNPARSED. */
static inline const char *wire_cmd_name(uint32_t code)
{
    switch (code)
    {
        WIRE_CMD_TABLE(WIRE_X_CASE_ECHO)
        case WCMD_UNPARSED: return "";
        default: return NULL;
    }
}

/* Command_Frame length on is02/cmd; 0 = code not accepted on is02/cmd. */
static inline uint32_t wire_cmd_len(uint32_t code)
{
    switch (code)
    {
        WIRE_CMD_TABLE(WIRE_X_CASE_CLEN)
        default: return 0u;
    }
}

/* Ack_Frame length; 0 = variable length (GET:PARAMS, CONFIG) or unknown code. */
static inline uint32_t wire_ack_len(uint32_t code)
{
    switch (code)
    {
        WIRE_CMD_TABLE(WIRE_X_CASE_ALEN)
        case WCMD_UNPARSED: return WIRE_ACK_UNPARSED_LEN;
        default: return 0u;
    }
}

/* Baseline_Build g_stop_reason text ("" for SR_NONE). */
static inline const char *wire_stop_reason_text(uint32_t code)
{
    switch (code)
    {
        WIRE_STOP_TABLE(WIRE_X_CASE_TEXT)
        default: return NULL;
    }
}

static inline const char *wire_abort_text(uint32_t code)
{
    switch (code)
    {
        WIRE_ABORT_TABLE(WIRE_X_CASE_TEXT)
        default: return NULL;
    }
}

#undef WIRE_X_CASE_NAME
#undef WIRE_X_CASE_ABITS
#undef WIRE_X_CASE_TEXT
#undef WIRE_X_CASE_ECHO
#undef WIRE_X_CASE_CLEN
#undef WIRE_X_CASE_ALEN

/* ---- compile-time checks -------------------------------------------------- */

/* Each code is its position in its table: 0, 1, 2, ... with no gap and no
   value used twice (Requirement 16.3). */
/* Position enums: sequential enumerators, one per table row. */
#define WIRE_X_PEV(c, n, b, v)          WIRE_PEV_##n,
#define WIRE_X_PRS(c, n, t)             WIRE_PRS_##n,
#define WIRE_X_PCM(c, n, e, lc, la)     WIRE_PCM_##n,
#define WIRE_X_PSR(c, n, t)             WIRE_PSR_##n,
#define WIRE_X_PAB(c, n, t)             WIRE_PAB_##n,
enum { WIRE_EVENT_TABLE(WIRE_X_PEV)  WIRE_PEV_END_ };
enum { WIRE_REASON_TABLE(WIRE_X_PRS) WIRE_PRS_END_ };
enum { WIRE_CMD_TABLE(WIRE_X_PCM)    WIRE_PCM_END_ };
enum { WIRE_STOP_TABLE(WIRE_X_PSR)   WIRE_PSR_END_ };
enum { WIRE_ABORT_TABLE(WIRE_X_PAB)  WIRE_PAB_END_ };
#undef WIRE_X_PEV
#undef WIRE_X_PRS
#undef WIRE_X_PCM
#undef WIRE_X_PSR
#undef WIRE_X_PAB

#define WIRE_X_CHK_EV(c, n, b, v) \
    _Static_assert((c) == WIRE_PEV_##n, "wire_ids: Event_Code " #n " is not its table position"); \
    _Static_assert((b) <= WIRE_EVT_ARG_CAP_BITS, "wire_ids: Event_Code " #n " arguments too long"); \
    _Static_assert(((v) == 0) || ((v) == 1), "wire_ids: Event_Code " #n " b_variable is not 0 or 1");
#define WIRE_X_CHK_RS(c, n, t) \
    _Static_assert((c) == WIRE_PRS_##n, "wire_ids: Reason_Code " #n " is not its table position");
#define WIRE_X_CHK_CM(c, n, e, lc, la) \
    _Static_assert((c) == WIRE_PCM_##n, "wire_ids: command code " #n " is not its table position");
#define WIRE_X_CHK_SR(c, n, t) \
    _Static_assert((c) == WIRE_PSR_##n, "wire_ids: stop reason " #n " is not its table position");
#define WIRE_X_CHK_AB(c, n, t) \
    _Static_assert((c) == WIRE_PAB_##n, "wire_ids: abort reason " #n " is not its table position");
WIRE_EVENT_TABLE(WIRE_X_CHK_EV)
WIRE_REASON_TABLE(WIRE_X_CHK_RS)
WIRE_CMD_TABLE(WIRE_X_CHK_CM)
WIRE_STOP_TABLE(WIRE_X_CHK_SR)
WIRE_ABORT_TABLE(WIRE_X_CHK_AB)
#undef WIRE_X_CHK_EV
#undef WIRE_X_CHK_RS
#undef WIRE_X_CHK_CM
#undef WIRE_X_CHK_SR
#undef WIRE_X_CHK_AB

/* Format_Version fits its 4-bit field and never reads as text (D7). */
_Static_assert((WIRE_FORMAT_VERSION >= 1u) && (WIRE_FORMAT_VERSION < (1u << WIRE_VER_BITS)),
               "wire_ids: Format_Version does not fit U4");
_Static_assert((WIRE_FORMAT_VERSION != 4u) && (WIRE_FORMAT_VERSION != 5u),
               "wire_ids: Format_Version 4 and 5 are reserved (text commands)");

/* Table counts (Requirements 2.9, 8.2, 12.4, 13.2). */
_Static_assert(EVC_COUNT == 45, "wire_ids: 45 Event_Codes expected");
_Static_assert(RSN_COUNT == 28, "wire_ids: 28 Reason_Codes expected");
_Static_assert(WCMD_TABLE_COUNT == 12, "wire_ids: command codes 0..11 expected");
_Static_assert(WIRE_CMD_FRAME_COUNT == (uint32_t)WCMD_CONFIG, "wire_ids: 11 is02/cmd command codes expected");
_Static_assert(STOP_REASON_COUNT == 10, "wire_ids: 10 stop reasons expected");
_Static_assert(ABORT_REASON_COUNT == 11, "wire_ids: 11 abort reasons expected");

/* Each table fits its field width (Requirement 2.13). */
_Static_assert((uint32_t)EVC_COUNT <= (1u << WIRE_EVT_CODE_BITS), "wire_ids: Event_Code E6 too narrow");
_Static_assert((uint32_t)RSN_COUNT <= (1u << WIRE_REASON_BITS), "wire_ids: Reason_Code E5 too narrow");
_Static_assert((uint32_t)WCMD_TABLE_COUNT <= (uint32_t)WCMD_UNPARSED, "wire_ids: command codes reach UNPARSED");
_Static_assert((uint32_t)WCMD_UNPARSED < (1u << WIRE_CMD_CODE_BITS), "wire_ids: command code E4 too narrow");
_Static_assert((uint32_t)STOP_REASON_COUNT <= (1u << WIRE_STOP_REASON_BITS), "wire_ids: stop reason E4 too narrow");
_Static_assert((uint32_t)ABORT_REASON_COUNT <= (1u << WIRE_ABORT_BITS), "wire_ids: abort reason E4 too narrow");

/* Header bit counts (D7, D10, D11, D12). */
_Static_assert(WIRE_EVT_HDR_BITS == WIRE_VER_BITS + WIRE_TIME_BITS + WIRE_EVT_CODE_BITS,
               "wire_ids: Event_Frame header");
_Static_assert(WIRE_ACK_HDR_BITS == WIRE_VER_BITS + WIRE_TIME_BITS + WIRE_CMD_CODE_BITS + WIRE_REASON_BITS,
               "wire_ids: Ack_Frame header");
_Static_assert(WIRE_CMD_HDR_BITS == WIRE_VER_BITS + WIRE_CMD_CODE_BITS, "wire_ids: Command_Frame header");

/* Fixed frame lengths (Requirements 11.5, 12.7). */
_Static_assert(WIRE_TLM_LEN == WIRE_BYTES(WIRE_TLM_BITS), "wire_ids: WIRE_TLM_LEN");
_Static_assert(WIRE_HB_LEN == WIRE_BYTES(WIRE_HB_BITS), "wire_ids: WIRE_HB_LEN");
_Static_assert(WIRE_MAP_LEN == WIRE_BYTES(WIRE_MAP_BITS), "wire_ids: WIRE_MAP_LEN");
_Static_assert(WIRE_CFG_LEN(0) == 2u, "wire_ids: empty Config_Frame length");
_Static_assert(WIRE_CFG_LEN(55) == 263u, "wire_ids: 55-pair Config_Frame length");

/* Event_Frame lengths: the variable maxima match D10. */
_Static_assert(WIRE_FINE_SCAN_MAX < (1u << 4), "wire_ids: FINE_SCAN count U4 too narrow");
_Static_assert(WIRE_BC_WIDTHS_MAX < (1u << 5), "wire_ids: BC_DBG width count U5 too narrow");
_Static_assert(WIRE_BC_PARTS_MAX < (1u << 3), "wire_ids: BC_DBG part U3 too narrow");
_Static_assert(WIRE_BC_DBG_ARG_MAX == 203u, "wire_ids: BC_DBG longest arguments");
_Static_assert(WIRE_FINE_SCAN_ARG_MAX == 110u, "wire_ids: FINE_SCAN longest arguments");
_Static_assert(WIRE_EVT_LEN(WIRE_BC_DBG_ARG_MAX) == WIRE_EVT_MAX_LEN, "wire_ids: longest Event_Frame");

/* Command_Frame and Ack_Frame lengths per code (D11, D12): header plus
   argument bits, rounded up to whole bytes. */
#define WIRE_X_LEN_ENUM(c, n, e, lc, la)  WIRE_CMD_LEN_##n = (lc), WIRE_ACK_LEN_##n = (la),
enum { WIRE_CMD_TABLE(WIRE_X_LEN_ENUM) WIRE_LEN_END_ };
#undef WIRE_X_LEN_ENUM

#define WIRE_CMD_LEN_OF(arg_bits)  WIRE_BYTES(WIRE_CMD_HDR_BITS + (arg_bits))
#define WIRE_ACK_LEN_OF(arg_bits)  WIRE_BYTES(WIRE_ACK_HDR_BITS + (arg_bits))
_Static_assert(WIRE_CMD_LEN_ESTOP      == WIRE_CMD_LEN_OF(0u),  "wire_ids: ESTOP frame length");
_Static_assert(WIRE_CMD_LEN_RESUME     == WIRE_CMD_LEN_OF(0u),  "wire_ids: RESUME frame length");
_Static_assert(WIRE_CMD_LEN_START      == WIRE_CMD_LEN_OF(0u),  "wire_ids: START frame length");
_Static_assert(WIRE_CMD_LEN_STOP       == WIRE_CMD_LEN_OF(0u),  "wire_ids: STOP frame length");
_Static_assert(WIRE_CMD_LEN_BARCODE    == WIRE_CMD_LEN_OF(5u + 3u), "wire_ids: BARCODE frame length");
_Static_assert(WIRE_CMD_LEN_MAP        == WIRE_CMD_LEN_OF(WIRE_MAP_BODY_BITS), "wire_ids: MAP frame length");
_Static_assert(WIRE_CMD_LEN_MAP_CLEAR  == WIRE_CMD_LEN_OF(0u),  "wire_ids: MAP:CLEAR frame length");
_Static_assert(WIRE_CMD_LEN_GET_MAP    == WIRE_CMD_LEN_OF(0u),  "wire_ids: GET:MAP frame length");
_Static_assert(WIRE_CMD_LEN_PING       == WIRE_CMD_LEN_OF(16u), "wire_ids: PING frame length");
_Static_assert(WIRE_CMD_LEN_SET        == WIRE_CMD_LEN_OF(6u + 32u + 2u), "wire_ids: SET frame length");
_Static_assert(WIRE_CMD_LEN_GET_PARAMS == WIRE_CMD_LEN_OF(0u),  "wire_ids: GET:PARAMS frame length");
_Static_assert(WIRE_CMD_LEN_CONFIG     == 0,                    "wire_ids: CONFIG is not a Command_Frame");

_Static_assert(WIRE_ACK_LEN_ESTOP      == WIRE_ACK_LEN_OF(16u), "wire_ids: ESTOP ack length");
_Static_assert(WIRE_ACK_LEN_RESUME     == WIRE_ACK_LEN_OF(0u),  "wire_ids: RESUME ack length");
_Static_assert(WIRE_ACK_LEN_START      == WIRE_ACK_LEN_OF(0u),  "wire_ids: START ack length");
_Static_assert(WIRE_ACK_LEN_STOP       == WIRE_ACK_LEN_OF(0u),  "wire_ids: STOP ack length");
_Static_assert(WIRE_ACK_LEN_BARCODE    == WIRE_ACK_LEN_OF(5u),  "wire_ids: BARCODE ack length");
_Static_assert(WIRE_ACK_LEN_MAP        == WIRE_ACK_LEN_OF(78u + 5u), "wire_ids: MAP ack length");
_Static_assert(WIRE_ACK_LEN_MAP_CLEAR  == WIRE_ACK_LEN_OF(78u + 5u), "wire_ids: MAP:CLEAR ack length");
_Static_assert(WIRE_ACK_LEN_GET_MAP    == WIRE_ACK_LEN_OF(78u + 5u), "wire_ids: GET:MAP ack length");
_Static_assert(WIRE_ACK_LEN_PING       == WIRE_ACK_LEN_OF(16u), "wire_ids: PING ack length");
_Static_assert(WIRE_ACK_LEN_SET        == WIRE_ACK_LEN_OF(6u),  "wire_ids: SET ack length");
_Static_assert(WIRE_ACK_LEN_GET_PARAMS == 0,                    "wire_ids: GET:PARAMS ack is variable");
_Static_assert(WIRE_ACK_LEN_CONFIG     == 0,                    "wire_ids: CONFIG ack is variable");
_Static_assert(WIRE_ACK_UNPARSED_LEN   == WIRE_ACK_LEN_OF(0u),  "wire_ids: UNPARSED ack length");
_Static_assert(WIRE_GETP_MAX_LEN == 63u, "wire_ids: GET:PARAMS part longer than 63 bytes");
_Static_assert(WIRE_GETP_PER_PART < (1u << 4), "wire_ids: GET:PARAMS count U4 too narrow");
_Static_assert(WIRE_CFG_ACK_LEN(55) == 48u, "wire_ids: CONFIG ack with 55 rejections");
#undef WIRE_CMD_LEN_OF
#undef WIRE_ACK_LEN_OF

#endif /* WIRE_IDS_H */
