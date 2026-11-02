/**
 * @file    evt.h
 * @brief   Event_Frame builder: the binary replacement of post_event(name, text).
 *
 * A task builds the event's arguments as packed bit fields, in the order and
 * widths listed below, then posts it to MBX_TELEMETRY. TSK_TELEMETRY adds the
 * robot time and publishes it on is02/event (wire.c). The laptop codecs turn
 * each event back into the Baseline_Build name and text.
 *
 * Argument layouts = design D10 (Event_Code: fields in order, widths in bits).
 * Shorthands: CM9 = U9 whole cm 0..431, 511 = no echo; CM13 = U13 0.1 cm
 * 0..431.0, 8191 = no echo; C6 = Code 39 index U6 0..42, 63 = none;
 * L5 = letter U5 (0 = 'A'); ACT3 = nav_action_t E3; RES2 = window result E2.
 *   STATE            state E3                                             (3)
 *   IMU_OK           chip E2 (0 none, 1 LSM303DLHC, 2 LSM303D), addr U7   (9)
 *   HUMP_START/PEAK/END  valid F1, height U14 (0.1 mm), run max U14 (0.1 mm) (29)
 *   MAP, MAP_WARNING 26 x ACT3, letter A first                           (78)
 *   MISSION_COMPLETE time U17 (s), dist U14 (0.01 m), barcodes U6, humps U4,
 *                    hump max U14 (0.1 mm), obstacles U4                 (59)
 *   TURN_SNAP        not_seen F1 (0 "stopped on the line")                (1)
 *   NAV_ACTION       L5, ACT3                                             (8)
 *   NAV              variant U2 (evt_nav_kind_t), L5, ACT3               (10)
 *   EMERGENCY_STOP, ERROR  stop reason E4                                 (4)
 *   RECOVERY_REFUSED why U15: bit 0 obstacle < 8 cm, bit 1 INT1 active,
 *                    bits 2..14 missing calibration items                (15)
 *   BC_DBG           variant U3, then:
 *                      0 off: nothing
 *                      1 window: n U7, RES2, C6, rev F1, thr_b U12, thr_w U12, ovr U16
 *                      2 pass: the window fields + level U3, conflict F1
 *                      3 levels: n_levels U3, always 6 x (RES2, C6, n U7)
 *                      4 widths: part U3 1..6, count U5 0..16, count x U12 (0.1 mm)
 *   BARCODE, BARCODE_UNKNOWN  C6, rev F1                                  (7)
 *   LINE_SEARCH      right F1                                             (1)
 *   FINE_SCAN        first S7 (deg), count U4 1..11, count x CM9  (11 + 9n)
 *   SIDE_FOLLOW      n60 U8, seen U8, near CM13, limit U13 (0.1 cm),
 *                    leg U11 (mm), x U11 (mm)                            (64)
 *   SIDE_LOOK        d90 CM13, d60 CM13, gone F1                         (27)
 *   BYPASS_EXTEND    variant F1, CM9                                     (10)
 *   BYPASS_SIDESTEP  mm U10                                              (10)
 *   OBSTACLE_PROFILE r CM9, w U10 (cm), off S10 (cm), clrL S10 (cm),
 *                    clrR S10 (cm), side E2                              (51)
 *   AVOIDANCE_ABORT  abort reason E4                                      (4)
 *   OBST_REVERSE_CAP variant U2, n U10 (mm), used U10 (mm), cap U10 (mm),
 *                    total U12 (mm)                                      (44)
 *   OBSTACLE_BACKOFF need U8 (mm), right F1                               (9)
 *   OBSTACLE_RESCAN  blocked F1                                           (1)
 *   OBSTACLE_TIE     dl U9 (cm), dr U9 (cm), side E2                     (20)
 *   BYPASS_COMPLETE  right F1, search F1                                  (2)
 *   BYPASS_BLOCKED   right F1, no_end F1, CM9, deg S8                    (19)
 *   BYPASS_RETRY     right F1, wider F1                                   (2)
 *   OBSTACLE         cm U13 (0.1 cm), latency U16 (us, saturates)        (29)
 *   others           no arguments
 * The authoritative widths are WIRE_EVENT_TABLE (wire_ids.h): evt_post()
 * refuses an event whose bit count does not match.
 */
#ifndef EVT_H
#define EVT_H

#include <stdbool.h>
#include <stdint.h>

#include "wire.h"

typedef struct
{
    uint32_t   t_ms;          /* robot time when the event happened */
    wire_evt_t e;
} evt_msg_t;

typedef struct
{
    evt_msg_t m;
    bw_t      w;
} evt_b_t;

/* NAV kinds */
typedef enum
{
    NAVK_DONE = 0,            /* turn finished: action */
    NAVK_FAILED,              /* turn did not complete */
    NAVK_STORED,              /* letter stored for the next junction */
    NAVK_BC_FAULT             /* 3 ambiguous barcodes: going straight */
} evt_nav_kind_t;

#define EVT_CM13_NONE  (8191u)
#define EVT_CM9_NONE   (511u)

void evt_begin(evt_b_t *p_b, evt_code_t code);
void evt_u(evt_b_t *p_b, uint32_t v, uint32_t width);   /* saturates at 2^width - 1 */
void evt_s(evt_b_t *p_b, int32_t v, uint32_t width);    /* saturates to the signed range */
void evt_cm13(evt_b_t *p_b, float cm);                  /* < 0 = no echo */
void evt_cm9(evt_b_t *p_b, float cm);                   /* < 0 = no echo */
bool evt_post(evt_b_t *p_b);                            /* false: wrong bit count or pool empty */
bool evt_post0(evt_code_t code);                        /* event with no arguments */
bool evt_post1(evt_code_t code, uint32_t v, uint32_t width);   /* one unsigned argument */

/* MAP / MAP_WARNING arguments: 26 x E3 from nav_map_actions(). */
void evt_map26(evt_b_t *p_b, const uint8_t p_act[26]);

/* Barcode character as C6: Code 39 index 0..42, 63 for none / not a Code 39 character. */
uint32_t evt_char6(char c);

#endif /* EVT_H */
