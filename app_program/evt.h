/**
 * @file    evt.h
 * @brief   Event_Frame builder: the binary replacement of post_event(name, text).
 *
 * A task builds the event's arguments as packed bit fields, in the order and
 * widths listed below, then posts it to MBX_TELEMETRY. TSK_TELEMETRY adds the
 * robot time and publishes it on is02/event (wire.c). The laptop codecs turn
 * each event back into the Baseline_Build name and text.
 *
 * Argument layouts (Event_Code: fields in order; widths in bits):
 *   STATE            state E3
 *   IMU_OK           chip E2 (0 none, 1 LSM303DLHC, 2 LSM303D), address U7
 *   HUMP_START/PEAK/END  valid U1, height U14 (0.1 mm), run max U14 (0.1 mm)
 *   MAP, MAP_WARNING 26 x nav_action_t E3 (letters A..Z)
 *   MISSION_COMPLETE time U12 (s), dist U14 (0.01 m), barcodes U7, humps U6,
 *                    hump max U14 (0.1 mm), obstacles U6
 *   TURN_SNAP        on_line U1
 *   NAV_ACTION       letter U5 (0 = 'A'), action E3
 *   NAV              kind E2 (evt_nav_kind_t), letter U5, action E3
 *   EMERGENCY_STOP, ERROR  stop_reason_t E4
 *   RECOVERY_REFUSED evt_refuse bits U15
 *   BC_DBG           variant U3, then (wire.c):
 *                      1 window: n U7, res E2, code U6, rev U1, thr_b U12, thr_w U12, ovr U16
 *                      2 multi:  the window fields + level U3, conflict U1
 *                      3 levels: count U3, count x (res E2, code U6, n U7)
 *                      4 widths: part U3, count U5, count x width U12 (0.1 mm)
 *   BARCODE, BARCODE_UNKNOWN  code U6 (ASCII - 32), backwards U1
 *   LINE_SEARCH      right_first U1
 *   FINE_SCAN        first angle S7 (deg), count U4 (1..11), count x CM9
 *   SIDE_FOLLOW      d90 CM13, d60 CM13, near U13 (0.1 cm), limit U13 (0.1 cm),
 *                    leg U12 (mm)
 *   SIDE_LOOK        d90 CM13, d60 CM13, b_right U1
 *   BYPASS_EXTEND    value CM9 / mm U9 (evt detail), kind U1
 *   BYPASS_SIDESTEP  mm U10
 *   OBSTACLE_PROFILE r CM9, width U9 (cm), offset S9 (cm), clr_l S8 (cm),
 *                    clr_r S8 (cm), side E2, theta_l S? -> see tsk_obstacle.c
 *   AVOIDANCE_ABORT  abort_reason_t E4
 *   OBST_REVERSE_CAP see tsk_obstacle.c (44 bits)
 *   OBSTACLE_BACKOFF mm U9
 *   OBSTACLE_RESCAN  kind U1
 *   OBSTACLE_TIE     see tsk_obstacle.c (20 bits)
 *   BYPASS_COMPLETE  how E2
 *   BYPASS_BLOCKED   see tsk_obstacle.c (19 bits)
 *   BYPASS_RETRY     kind E2
 *   OBSTACLE         distance CM13, stop latency U16 (us, saturates)
 *   others           no arguments
 * CM13 = ultrasonic distance U13 in 0.1 cm (0..431.0), 8191 = no echo.
 * CM9  = ultrasonic distance U9 in whole cm (0..431), 511 = no echo.
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

/* Barcode character as U6 (ASCII - 32), 0 for none / outside ' '..'_'. */
uint32_t evt_char6(char c);

#endif /* EVT_H */
