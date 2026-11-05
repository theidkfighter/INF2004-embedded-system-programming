/**
 * @file    shared.h
 * @brief   Packed Task_Shared_Data outside the Status_Snapshot (design D3):
 *          buzzer pattern, E-stop flags and times, motion done word, odometry.
 *
 * Plain C header (Requirement 9.5), implemented in app_shared.c, so the host
 * tests run the same code against crit_stub.c (Property 18).
 *
 *   unit                 fields                         writers, protection
 *   g_buzz (8)           pattern E3 BUZZ_NONE..BUZZ_CLIP  any task, safety: dispatch disabled
 *   g_net_flags (8)      estop_pending [0],               radio task, safety, telemetry:
 *                        estop_ack_due [1],               dispatch disabled
 *                        config_due [2]
 *   g_estop_rx_us (32)   M32 receive time, us             radio task, in the section that
 *                                                         sets estop_pending
 *   g_estop_pwm_off_us (16)  U16 us, saturating           TSK_SAFETY_MON, one store
 *   g_done (32)          done_id U16 [0..15],             TSK_MOTION_CTRL, one store in
 *                        done_result E2 [16..17]          one section; readers one load
 *   g_odo_rec (5 x 32)   see below                        TSK_MOTION_CTRL, dispatch disabled
 *
 * Interrupt handlers never call these functions (crit.h).
 */
#ifndef SHARED_H
#define SHARED_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

/* ---- buzzer patterns (EVF_BUZZER_REQ) ---- */
#define BUZZ_NONE        (0u)
#define BUZZ_START       (1u)
#define BUZZ_BARCODE     (2u)
#define BUZZ_COMPLETE    (3u)
#define BUZZ_ERROR       (4u)
#define BUZZ_LOW_BATTERY (5u)
#define BUZZ_WARN        (6u)
#define BUZZ_CLIP        (7u)    /* PLAY: the clip from clip_pcm.h, not a tone pattern */

bool     buzz_put(uint32_t pattern);        /* false, nothing stored, if > BUZZ_CLIP */
uint32_t buzzer_pattern_take(void);         /* read + clear in one section */

/* ---- E-stop receive (consistent set, design D3.5) and config flag ---- */
void     estop_rx_put(uint32_t rx_us);      /* radio task: estop_pending, estop_ack_due, time */
bool     estop_rx_take(uint32_t *p_rx_us);  /* safety: read + clear estop_pending and time */
void     estop_ack_request(void);           /* estop_ack_due only (queued ESTOP) */
bool     estop_ack_take(void);              /* telemetry: true once, after the safety monitor acted */
void     estop_pwm_off_put(uint32_t us);    /* TSK_SAFETY_MON only; holds at 65,535 */
uint16_t estop_pwm_off(void);

void     config_due_set(void);
void     config_due_clear(void);
bool     config_due(void);

/* ---- motion done word (Requirement 4.5) ----
   One 32-bit unit, so an id is never paired with another primitive's result. */
#define DONE_ID(w)       ((uint16_t)((w) & 0xFFFFu))
#define DONE_RESULT(w)   ((motion_result_t)(((w) >> 16) & 3u))

void     motion_done_put(uint16_t id, motion_result_t result);   /* one composed store */
uint32_t motion_done_get(void);                                  /* one load */

/* ---- shared odometry (written by the motion task) ----
   The callers use this float form; the shared copy is a packed 5-word record:
     odo_um U32 (um, monotonic, wraps), dist S28 (0.01 mm, +-1 km),
     heading S25 (0.01 deg, +-100,000 deg), v S18 (0.01 mm/s, +-1000),
     a_fwd S17 (1e-4 g, +-4 g), turn S18 (0.01 deg/s, +-720)  = 138 bits.
   odometry_get() returns the dequantized values; a reader that differences
   two reads (heading, distance) sees no drift, because both values come
   from the same exact accumulators in the motion task. */
typedef struct
{
    float    dist_mm;           /* mean of both wheels, signed, forward + */
    float    heading_deg;       /* from the encoders, + = clockwise */
    float    v_mm_s;            /* mean of both wheels */
    float    a_fwd_g;           /* forward acceleration from the encoder speed */
    float    turn_dps;
    uint32_t odo_um;            /* travelled distance, monotonic, micrometres */
} odometry_t;
void     odometry_get(odometry_t *p_out);
void     odometry_put(const odometry_t *p_in);

#endif /* SHARED_H */
