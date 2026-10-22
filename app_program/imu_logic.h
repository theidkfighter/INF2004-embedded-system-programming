/**
 * @file    imu_logic.h
 * @brief   Pitch, hump profiler and motion-event classifier (DR 6.3).
 *
 * Region A (hump profiler): FLAT -> ASCENT (theta >= 5 deg for 3 samples)
 * -> CREST (|theta| <= 3 deg with h > 15 mm) -> DESCENT (theta <= -5 deg for
 * 3 samples) -> completion (|theta| <= 3 deg for 5 samples). Height is
 * integrated as h += sin(theta) * ds from the encoder distance.
 * Region B (motion events): IMPACT > CLIMBING/DESCENDING > TURNING >
 * ACCELERATING/DECELERATING > STATIONARY > CRUISING, held 3 samples.
 *
 * Pure logic: unit-tested on the PC (tests/).
 */
#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"

/* Pitch in degrees, nose up = positive.
   axis: 1 = X forward, 2 = Y forward; sign: +1/-1 (test_imu U5).
   a_fwd_g: forward acceleration from the encoders, removed so speed
   changes do not read as tilt (DR Assumption A8). */
float imu_pitch_deg(float ax, float ay, float az, int32_t axis, int32_t sign, float a_fwd_g);

typedef enum
{
    HUMP_FLAT = 0,
    HUMP_ASCENT,
    HUMP_CREST,
    HUMP_DESCENT
} hump_state_t;

typedef struct
{
    float    start_deg;       /* 5   */
    float    level_deg;       /* 3   */
    float    min_height_mm;   /* 15  */
    uint32_t start_samples;   /* 3   */
    uint32_t end_samples;     /* 5   */
    float    max_len_mm;      /* give up on a "hump" longer than this */
} hump_params_t;

typedef struct
{
    hump_state_t state;
    uint32_t     n_up;
    uint32_t     n_down;
    uint32_t     n_level;
    float        h_mm;
    float        peak_mm;
    float        len_mm;
} hump_ctx_t;

typedef struct
{
    bool  b_start;            /* EVF_HUMP_START */
    bool  b_peak;             /* EVF_HUMP_PEAK  */
    bool  b_end;              /* EVF_HUMP_END   */
    bool  b_valid;            /* with b_end: a real hump, height_mm is its peak */
    float height_mm;
    float len_mm;
} hump_out_t;

void       hump_reset(hump_ctx_t *p_h);
hump_out_t hump_step(hump_ctx_t *p_h, const hump_params_t *p_par, float pitch_deg, float ds_mm);

typedef struct
{
    motion_event_t current;
    motion_event_t candidate;
    uint32_t       n_candidate;
    uint32_t       still_ms;
} mev_ctx_t;

void           mev_reset(mev_ctx_t *p_m);
/* Returns the reported event (changes only after 3 identical samples). */
motion_event_t mev_step(mev_ctx_t *p_m, bool b_impact, hump_state_t hump, float turn_dps,
                        float a_fwd_g, float v_mm_s, uint32_t dt_ms);

/* EMA low-pass: y += alpha * (x - y) */
float          ema(float y, float x, float alpha);

#endif /* IMU_LOGIC_H */
