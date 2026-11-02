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
#include "bitpack.h"
#include "robot_config.h"

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

/* The sample counters only ever meet the thresholds, so each is stored as the
   change requirement's "value 1 to 5 in 3 bits" example: it counts up to the
   larger threshold (5) and holds there (bp_sat_inc). A held 5 compares the
   same as the baseline's higher count against a threshold of 3 or 5. A
   caller's start_samples / end_samples must not exceed HUMP_N_MAX. */
#define HUMP_N_MAX      ((uint32_t)((HUMP_START_SAMPLES > HUMP_END_SAMPLES) ? \
                                    HUMP_START_SAMPLES : HUMP_END_SAMPLES))

#define HUMP_SH_STATE   (0u)
#define HUMP_W_STATE    (2u)
#define HUMP_SH_N_UP    (2u)
#define HUMP_SH_N_DOWN  (5u)
#define HUMP_SH_N_LEVEL (8u)
#define HUMP_W_N        (3u)

BP_STATIC_ASSERT((uint32_t)HUMP_DESCENT < (1u << HUMP_W_STATE), "hump_ctx.state too narrow");
BP_STATIC_ASSERT(HUMP_N_MAX <= 7u, "hump_ctx.n_up, n_down, n_level too narrow");
BP_STATIC_ASSERT((HUMP_SH_N_LEVEL + HUMP_W_N) <= 16u, "hump_ctx.n_level does not fit its unit");

typedef struct
{
    uint16_t f;               /* state E2, n_up, n_down, n_level U3 sat 0..5 */
    float    h_mm;            /* the rest: calculation variables (D16) */
    float    peak_mm;
    float    len_mm;
} hump_ctx_t;

/* Packed field get / set of one task's private context, so no section. A
   setter rejects an out-of-range value: it returns false and leaves the unit
   unchanged. */
static inline bool hump_put(hump_ctx_t *p_h, uint32_t sh, uint32_t w, uint32_t v, uint32_t max)
{
    uint32_t u    = p_h->f;
    bool     b_ok = bp_set_checked(&u, sh, w, v, 0u, max);

    p_h->f = (uint16_t)u;
    return b_ok;
}

static inline hump_state_t hump_state(const hump_ctx_t *p_h)
{
    return (hump_state_t)BP_GET(p_h->f, HUMP_SH_STATE, HUMP_W_STATE);
}

static inline uint32_t hump_n_up(const hump_ctx_t *p_h)
{
    return BP_GET(p_h->f, HUMP_SH_N_UP, HUMP_W_N);
}

static inline uint32_t hump_n_down(const hump_ctx_t *p_h)
{
    return BP_GET(p_h->f, HUMP_SH_N_DOWN, HUMP_W_N);
}

static inline uint32_t hump_n_level(const hump_ctx_t *p_h)
{
    return BP_GET(p_h->f, HUMP_SH_N_LEVEL, HUMP_W_N);
}

static inline bool hump_set_state(hump_ctx_t *p_h, hump_state_t state)
{
    return hump_put(p_h, HUMP_SH_STATE, HUMP_W_STATE, (uint32_t)state, (uint32_t)HUMP_DESCENT);
}

static inline bool hump_set_n_up(hump_ctx_t *p_h, uint32_t n)
{
    return hump_put(p_h, HUMP_SH_N_UP, HUMP_W_N, n, HUMP_N_MAX);
}

static inline bool hump_set_n_down(hump_ctx_t *p_h, uint32_t n)
{
    return hump_put(p_h, HUMP_SH_N_DOWN, HUMP_W_N, n, HUMP_N_MAX);
}

static inline bool hump_set_n_level(hump_ctx_t *p_h, uint32_t n)
{
    return hump_put(p_h, HUMP_SH_N_LEVEL, HUMP_W_N, n, HUMP_N_MAX);
}

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

/* n_candidate holds at MEV_HOLD, the only value it is compared with. */
#define MEV_HOLD        (3u)

#define MEV_SH_CURRENT  (0u)
#define MEV_SH_CAND     (3u)
#define MEV_W_EV        (3u)
#define MEV_SH_N_CAND   (6u)
#define MEV_W_N_CAND    (2u)

BP_STATIC_ASSERT((uint32_t)MEV_COUNT <= (1u << MEV_W_EV), "mev_ctx.current, candidate too narrow");
BP_STATIC_ASSERT(MEV_HOLD < (1u << MEV_W_N_CAND), "mev_ctx.n_candidate too narrow");
BP_STATIC_ASSERT((MEV_SH_N_CAND + MEV_W_N_CAND) <= 8u, "mev_ctx.n_candidate does not fit its unit");

typedef struct
{
    uint8_t  f;               /* current E3, candidate E3, n_candidate U2 sat 0..3 */
    uint32_t still_ms;        /* timer: calculation variable (D16) */
} mev_ctx_t;

/* Same rules as hump_put(). */
static inline bool mev_put(mev_ctx_t *p_m, uint32_t sh, uint32_t w, uint32_t v, uint32_t max)
{
    uint32_t u    = p_m->f;
    bool     b_ok = bp_set_checked(&u, sh, w, v, 0u, max);

    p_m->f = (uint8_t)u;
    return b_ok;
}

static inline motion_event_t mev_current(const mev_ctx_t *p_m)
{
    return (motion_event_t)BP_GET(p_m->f, MEV_SH_CURRENT, MEV_W_EV);
}

static inline motion_event_t mev_candidate(const mev_ctx_t *p_m)
{
    return (motion_event_t)BP_GET(p_m->f, MEV_SH_CAND, MEV_W_EV);
}

static inline uint32_t mev_n_candidate(const mev_ctx_t *p_m)
{
    return BP_GET(p_m->f, MEV_SH_N_CAND, MEV_W_N_CAND);
}

static inline bool mev_set_current(mev_ctx_t *p_m, motion_event_t ev)
{
    return mev_put(p_m, MEV_SH_CURRENT, MEV_W_EV, (uint32_t)ev, (uint32_t)MEV_COUNT - 1u);
}

static inline bool mev_set_candidate(mev_ctx_t *p_m, motion_event_t ev)
{
    return mev_put(p_m, MEV_SH_CAND, MEV_W_EV, (uint32_t)ev, (uint32_t)MEV_COUNT - 1u);
}

static inline bool mev_set_n_candidate(mev_ctx_t *p_m, uint32_t n)
{
    return mev_put(p_m, MEV_SH_N_CAND, MEV_W_N_CAND, n, MEV_HOLD);
}

void           mev_reset(mev_ctx_t *p_m);
/* Returns the reported event (changes only after 3 identical samples). */
motion_event_t mev_step(mev_ctx_t *p_m, bool b_impact, hump_state_t hump, float turn_dps,
                        float a_fwd_g, float v_mm_s, uint32_t dt_ms);

/* EMA low-pass: y += alpha * (x - y) */
float          ema(float y, float x, float alpha);

#endif /* IMU_LOGIC_H */
