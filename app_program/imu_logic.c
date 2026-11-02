/**
 * @file    imu_logic.c
 * @brief   Pitch, hump profiler and motion-event classifier (see imu_logic.h).
 */
#include <math.h>
#include "imu_logic.h"
#include "robot_config.h"

#define RAD_TO_DEG   (57.29578f)
#define DEG_TO_RAD   (0.01745329f)
#define STILL_MS     (200u)

float imu_pitch_deg(float ax, float ay, float az, int32_t axis, int32_t sign, float a_fwd_g)
{
    float fwd   = (axis == 2) ? ay : ax;
    float side  = (axis == 2) ? ax : ay;
    float s     = (sign < 0) ? -1.0f : 1.0f;
    float other = sqrtf((side * side) + (az * az));

    return atan2f((s * fwd) - a_fwd_g, other) * RAD_TO_DEG;
}

float ema(float y, float x, float alpha)
{
    return y + (alpha * (x - y));
}

void hump_reset(hump_ctx_t *p_h)
{
    p_h->f       = 0u;                 /* HUMP_FLAT, every counter 0 */
    p_h->h_mm    = 0.0f;
    p_h->peak_mm = 0.0f;
    p_h->len_mm  = 0.0f;
}

static void end_hump(hump_ctx_t *p_h, hump_out_t *p_out, bool b_valid)
{
    p_out->b_end     = true;
    p_out->b_valid   = b_valid;
    p_out->height_mm = b_valid ? p_h->peak_mm : 0.0f;
    p_out->len_mm    = p_h->len_mm;
    hump_reset(p_h);
}

hump_out_t hump_step(hump_ctx_t *p_h, const hump_params_t *p_par, float pitch_deg, float ds_mm)
{
    hump_out_t out = { false, false, false, false, 0.0f, 0.0f };
    bool       b_level = (fabsf(pitch_deg) <= p_par->level_deg);

    if (ds_mm < 0.0f)
    {
        ds_mm = -ds_mm;
    }
    (void)hump_set_n_up(p_h, (pitch_deg >= p_par->start_deg) ? bp_sat_inc(hump_n_up(p_h), HUMP_N_MAX) : 0u);
    (void)hump_set_n_down(p_h, (pitch_deg <= -p_par->start_deg) ? bp_sat_inc(hump_n_down(p_h), HUMP_N_MAX) : 0u);
    (void)hump_set_n_level(p_h, b_level ? bp_sat_inc(hump_n_level(p_h), HUMP_N_MAX) : 0u);

    if (hump_state(p_h) == HUMP_FLAT)
    {
        if (hump_n_up(p_h) >= p_par->start_samples)
        {
            (void)hump_set_state(p_h, HUMP_ASCENT);
            p_h->h_mm    = 0.0f;
            p_h->peak_mm = 0.0f;
            p_h->len_mm  = 0.0f;
            out.b_start  = true;
        }
        return out;
    }

    /* integrate the height in every hump state */
    p_h->h_mm   += sinf(pitch_deg * DEG_TO_RAD) * ds_mm;
    p_h->len_mm += ds_mm;
    if (p_h->h_mm > p_h->peak_mm)
    {
        p_h->peak_mm = p_h->h_mm;
    }

    if (p_h->len_mm > p_par->max_len_mm)
    {
        end_hump(p_h, &out, false);
        return out;
    }

    switch (hump_state(p_h))
    {
        case HUMP_ASCENT:
            if (pitch_deg < p_par->level_deg)
            {
                if (p_h->h_mm > p_par->min_height_mm)
                {
                    (void)hump_set_state(p_h, HUMP_CREST);
                    out.b_peak = true;
                }
                else
                {
                    end_hump(p_h, &out, false);   /* false slope */
                }
            }
            break;

        case HUMP_CREST:
            if (hump_n_down(p_h) >= p_par->start_samples)
            {
                (void)hump_set_state(p_h, HUMP_DESCENT);
            }
            else if (hump_n_up(p_h) >= p_par->start_samples)
            {
                (void)hump_set_state(p_h, HUMP_ASCENT);   /* double hump: keep the running peak */
            }
            else
            {
                /* flat top */
            }
            break;

        case HUMP_DESCENT:
            if (hump_n_level(p_h) >= p_par->end_samples)
            {
                end_hump(p_h, &out, true);
            }
            break;

        default:
            hump_reset(p_h);
            break;
    }
    return out;
}

void mev_reset(mev_ctx_t *p_m)
{
    p_m->f        = 0u;                /* MEV_STATIONARY twice, count 0 */
    p_m->still_ms = 0u;
}

motion_event_t mev_step(mev_ctx_t *p_m, bool b_impact, hump_state_t hump, float turn_dps,
                        float a_fwd_g, float v_mm_s, uint32_t dt_ms)
{
    motion_event_t ev;

    p_m->still_ms = (fabsf(v_mm_s) < STATIONARY_MM_S) ? (p_m->still_ms + dt_ms) : 0u;

    if (b_impact)
    {
        ev = MEV_IMPACT;
    }
    else if ((hump == HUMP_ASCENT) || (hump == HUMP_CREST))
    {
        ev = MEV_CLIMBING;
    }
    else if (hump == HUMP_DESCENT)
    {
        ev = MEV_DESCENDING;
    }
    else if (fabsf(turn_dps) >= TURNING_DPS)
    {
        ev = MEV_TURNING;
    }
    else if (a_fwd_g >= ACCEL_G)
    {
        ev = MEV_ACCELERATING;
    }
    else if (a_fwd_g <= -ACCEL_G)
    {
        ev = MEV_DECELERATING;
    }
    else if (p_m->still_ms >= STILL_MS)
    {
        ev = MEV_STATIONARY;
    }
    else
    {
        ev = MEV_CRUISING;
    }

    if (ev == MEV_IMPACT)
    {
        (void)mev_set_current(p_m, ev);       /* an impact is reported at once */
        (void)mev_set_n_candidate(p_m, 0u);
    }
    else if (ev == mev_candidate(p_m))
    {
        (void)mev_set_n_candidate(p_m, bp_sat_inc(mev_n_candidate(p_m), MEV_HOLD));
        if (mev_n_candidate(p_m) >= MEV_HOLD)
        {
            (void)mev_set_current(p_m, ev);
        }
    }
    else
    {
        (void)mev_set_candidate(p_m, ev);
        (void)mev_set_n_candidate(p_m, 1u);
    }
    return mev_current(p_m);
}
