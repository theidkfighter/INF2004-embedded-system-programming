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

motion_event_t mev_step(mev_ctx_t *p_m, bool b_lifted, hump_state_t hump, float turn_dps,
                        float a_fwd_g, float v_mm_s, uint32_t dt_ms)
{
    motion_event_t ev;

    p_m->still_ms = (fabsf(v_mm_s) < STATIONARY_MM_S) ? (p_m->still_ms + dt_ms) : 0u;

    if (b_lifted)
    {
        ev = MEV_LIFTED;
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

    if (ev == MEV_LIFTED)
    {
        (void)mev_set_current(p_m, ev);       /* a lift is reported at once */
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

/* ---- tilt hump detector (see imu_logic.h) ---------------------------------------- */

float imu_tilt_deg(const float p_g[3], const float p_g0[3])
{
    float dot = (p_g[0] * p_g0[0]) + (p_g[1] * p_g0[1]) + (p_g[2] * p_g0[2]);
    float n   = sqrtf(((p_g[0] * p_g[0]) + (p_g[1] * p_g[1]) + (p_g[2] * p_g[2])) *
                      ((p_g0[0] * p_g0[0]) + (p_g0[1] * p_g0[1]) + (p_g0[2] * p_g0[2])));
    float c;

    if (n < 0.25f)
    {
        return 0.0f;                          /* no reference yet */
    }
    c = dot / n;
    c = (c > 1.0f) ? 1.0f : ((c < -1.0f) ? -1.0f : c);
    return acosf(c) * RAD_TO_DEG;
}

float imu_lift_mm(float lever_mm, float tilt_deg)
{
    return lever_mm * sinf(fabsf(tilt_deg) * DEG_TO_RAD);
}

static uint32_t th_get(const tilt_hump_t *p_t, uint32_t sh, uint32_t w)
{
    return BP_GET(p_t->f, sh, w);
}

static void th_put(tilt_hump_t *p_t, uint32_t sh, uint32_t w, uint32_t v)
{
    p_t->f = (uint8_t)BP_PUT(p_t->f, sh, w, v);
}

void tilt_hump_reset(tilt_hump_t *p_t)
{
    p_t->f          = 0u;
    p_t->h_pitch_mm = 0.0f;
    p_t->lift_mm    = 0.0f;
    p_t->peak_mm    = 0.0f;
    p_t->len_mm     = 0.0f;
    p_t->pitch_min  = 0.0f;
}

static bool th_valid(const tilt_hump_t *p_t, const hump_params_t *p_par)
{
    return (p_t->peak_mm >= p_par->min_height_mm) &&
           ((HUMP_TILT_NOSE_DOWN_DEG <= 0.0f) || (p_t->pitch_min <= -HUMP_TILT_NOSE_DOWN_DEG) ||
            (p_t->h_pitch_mm >= p_par->min_height_mm));
}

static void th_end(tilt_hump_t *p_t, hump_out_t *p_out, bool b_valid)
{
    p_out->b_end     = true;
    p_out->b_valid   = b_valid;
    p_out->height_mm = b_valid ? p_t->peak_mm : 0.0f;
    p_out->len_mm    = p_t->len_mm;
    th_put(p_t, TH_SH_ON, 1u, 0u);
    p_t->lift_mm = 0.0f;
}

hump_out_t tilt_hump_step(tilt_hump_t *p_t, const hump_params_t *p_par, float pitch_deg,
                          float tilt_deg, float lever_mm, float ds_mm)
{
    hump_out_t out = { false, false, false, false, 0.0f, 0.0f };
    uint32_t   n_tilt  = th_get(p_t, TH_SH_N_TILT, TH_W_N);
    uint32_t   n_level = th_get(p_t, TH_SH_N_LEVEL, TH_W_N);

    if (ds_mm < 0.0f)
    {
        ds_mm = -ds_mm;
    }
    n_tilt  = (tilt_deg >= p_par->start_deg) ? bp_sat_inc(n_tilt, 7u) : 0u;
    n_level = (tilt_deg <= p_par->level_deg) ? bp_sat_inc(n_level, 7u) : 0u;
    th_put(p_t, TH_SH_N_TILT, TH_W_N, n_tilt);
    th_put(p_t, TH_SH_N_LEVEL, TH_W_N, n_level);

    if (!tilt_hump_on(p_t))
    {
        if (n_tilt >= p_par->start_samples)
        {
            p_t->h_pitch_mm = 0.0f;
            p_t->lift_mm    = 0.0f;
            p_t->peak_mm    = 0.0f;
            p_t->len_mm     = 0.0f;
            p_t->pitch_min  = 0.0f;
            th_put(p_t, TH_SH_ON, 1u, 1u);
            th_put(p_t, TH_SH_PEAK, 1u, 0u);
            out.b_start = true;
        }
        return out;
    }
    p_t->h_pitch_mm += sinf(pitch_deg * DEG_TO_RAD) * ds_mm;
    p_t->len_mm     += ds_mm;
    p_t->pitch_min   = (pitch_deg < p_t->pitch_min) ? pitch_deg : p_t->pitch_min;
    {
        float tilt_lift = imu_lift_mm(lever_mm, tilt_deg);

        p_t->lift_mm = (tilt_lift > p_t->h_pitch_mm) ? tilt_lift : p_t->h_pitch_mm;
    }
    if (p_t->lift_mm > p_t->peak_mm)
    {
        p_t->peak_mm = p_t->lift_mm;
    }
    if ((th_get(p_t, TH_SH_PEAK, 1u) == 0u) && th_valid(p_t, p_par))
    {
        th_put(p_t, TH_SH_PEAK, 1u, 1u);
        out.b_peak = true;
    }
    if (p_t->len_mm > p_par->max_len_mm)
    {
        th_end(p_t, &out, false);             /* too long for a hump: no height */
    }
    else if (n_level >= p_par->end_samples)
    {
        th_end(p_t, &out, th_valid(p_t, p_par));
    }
    else
    {
        /* still on the hump */
    }
    return out;
}

/* ---- lift / overturn ------------------------------------------------------- */

#define LIFT_G_MM_S2     (9806.65f)
#define LIFT_DT_S        (0.02f)

void lift_reset(lift_ctx_t *p_l)
{
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        p_l->ref[i] = 0.0f;
        p_l->lp[i]  = 0.0f;
    }
    p_l->v_mm_s  = 0.0f;
    p_l->n_blank = 0u;
    p_l->n_tilt  = 0u;
    p_l->n_over  = 0u;
    p_l->cause   = (uint8_t)LIFT_NONE;
}

float lift_tilt_deg(const lift_ctx_t *p_l)
{
    return (p_l->n_blank == 0xFFu) ? imu_tilt_deg(p_l->lp, p_l->ref) : 0.0f;
}

static uint8_t sat_inc(uint8_t n)
{
    return (n < 0xFEu) ? (uint8_t)(n + 1u) : n;
}

lift_cause_t lift_step(lift_ctx_t *p_l, const lift_params_t *p_par, float ax, float ay, float az)
{
    float    a[3] = { ax, ay, az };
    float    up;
    float    n;
    float    tilt;
    float    da;
    uint32_t i;

    if (p_l->cause != (uint8_t)LIFT_NONE)
    {
        return LIFT_NONE;                         /* latched: reported once */
    }
    if (p_l->n_blank != 0xFFu)                    /* 0xFF = reference ready */
    {
        for (i = 0u; i < 3u; i++)
        {
            p_l->ref[i] += a[i];
            p_l->lp[i]   = a[i];
        }
        p_l->n_blank++;
        if (p_l->n_blank >= p_par->blank_samples)
        {
            for (i = 0u; i < 3u; i++)
            {
                p_l->ref[i] /= (float)p_l->n_blank;
            }
            p_l->n_blank = 0xFFu;
        }
        return LIFT_NONE;
    }

    for (i = 0u; i < 3u; i++)
    {
        p_l->lp[i] += p_par->alpha * (a[i] - p_l->lp[i]);
    }
    n    = sqrtf((p_l->ref[0] * p_l->ref[0]) + (p_l->ref[1] * p_l->ref[1]) + (p_l->ref[2] * p_l->ref[2]));
    up   = (n > 0.25f) ? (((a[0] * p_l->ref[0]) + (a[1] * p_l->ref[1]) + (a[2] * p_l->ref[2])) / n) : 1.0f;
    da   = up - 1.0f;
    da   = (da > p_par->up_clip_g) ? p_par->up_clip_g : ((da < -p_par->up_clip_g) ? -p_par->up_clip_g : da);
    p_l->v_mm_s = (p_par->v_leak * p_l->v_mm_s) + (da * LIFT_G_MM_S2 * LIFT_DT_S);
    tilt = imu_tilt_deg(p_l->lp, p_l->ref);

    p_l->n_tilt = (tilt >= p_par->tilt_deg) ? sat_inc(p_l->n_tilt) : 0u;
    p_l->n_over = (tilt >= p_par->over_deg) ? sat_inc(p_l->n_over) : 0u;

    if (p_l->n_over >= p_par->over_samples)
    {
        p_l->cause = (uint8_t)LIFT_OVER;
    }
    else if (p_l->v_mm_s >= p_par->v_lift_mm_s)
    {
        p_l->cause = (uint8_t)LIFT_UP;
    }
    else if (p_l->n_tilt >= p_par->tilt_samples)
    {
        p_l->cause = (uint8_t)((tilt >= p_par->over_deg) ? LIFT_OVER : LIFT_TILT);
    }
    else
    {
        /* on the floor */
    }
    return (lift_cause_t)p_l->cause;
}
