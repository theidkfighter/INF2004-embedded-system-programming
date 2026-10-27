/**
 * @file    bc_detect.c
 * @brief   Barcode bar detector on distance bins (see bc_detect.h).
 */
#include "bc_detect.h"

void det_init(det_t *p_d, uint32_t white, uint32_t black, float hys_frac)
{
    det_init_level(p_d, white, black, hys_frac, 0.5f);
}

void det_init_level(det_t *p_d, uint32_t white, uint32_t black, float hys_frac, float level_frac)
{
    int32_t w = (int32_t)white;
    int32_t b = (int32_t)black;
    int32_t span;

    p_d->b_inv = (b < w);
    if (p_d->b_inv)
    {
        w = 4095 - w;
        b = 4095 - b;
    }
    span = b - w;
    if (span < 1)
    {
        span = 1;
    }
    if (level_frac < 0.05f) { level_frac = 0.05f; }
    if (level_frac > 0.95f) { level_frac = 0.95f; }
    p_d->mid = w + (int32_t)(level_frac * (float)span);
    p_d->hys = (int32_t)(hys_frac * (float)span);
    if (p_d->hys < DET_HYS_MIN)
    {
        p_d->hys = DET_HYS_MIN;
    }
    p_d->bin_idx      = 0u;
    p_d->sum          = 0u;
    p_d->cnt          = 0u;
    p_d->b_started    = false;
    p_d->raw0         = 0;
    p_d->raw1         = 0;
    p_d->raw_n        = 0u;
    p_d->b_black      = false;
    p_d->b_have_prev  = false;
    p_d->prev_s       = 0;
    p_d->prev_k       = 0u;
    p_d->cross_um     = 0u;
    p_d->run_ext      = 0;
    p_d->bars         = 0u;
    p_d->faintest_bar = 0x7FFFFFFF;
    p_d->palest_space = -1;
    p_d->last_space   = -1;
}

/* One smoothed bin s (darker = higher) for bin k. Returns 1 if an edge. */
static uint32_t level(det_t *p_d, int32_t s, uint32_t k, det_edge_t *p_out)
{
    uint32_t n = 0u;
    bool     b_now;

    /* where did the signal last cross the mid level? (between bin centres) */
    if (p_d->b_have_prev && (s != p_d->prev_s) &&
        (((p_d->prev_s - p_d->mid) * (s - p_d->mid)) <= 0))
    {
        float f = (float)(p_d->mid - p_d->prev_s) / (float)(s - p_d->prev_s);

        p_d->cross_um = (uint32_t)((((float)p_d->prev_k + f) * (float)DET_BIN_UM) + (float)(DET_BIN_UM / 2u));
    }
    b_now = p_d->b_black ? (s > (p_d->mid - p_d->hys)) : (s >= (p_d->mid + p_d->hys));
    if (b_now != p_d->b_black)
    {
        if (b_now)
        {
            /* a space ended: was it between two bars? */
            if (p_d->bars > 0u)
            {
                p_d->last_space = p_d->run_ext;
            }
        }
        else
        {
            p_d->bars++;
            if (p_d->run_ext < p_d->faintest_bar)
            {
                p_d->faintest_bar = p_d->run_ext;
            }
            if ((p_d->last_space >= 0) && (p_d->last_space > p_d->palest_space))
            {
                p_d->palest_space = p_d->last_space;
            }
        }
        p_d->b_black = b_now;
        p_d->run_ext = s;
        p_out[0].pos_um  = p_d->cross_um;
        p_out[0].b_black = b_now;
        n = 1u;
    }
    else if (p_d->b_black ? (s > p_d->run_ext) : (s < p_d->run_ext))
    {
        p_d->run_ext = s;                     /* darkest point of a bar, lightest of a space */
    }
    p_d->prev_s      = s;
    p_d->prev_k      = k;
    p_d->b_have_prev = true;
    return n;
}

uint32_t det_sample(det_t *p_d, uint32_t pos_um, uint32_t adc, det_edge_t *p_out)
{
    uint32_t bin = pos_um / DET_BIN_UM;
    uint32_t v   = p_d->b_inv ? (4095u - adc) : adc;
    uint32_t n   = 0u;

    if (!p_d->b_started)
    {
        p_d->b_started = true;
        p_d->bin_idx   = bin;
        p_d->sum       = v;
        p_d->cnt       = 1u;
        return 0u;
    }
    if (bin == p_d->bin_idx)
    {
        p_d->sum += v;
        p_d->cnt++;
        return 0u;
    }
    if (bin < p_d->bin_idx)
    {
        return 0u;                            /* positions never go back */
    }
    {
        int32_t  done = (int32_t)(p_d->sum / p_d->cnt);
        uint32_t k    = p_d->bin_idx;

        if ((bin - k) > 64u)
        {
            k = bin - 64u;                    /* a long jump (16 mm): replay only the end */
        }
        /* finished bin(s); bins skipped by a fast move repeat the last value.
           1-2-1 smoothing over neighbouring bins, so this runs one bin late. */
        for (; k < bin; k++)
        {
            det_edge_t e[1];

            if (p_d->raw_n == 0u)
            {
                p_d->raw0 = done;
                p_d->raw1 = done;
            }
            else if (level(p_d, (p_d->raw0 + (2 * p_d->raw1) + done) / 4, k - 1u, e) != 0u)
            {
                if (n < DET_MAX_OUT)
                {
                    p_out[n++] = e[0];
                }
            }
            p_d->raw0 = p_d->raw1;
            p_d->raw1 = done;
            p_d->raw_n++;
        }
        p_d->bin_idx = bin;
        p_d->sum     = v;
        p_d->cnt     = 1u;
    }
    return n;
}

float det_sensor_ds(int32_t d_r, int32_t d_l, float mm_per_count, float track_mm, float y_mm)
{
    float dc  = 0.5f * (float)(d_r + d_l) * mm_per_count;
    float dth = (track_mm > 0.0f) ? (((float)(d_r - d_l) * mm_per_count) / track_mm) : 0.0f;

    return dc - (y_mm * dth);     /* turning left (right wheel ahead): dth > 0 */
}

void det_stats(const det_t *p_d, uint32_t *p_bars, int32_t *p_faintest_bar, int32_t *p_palest_space)
{
    int32_t fb = (p_d->bars > 0u) ? p_d->faintest_bar : -1;
    int32_t ps = p_d->palest_space;

    if (p_d->b_inv)
    {
        fb = (fb >= 0) ? (4095 - fb) : -1;
        ps = (ps >= 0) ? (4095 - ps) : -1;
    }
    *p_bars          = p_d->bars;
    *p_faintest_bar  = fb;
    *p_palest_space  = ps;
}
