/**
 * @file    line_logic.c
 * @brief   Straddle line following and line search (see line_logic.h).
 */
#include "line_logic.h"

static float clamp1(float v)
{
    return (v > 1.0f) ? 1.0f : ((v < -1.0f) ? -1.0f : v);
}

void line_reset(line_ctx_t *p_ctx)
{
    p_ctx->state          = LINE_CENTERED;
    p_ctx->side           = 0;
    p_ctx->black_s        = 0.0f;
    p_ctx->episode_mm     = 0.0f;
    p_ctx->hold_left_mm   = 0.0f;
    p_ctx->since_black_mm = 0.0f;
    p_ctx->b_last_curve   = false;
    p_ctx->b_both_prev    = false;
}

line_out_t line_step(line_ctx_t *p_ctx, const line_params_t *p_par,
                     bool b_left_black, bool b_right_black, float ds_mm, float dt_s)
{
    line_out_t out = { 0.0f, false, false };

    if (ds_mm < 0.0f)
    {
        ds_mm = -ds_mm;
    }

    if (b_left_black && b_right_black)
    {
        /* junction / crossing line: drive straight through */
        out.b_junction        = !p_ctx->b_both_prev;
        p_ctx->state          = LINE_JUNCTION;
        p_ctx->since_black_mm = 0.0f;
        p_ctx->hold_left_mm   = 0.0f;
        p_ctx->black_s        = 0.0f;
        p_ctx->episode_mm     = 0.0f;
        p_ctx->b_last_curve   = false;
        out.steer             = 0.0f;
    }
    else if (b_left_black || b_right_black)
    {
        int8_t side = b_left_black ? -1 : 1;
        bool   b_same = ((side < 0) && (p_ctx->state == LINE_CORRECT_LEFT)) ||
                        ((side > 0) && (p_ctx->state == LINE_CORRECT_RIGHT));

        if (!b_same)
        {
            p_ctx->black_s    = 0.0f;
            p_ctx->episode_mm = 0.0f;
        }
        p_ctx->side            = side;
        p_ctx->state           = (side < 0) ? LINE_CORRECT_LEFT : LINE_CORRECT_RIGHT;
        p_ctx->black_s        += dt_s;
        p_ctx->episode_mm     += ds_mm;
        p_ctx->since_black_mm  = 0.0f;
        out.steer = clamp1((float)side * (p_par->steer_base + (p_par->steer_ramp * p_ctx->black_s)));
    }
    else
    {
        /* both white */
        if ((p_ctx->state == LINE_CORRECT_LEFT) || (p_ctx->state == LINE_CORRECT_RIGHT))
        {
            p_ctx->b_last_curve = (p_ctx->episode_mm >= p_par->curve_mm);
            p_ctx->hold_left_mm = p_par->hold_mm;
            p_ctx->state        = (p_ctx->side < 0) ? LINE_HOLD_LEFT : LINE_HOLD_RIGHT;
        }
        else if (p_ctx->state == LINE_JUNCTION)
        {
            p_ctx->state = LINE_CENTERED;
        }
        else
        {
            /* stays as it is */
        }
        p_ctx->since_black_mm += ds_mm;

        if (p_ctx->state == LINE_LOST)
        {
            out.steer = 0.0f;
        }
        else if (p_ctx->b_last_curve && (p_ctx->since_black_mm >= p_par->lost_mm))
        {
            p_ctx->state = LINE_LOST;
            out.b_lost   = true;
            out.steer    = 0.0f;
        }
        else if ((p_ctx->state == LINE_HOLD_LEFT) || (p_ctx->state == LINE_HOLD_RIGHT))
        {
            p_ctx->hold_left_mm -= ds_mm;
            if (p_ctx->hold_left_mm > 0.0f)
            {
                out.steer = (float)p_ctx->side * p_par->hold_steer;
            }
            else
            {
                /* keep the HOLD state (it remembers the side) but stop steering */
                out.steer = 0.0f;
            }
        }
        else
        {
            out.steer = 0.0f;
        }
    }
    p_ctx->b_both_prev = b_left_black && b_right_black;
    return out;
}

void line_wheels(float base_mm_s, float steer, float *p_left, float *p_right)
{
    steer    = clamp1(steer);
    *p_left  = base_mm_s * (1.0f + steer);
    *p_right = base_mm_s * (1.0f - steer);
}

void search_start(search_ctx_t *p_s, int8_t first_dir, float first_deg, float max_deg)
{
    p_s->heading_deg = 0.0f;
    p_s->amp_deg     = first_deg;
    p_s->max_deg     = max_deg;
    p_s->dir         = (first_dir < 0) ? -1 : 1;
    p_s->b_second    = false;
}

int8_t search_step(search_ctx_t *p_s, float dheading_deg)
{
    float target;

    p_s->heading_deg += dheading_deg;
    if (p_s->amp_deg > p_s->max_deg)
    {
        return 0;
    }
    target = (float)p_s->dir * p_s->amp_deg;
    if (((p_s->dir > 0) && (p_s->heading_deg >= target)) ||
        ((p_s->dir < 0) && (p_s->heading_deg <= target)))
    {
        /* reached this side: sweep the other way */
        p_s->dir = (int8_t)(-p_s->dir);
        if (p_s->b_second)
        {
            p_s->amp_deg *= 2.0f;
            if (p_s->amp_deg > p_s->max_deg)
            {
                return 0;
            }
        }
        p_s->b_second = !p_s->b_second;
    }
    return p_s->dir;
}
