/**
 * @file    speed_ctl.c
 * @brief   Per-wheel speed controller (see speed_ctl.h).
 */
#include "speed_ctl.h"

void wheel_reset(wheel_ctl_t *p_w)
{
    p_w->target = 0.0f;
    p_w->speed  = 0.0f;
    p_w->integ  = 0.0f;
    p_w->duty   = 0.0f;
}

float wheel_update(wheel_ctl_t *p_w, const speed_params_t *p_par, int32_t counts_delta,
                   float mm_per_count, float dt_s)
{
    float raw = ((float)counts_delta * mm_per_count) / dt_s;

    p_w->speed = (0.5f * p_w->speed) + (0.5f * raw);

    if (p_w->target == 0.0f)
    {
        p_w->integ = 0.0f;
        p_w->duty  = 0.0f;               /* 0 = brake (driver Low/Low) */
    }
    else
    {
        float err = p_w->target - p_w->speed;
        float ff  = 0.0f;
        float lim = (p_par->ki > 0.0f) ? (0.5f / p_par->ki) : 0.0f;

        p_w->integ += err * dt_s;
        if (p_w->integ > lim)
        {
            p_w->integ = lim;
        }
        if (p_w->integ < -lim)
        {
            p_w->integ = -lim;
        }
        if (p_par->ff_mm_s_per_pct > 0.0f)
        {
            float mag = (p_w->target > 0.0f) ? p_w->target : -p_w->target;

            ff = (p_par->ff_deadband_pct + (mag / p_par->ff_mm_s_per_pct)) / 100.0f;
            ff = (p_w->target > 0.0f) ? ff : -ff;
        }
        p_w->duty = ff + (p_par->kp * err) + (p_par->ki * p_w->integ);
        if (p_w->duty > p_par->max_duty)
        {
            p_w->duty = p_par->max_duty;
        }
        if (p_w->duty < -p_par->max_duty)
        {
            p_w->duty = -p_par->max_duty;
        }
    }
    return p_w->duty;
}
