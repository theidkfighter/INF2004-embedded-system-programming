/**
 * @file    speed_ctl.h
 * @brief   Per-wheel speed controller: feed-forward + PI with anti-windup
 *          (FR13, same law as full_tests ft_speed_pid).
 *
 * duty = ff(target) + Kp * e + Ki * integral(e), clamped to -1..+1
 * ff(target) = (deadband % + |target| / (mm/s per %)) / 100, signed.
 * The integral is clamped to +-0.5 / Ki (0.5 duty) so a stalled wheel does
 * not wind up.
 *
 * Pure logic: unit-tested on the PC (tests/).
 */
#ifndef SPEED_CTL_H
#define SPEED_CTL_H

#include <stdint.h>

typedef struct
{
    float target;      /* mm/s */
    float speed;       /* filtered measured mm/s */
    float integ;
    float duty;
} wheel_ctl_t;

typedef struct
{
    float kp;
    float ki;
    float ff_deadband_pct;
    float ff_mm_s_per_pct;     /* <= 0: no feed-forward */
    float max_duty;            /* 1.0 */
} speed_params_t;

void  wheel_reset(wheel_ctl_t *p_w);
/* counts_delta: 4x encoder counts in dt_s; returns the new duty */
float wheel_update(wheel_ctl_t *p_w, const speed_params_t *p_par, int32_t counts_delta,
                   float mm_per_count, float dt_s);

#endif /* SPEED_CTL_H */
