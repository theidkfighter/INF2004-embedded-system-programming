/**
 * @file    obstacle_logic.c
 * @brief   Ultrasonic filtering, obstacle profiling and bypass planning
 *          (see obstacle_logic.h).
 */
#include <math.h>
#include "obstacle_logic.h"

#define DEG_TO_RAD      (0.01745329f)
#define CLUSTER_CM      (15.0f)    /* points within r_min + 15 cm belong to the obstacle */
#define OPEN_CM         (200.0f)   /* no echo on a side = this much room */
#define SIN45           (0.7071068f)

const float COARSE_DEG[COARSE_N] = { -60.0f, -30.0f, 0.0f, 30.0f, 60.0f };

float median3(float a, float b, float c)
{
    float t;

    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

void usf_reset(us_filter_t *p_f)
{
    p_f->r[0]  = -1.0f;
    p_f->r[1]  = -1.0f;
    p_f->r[2]  = -1.0f;
    p_f->n     = 0u;
    p_f->below = 0u;
}

float usf_add(us_filter_t *p_f, float cm)
{
    float far = 1000.0f;

    p_f->r[p_f->n % 3u] = (cm < 0.0f) ? far : cm;     /* no echo = nothing close */
    p_f->n++;
    if (p_f->n < 3u)
    {
        return far;
    }
    return median3(p_f->r[0], p_f->r[1], p_f->r[2]);
}

bool usf_trigger(us_filter_t *p_f, float filtered_cm, float stop_cm)
{
    p_f->below = (filtered_cm < stop_cm) ? (p_f->below + 1u) : 0u;
    return p_f->below >= 2u;
}

bool coarse_all_clear(const float *p_cm, float clear_cm)
{
    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        if ((p_cm[i] >= 0.0f) && (p_cm[i] <= clear_cm))
        {
            return false;
        }
    }
    return true;
}

bool obstacle_profile(const scan_t *p_fine, const float *p_coarse_cm,
                      float robot_half_mm, float margin_mm, obstacle_profile_t *p_out)
{
    uint32_t i_min = 0u;
    float    r_min = 1.0e9f;
    float    thr;
    uint32_t lo;
    uint32_t hi;
    float    need_mm = (2.0f * robot_half_mm) + margin_mm;   /* room the car needs beside it */
    float    shift_l;
    float    shift_r;
    bool     b_left_ok;
    bool     b_right_ok;

    p_out->b_valid = false;
    p_out->side    = 0;
    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        p_out->scan_cm[i] = p_coarse_cm[i];
    }

    for (uint32_t i = 0u; i < p_fine->n; i++)
    {
        if ((p_fine->cm[i] >= 0.0f) && (p_fine->cm[i] < r_min))
        {
            r_min = p_fine->cm[i];
            i_min = i;
        }
    }
    if (r_min > 1.0e8f)
    {
        return false;                        /* nothing seen in the fine scan */
    }

    /* grow the cluster around the closest point */
    thr = r_min + CLUSTER_CM;
    lo  = i_min;
    hi  = i_min;
    while ((lo > 0u) && (p_fine->cm[lo - 1u] >= 0.0f) && (p_fine->cm[lo - 1u] <= thr))
    {
        lo--;
    }
    while (((hi + 1u) < p_fine->n) && (p_fine->cm[hi + 1u] >= 0.0f) && (p_fine->cm[hi + 1u] <= thr))
    {
        hi++;
    }

    p_out->r_min_cm    = r_min;
    p_out->theta_l_deg = p_fine->deg[lo];
    p_out->theta_r_deg = p_fine->deg[hi];
    p_out->x_l_cm      = p_fine->cm[lo] * sinf(p_fine->deg[lo] * DEG_TO_RAD);
    p_out->x_r_cm      = p_fine->cm[hi] * sinf(p_fine->deg[hi] * DEG_TO_RAD);
    p_out->width_cm    = p_out->x_r_cm - p_out->x_l_cm;
    p_out->offset_cm   = 0.5f * (p_out->x_l_cm + p_out->x_r_cm);

    /* side clearance from the coarse sweep: distance from the obstacle edge
       to the next thing seen on that side (no echo = open) */
    p_out->clear_left_cm  = OPEN_CM;
    p_out->clear_right_cm = OPEN_CM;
    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        float r = p_coarse_cm[i];
        float x;

        if ((r < 0.0f) || (r <= thr) || (COARSE_DEG[i] == 0.0f))
        {
            continue;                        /* no echo, or the obstacle itself */
        }
        x = r * sinf(COARSE_DEG[i] * DEG_TO_RAD);
        if ((COARSE_DEG[i] < 0.0f) && (x < p_out->x_l_cm))
        {
            float c = p_out->x_l_cm - x;

            p_out->clear_left_cm = (c < p_out->clear_left_cm) ? c : p_out->clear_left_cm;
        }
        if ((COARSE_DEG[i] > 0.0f) && (x > p_out->x_r_cm))
        {
            float c = x - p_out->x_r_cm;

            p_out->clear_right_cm = (c < p_out->clear_right_cm) ? c : p_out->clear_right_cm;
        }
    }

    /* minimise deviation from the route: the side needing the smaller shift,
       if it has room (DR 6.2 STAGE3, brief: "minimise deviation") */
    shift_l    = (robot_half_mm + margin_mm) - (p_out->x_l_cm * 10.0f);
    shift_r    = (robot_half_mm + margin_mm) + (p_out->x_r_cm * 10.0f);
    b_left_ok  = (p_out->clear_left_cm * 10.0f) >= need_mm;
    b_right_ok = (p_out->clear_right_cm * 10.0f) >= need_mm;
    if (b_left_ok && (!b_right_ok || (shift_l <= shift_r)))
    {
        p_out->side = 'L';
    }
    else if (b_right_ok)
    {
        p_out->side = 'R';
    }
    else
    {
        p_out->side = 0;                     /* no room: reverse and re-attempt */
    }
    p_out->b_valid = true;
    return true;
}

bool bypass_plan(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                 float depth_mm, float max_lateral_mm, bypass_plan_t *p_plan)
{
    float s;

    if (!p_prof->b_valid || (p_prof->side == 0))
    {
        return false;
    }
    p_plan->side = (p_prof->side == 'R') ? 1 : -1;
    s = (float)p_plan->side;
    if (p_plan->side > 0)
    {
        p_plan->shift_mm = (p_prof->x_r_cm * 10.0f) + robot_half_mm + margin_mm;
    }
    else
    {
        p_plan->shift_mm = robot_half_mm + margin_mm - (p_prof->x_l_cm * 10.0f);
    }
    if (p_plan->shift_mm < robot_half_mm)
    {
        p_plan->shift_mm = robot_half_mm;
    }
    p_plan->turn1_deg = s * 45.0f;
    p_plan->d1_mm     = p_plan->shift_mm / SIN45;
    p_plan->turn2_deg = -s * 45.0f;
    /* leg 1 already advanced shift_mm forward (45 deg); go past the far side
       of the obstacle plus the car's own length margin */
    p_plan->d2_mm = ((p_prof->r_min_cm * 10.0f) + depth_mm + robot_half_mm + margin_mm) - p_plan->shift_mm;
    if (p_plan->d2_mm < 50.0f)
    {
        p_plan->d2_mm = 50.0f;
    }
    p_plan->turn3_deg     = -s * 45.0f;
    p_plan->return_max_mm = max_lateral_mm / SIN45;
    p_plan->realign_deg   = s * 45.0f;
    return true;
}
