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
#define SIDE_ZONE_CM    (60.0f)    /* side blockers: up to this far beyond the obstacle's front */
#define TAN15           (0.2679492f)

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

/* one echo beside the obstacle: narrows that side's clearance */
static void side_blocker(obstacle_profile_t *p_out, float deg, float r, float zone_cm)
{
    float x;
    float y;

    if (r < 0.0f)
    {
        return;                              /* no echo */
    }
    x = r * sinf(deg * DEG_TO_RAD);
    y = r * cosf(deg * DEG_TO_RAD);
    if (y > zone_cm)
    {
        return;                              /* well past the obstacle: not in the bypass */
    }
    if ((deg < p_out->theta_l_deg) && (x < p_out->x_l_cm))
    {
        float c = p_out->x_l_cm - x;

        p_out->clear_left_cm = (c < p_out->clear_left_cm) ? c : p_out->clear_left_cm;
    }
    if ((deg > p_out->theta_r_deg) && (x > p_out->x_r_cm))
    {
        float c = x - p_out->x_r_cm;

        p_out->clear_right_cm = (c < p_out->clear_right_cm) ? c : p_out->clear_right_cm;
    }
}

bool obstacle_profile(const scan_t *p_fine, const float *p_coarse_cm,
                      float robot_half_mm, float margin_mm, obstacle_profile_t *p_out)
{
    uint32_t i_min = 0u;
    float    r_min = 1.0e9f;
    float    thr;
    uint32_t lo;
    uint32_t hi;

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

    /* an obstacle wider than the fine scan: its cluster reaches the end of
       the scan, and a coarse echo just beyond (within the cluster distance)
       is the same obstacle continuing - widen it instead of calling it a
       side blocker */
    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        float r = p_coarse_cm[i];
        float x;

        if ((r < 0.0f) || (r > thr))
        {
            continue;
        }
        x = r * sinf(COARSE_DEG[i] * DEG_TO_RAD);
        if ((lo == 0u) && (COARSE_DEG[i] < p_out->theta_l_deg) && (x < p_out->x_l_cm))
        {
            p_out->x_l_cm = x;
        }
        if (((hi + 1u) == p_fine->n) && (COARSE_DEG[i] > p_out->theta_r_deg) && (x > p_out->x_r_cm))
        {
            p_out->x_r_cm = x;
        }
    }
    p_out->width_cm    = p_out->x_r_cm - p_out->x_l_cm;
    p_out->offset_cm   = 0.5f * (p_out->x_l_cm + p_out->x_r_cm);

    /* side clearance: distance from the obstacle's edge to the nearest thing
       seen beside it (no echo = open). 28 Oct: EVERY echo that is not the
       obstacle's own cluster counts, however close - a second object just
       beside it used to be taken for the obstacle (within r_min + 15 cm)
       and ignored, so a blocked side read "200 cm, open". Only echoes
       within the bypass zone (up to SIDE_ZONE_CM beyond the obstacle's
       front) count: something a metre ahead is not in the way. */
    p_out->clear_left_cm  = OPEN_CM;
    p_out->clear_right_cm = OPEN_CM;
    for (uint32_t i = 0u; i < p_fine->n; i++)
    {
        if ((i < lo) || (i > hi))
        {
            side_blocker(p_out, p_fine->deg[i], p_fine->cm[i], r_min + SIDE_ZONE_CM);
        }
    }
    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        float r = p_coarse_cm[i];
        bool  b_cont;

        if ((COARSE_DEG[i] >= p_out->theta_l_deg) && (COARSE_DEG[i] <= p_out->theta_r_deg))
        {
            continue;                        /* the obstacle's own direction */
        }
        /* already used above to widen the obstacle */
        b_cont = (r >= 0.0f) && (r <= thr) &&
                 (((lo == 0u) && (COARSE_DEG[i] < p_out->theta_l_deg)) ||
                  (((hi + 1u) == p_fine->n) && (COARSE_DEG[i] > p_out->theta_r_deg)));
        if (!b_cont)
        {
            side_blocker(p_out, COARSE_DEG[i], r, r_min + SIDE_ZONE_CM);
        }
    }

    p_out->side    = obstacle_choose_side(p_out, robot_half_mm, margin_mm, 0u);
    p_out->b_valid = true;
    return true;
}

bool obstacle_side_room(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm)
{
    float need_mm = (2.0f * robot_half_mm) + margin_mm;   /* room the car needs beside it */

    if (side == 'L')
    {
        return (p_prof->clear_left_cm * 10.0f) >= need_mm;
    }
    if (side == 'R')
    {
        return (p_prof->clear_right_cm * 10.0f) >= need_mm;
    }
    return false;
}

char obstacle_choose_side(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                          uint32_t blocked)
{
    float shift_l    = (robot_half_mm + margin_mm) - (p_prof->x_l_cm * 10.0f);
    float shift_r    = (robot_half_mm + margin_mm) + (p_prof->x_r_cm * 10.0f);
    bool  b_left_ok  = obstacle_side_room(p_prof, 'L', robot_half_mm, margin_mm) && ((blocked & OBST_BLOCKED_L) == 0u);
    bool  b_right_ok = obstacle_side_room(p_prof, 'R', robot_half_mm, margin_mm) && ((blocked & OBST_BLOCKED_R) == 0u);

    if (b_left_ok && b_right_ok)
    {
        /* both possible: more space first (28 Oct), then the smaller deviation
           (DR 6.2 STAGE3, brief: "minimise deviation") */
        if (p_prof->clear_left_cm > (p_prof->clear_right_cm + OBST_SIDE_TIE_CM))
        {
            return 'L';
        }
        if (p_prof->clear_right_cm > (p_prof->clear_left_cm + OBST_SIDE_TIE_CM))
        {
            return 'R';
        }
        return (shift_l <= shift_r) ? 'L' : 'R';
    }
    if (b_left_ok)
    {
        return 'L';
    }
    if (b_right_ok)
    {
        return 'R';
    }
    return 0;                                /* no room: reverse and re-attempt */
}

float obstacle_standoff_mm(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm)
{
    float e = (side == 'L') ? (-p_prof->x_l_cm * 10.0f) : (p_prof->x_r_cm * 10.0f);

    /* the 45 deg leg from (0, 0) passes the corner (e, y) at (y - e) / sqrt(2) */
    return e + (1.4142136f * (robot_half_mm + margin_mm));
}

bool obstacle_in_path(float r_cm, float deg, float robot_half_mm, float side_mm, float ahead_cm)
{
    float x;
    float y;

    if (r_cm < 0.0f)
    {
        return false;
    }
    x = r_cm * sinf(deg * DEG_TO_RAD);
    y = r_cm * cosf(deg * DEG_TO_RAD);
    x = (x < 0.0f) ? -x : x;
    return (y >= 0.0f) && (y <= ahead_cm) && ((x * 10.0f) <= (robot_half_mm + side_mm));
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
    /* the diagonal return must be able to come back the whole shift
       (a wider retry shifts more than the DR's 300 mm) */
    p_plan->return_max_mm = (((p_plan->shift_mm + 100.0f) > max_lateral_mm) ? (p_plan->shift_mm + 100.0f) : max_lateral_mm) / SIN45;
    p_plan->realign_deg   = s * 45.0f;
    return true;
}

void obstacle_pose_advance(float *p_x, float *p_y, float h_prev_deg, float h_deg, float ds_mm)
{
    float hm = 0.5f * (h_prev_deg + h_deg) * DEG_TO_RAD;

    *p_x += ds_mm * sinf(hm);
    *p_y += ds_mm * cosf(hm);
}

float obstacle_sin_deg(float deg)
{
    return sinf(deg * DEG_TO_RAD);
}

bool obstacle_side_seen(float r_cm, float gap_mm)
{
    /* the side, gap_mm beside the car, seen at 60 deg is gap / sin 60 away;
       allow 10 cm more (the car drifts, the side is not straight) */
    return (r_cm >= 0.0f) && ((r_cm * 10.0f * 0.8660254f) <= (gap_mm + 100.0f));
}

float obstacle_side_trail_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm)
{
    /* last echo from the side's end with the beam's far edge at 75 deg: the
       back corner is then gap x tan15 ahead of the sensor, us_ahead + that
       ahead of the car's centre. The 45 deg return from the centre passes
       a corner gap to the side and dy ahead at (gap - dy) / sqrt2, which
       must be >= half + margin. */
    float d = us_ahead_mm + (gap_mm * TAN15) - gap_mm + (1.4142136f * (robot_half_mm + margin_mm));

    return (d < 50.0f) ? 50.0f : d;
}

bool obstacle_side_beside(float r90_cm, float gap_mm)
{
    return (r90_cm >= 0.0f) && ((r90_cm * 10.0f) <= ((gap_mm - 40.0f) + 100.0f));
}

float obstacle_side_trail90_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm)
{
    /* sensor us_ahead x cos30 ahead of the centre after the pivot, beam +-15
       deg reaches ~27 mm further; the 45 deg return then needs the corner
       (gap - dy) / sqrt2 >= half + margin away */
    float d = (us_ahead_mm * 0.8660254f) + 27.0f - gap_mm + (1.4142136f * (robot_half_mm + margin_mm));

    return (d < 50.0f) ? 50.0f : d;
}
