/**
 * @file    obstacle_logic.h
 * @brief   Ultrasonic filtering, obstacle profiling and bypass planning
 *          (DR 6.2 STAGE0..STAGE4, FR9-FR11, NFR8).
 *
 * Angles are in the robot frame: 0 = straight ahead, + = right, - = left.
 * Lateral positions: x = r sin(theta) (+ = right), forward y = r cos(theta).
 *
 * Pure logic: unit-tested on the PC (tests/).
 */
#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"

#define OBS_MAX_POINTS   (64u)

/* ---- STAGE0: NFR8 - a single bad echo must not trigger avoidance ----
 * Median of the last 3 readings; the trigger needs 2 filtered readings in a
 * row below the stop distance. A no-echo reading (< 0) counts as "far". */
typedef struct
{
    float    r[3];
    uint32_t n;
    uint32_t below;
} us_filter_t;

void  usf_reset(us_filter_t *p_f);
float usf_add(us_filter_t *p_f, float cm);           /* returns the filtered value */
bool  usf_trigger(us_filter_t *p_f, float filtered_cm, float stop_cm);

float median3(float a, float b, float c);

/* ---- STAGE1: coarse sweep ---- */
#define COARSE_N  (5u)
extern const float COARSE_DEG[COARSE_N];             /* -60 -30 0 30 60 */
bool  coarse_all_clear(const float *p_cm, float clear_cm);

/* ---- STAGE2/3: fine scan -> profile ---- */
typedef struct
{
    float   deg[OBS_MAX_POINTS];
    float   cm[OBS_MAX_POINTS];                       /* < 0 = no echo */
    uint32_t n;
} scan_t;

/* Builds the profile from the fine scan and the coarse sweep.
   robot_half_mm + margin_mm = how far the robot centre must pass from an edge. */
bool  obstacle_profile(const scan_t *p_fine, const float *p_coarse_cm,
                       float robot_half_mm, float margin_mm, obstacle_profile_t *p_out);

/* ---- STAGE4: bypass plan ----
 * (1) pivot 45 deg away, (2) forward d1 so the car clears the edge,
 * (3) pivot back 45 deg (parallel to the course), (4) forward d2 past the
 * obstacle, (5) pivot 45 deg towards the line; the caller then drives the
 * diagonal return until the line is found (STAGE5). */
typedef struct
{
    int8_t side;              /* +1 = pass on the right, -1 = left */
    float  turn1_deg;         /* + = right */
    float  d1_mm;
    float  turn2_deg;
    float  d2_mm;
    float  turn3_deg;
    float  shift_mm;          /* lateral offset reached by leg 2 */
    float  return_max_mm;     /* diagonal return limit (DR: 300 mm lateral) */
    float  realign_deg;       /* after the line is found */
} bypass_plan_t;

bool  bypass_plan(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                  float depth_mm, float max_lateral_mm, bypass_plan_t *p_plan);

#endif /* OBSTACLE_LOGIC_H */
