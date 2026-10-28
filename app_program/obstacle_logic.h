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

/* Sides already tried and found blocked during this avoidance */
#define OBST_BLOCKED_L   (1u)
#define OBST_BLOCKED_R   (2u)

/* Is there room for the car on 'side' ('L' / 'R') of the profiled obstacle? */
bool  obstacle_side_room(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* The side to pass on, among the sides that have room and are not in
   'blocked' (OBST_BLOCKED_*): the one with MORE clearance; if the two
   clearances are within OBST_SIDE_TIE_CM (e.g. both "no echo") the one
   needing the smaller sideways shift. 0 = none. obstacle_profile() uses it
   with blocked = 0. */
#define OBST_SIDE_TIE_CM (10.0f)
char  obstacle_choose_side(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                           uint32_t blocked);

/* How far (mm) the car must stand off the obstacle's front before the
   first 45 deg leg on 'side', so that leg passes the near corner by at
   least robot_half_mm + margin_mm: e + sqrt(2) (half + margin), e = how far
   the corner is out to that side of the car's centre line. If r_min is
   less, reverse the difference after the scan. */
float obstacle_standoff_mm(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* Bypass-leg watch: is a ping r_cm at deg (+ right) inside the car's path,
   i.e. within robot_half_mm + side_mm of the centre line and no more than
   ahead_cm in front? (r < 0 = no echo = no.) */
bool  obstacle_in_path(float r_cm, float deg, float robot_half_mm, float side_mm, float ahead_cm);

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

/* Parallel leg (side-following at 60 deg, the servo's limit): does a ping
   at 60 deg towards the obstacle still hit its side, gap_mm beside the car? */
bool  obstacle_side_seen(float r_cm, float gap_mm);

/* How far to drive on after the side was last seen at 60 deg before the
   45 deg turn back clears its back corner (us_ahead_mm: ultrasonic sensor
   ahead of the axle). */
float obstacle_side_trail_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Look-before-turning with the car pivoted 30 deg towards the obstacle and
   the servo at 60 (= square to its side): the side is still there if the
   ping is within the gap (less the 40 mm the pivot moved the sensor) +
   10 cm. If it is not, the back corner is at most ~10 cm ahead of the
   centre: drive on obstacle_side_trail90_mm() before the 45 deg turn. */
bool  obstacle_side_beside(float r90_cm, float gap_mm);
float obstacle_side_trail90_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Bypass dead reckoning: advance (x right, y forward) by ds_mm on the mean
   of two headings (deg, + = clockwise). */
void  obstacle_pose_advance(float *p_x, float *p_y, float h_prev_deg, float h_deg, float ds_mm);
float obstacle_sin_deg(float deg);

#endif /* OBSTACLE_LOGIC_H */
