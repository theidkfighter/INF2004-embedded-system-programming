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

/* Choice flags: sides already tried and found blocked during this
   avoidance, plus OBST_CHOOSE_LEGACY (the main-branch clearance-first rule) */
#define OBST_BLOCKED_L     (1u)
#define OBST_BLOCKED_R     (2u)
#define OBST_CHOOSE_LEGACY (4u)

/* Is there room for the car on 'side' ('L' / 'R') of the profiled obstacle? */
bool  obstacle_side_room(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* Sideways shift (mm) the car centre needs to pass on 'side' ('L' / 'R'):
   L = half + margin - 10 x_l, R = half + margin + 10 x_r. Any other side
   gives 0. shift_r - shift_l = 20 x offset_cm. */
float obstacle_shift_mm(const obstacle_profile_t *p_prof, char side, float robot_half_mm, float margin_mm);

/* The side to pass on, among the sides that have room and are not excluded
   by 'flags' (OBST_BLOCKED_*): the one needing the smaller sideways shift
   (obstacle_shift_mm), 'L' when the shifts are equal - i.e. away from the
   obstacle's offset. Clearance only gates room (clear x 10 >= 2 half +
   margin), so a "no echo" side (200 cm) is not preferred over a side that
   sees something further than the room it needs. 0 = no side.
   With OBST_CHOOSE_LEGACY in 'flags': the old rule, the side with MORE
   clearance first, the smaller shift only when the clearances are within
   OBST_SIDE_TIE_CM. obstacle_profile() uses it with flags = 0 (new rule). */
#define OBST_SIDE_TIE_CM  (10.0f)
#define OBST_SHIFT_TIE_MM (10.0f)   /* = offset within +-0.5 cm of the car's axis */
char  obstacle_choose_side(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                           uint32_t flags);

/* Is the side choice a tie (worth the 90 deg tie look)? True when both sides
   have room, neither is excluded by 'flags' (OBST_BLOCKED_*) and the shifts
   differ by at most OBST_SHIFT_TIE_MM. With OBST_CHOOSE_LEGACY: the
   clearances differ by at most OBST_SIDE_TIE_CM instead. Whether a side has
   been tried already is the caller's check. */
bool  obstacle_is_tie(const obstacle_profile_t *p_prof, float robot_half_mm, float margin_mm,
                      uint32_t flags);

/* Reverse budget for one avoidance (standoff + rescan reverses):
   cap_mm <= 0 = no cap: FULL. Fits in what is left of the cap (+0.5 mm):
   FULL, allowed = want. Otherwise a reverse that may not be shortened (the
   standoff) is REFUSED; one that may (the rescan) is SHORTENED to what is
   left, or REFUSED when less than OBST_REVERSE_MIN_MM is left.
   REFUSED: allowed = 0. */
#define OBST_REVERSE_MIN_MM (5.0f)
typedef enum
{
    OBST_REV_FULL = 0,
    OBST_REV_SHORTENED,
    OBST_REV_REFUSED
} obst_rev_t;

obst_rev_t obstacle_reverse_budget(float want_mm, float used_mm, float cap_mm, bool b_shortenable,
                                   float *p_allowed_mm);

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

/* How far to drive on, measured from the FIRST 60 deg ping that missed the
   side (miss_at), before the 45 deg turn back clears its back corner
   (us_ahead_mm: ultrasonic sensor ahead of the axle = pivot centre).
   A miss with the servo at 60 (beam 45..75 deg) proves the corner is less
   than A + g tan15 ahead of the centre (A = us_ahead, g = gap). After a
   trail T the 45 deg return passes it at (g - Y_c) / sqrt2, which is
   >= half + margin when T >= A + g tan15 - g + sqrt2 (half + margin)
   (condition a). During the pivot the corner's lateral distance in the car
   frame, g cos phi - Y_c sin phi, is smallest at phi = 0 or 45 deg, both
   >= half + margin, so the body (|x| <= half) stays >= margin away
   (condition b) whatever its length. Floor 50 mm. Measured from the last
   sighting instead, the corner can be one ping step further ahead. */
float obstacle_side_trail_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Look-before-turning with the car pivoted 30 deg towards the obstacle and
   the servo at 60 (= square to its side): the side is still there if the
   ping is within the gap (less the 40 mm the pivot moved the sensor) +
   10 cm. If it is not, the sensor is at (A sin30, A cos30) and the beam's
   forward edge meets the side (g - A sin30) tan15 ahead of it, so the back
   corner is at most A cos30 + (g - A sin30) tan15 ahead of the centre:
   drive on obstacle_side_trail90_mm() = that - g + sqrt2 (half + margin)
   (floor 50 mm) before the 45 deg turn. Conditions (a) and (b) as for
   obstacle_side_trail_mm(). */
bool  obstacle_side_beside(float r90_cm, float gap_mm);
float obstacle_side_trail90_mm(float gap_mm, float us_ahead_mm, float robot_half_mm, float margin_mm);

/* Flat faces (31 Oct): a flat face echoes only where the beam meets it
   square-on, so the fine scan sees a patch of the face, not its ends (the
   19.9 cm container read 11-20 cm wide and centred wherever it stood). The
   face contains the patch and is at least min_width_mm wide, so its left
   end may be as far left as x_r - w and its right end as far right as
   x_l + w: if the measured width is less than w, both edges are moved out
   to those worst cases (the offset, so the side choice, does not change).
   *p_ext_l_mm / *p_ext_r_mm = how far each edge moved (0 = unchanged).
   min_width_mm <= 0: off. */
void  obstacle_apply_min_width(obstacle_profile_t *p_prof, float min_width_mm, float *p_ext_l_mm, float *p_ext_r_mm);

/* Bypass dead reckoning: advance (x right, y forward) by ds_mm on the mean
   of two headings (deg, + = clockwise). */
void  obstacle_pose_advance(float *p_x, float *p_y, float h_prev_deg, float h_deg, float ds_mm);
float obstacle_sin_deg(float deg);

#endif /* OBSTACLE_LOGIC_H */
