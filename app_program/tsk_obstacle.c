/**
 * @file    tsk_obstacle.c
 * @brief   TSK_OBSTACLE_SCAN (DR Section 3 / 6.2): forward monitoring,
 *          coarse + fine scan, obstacle profile, bypass and line
 *          reacquisition (FR9-FR12, NFR1, NFR8, NFR21).
 *
 * STAGE0 (LINE FOLLOWING): servo straight ahead, one ping every ~70 ms,
 *   median-of-3 filter; two filtered readings < OBSTACLE_STOP_CM -> STOP at
 *   once (NFR1) and EVF_OBSTACLE_TRIGGER (TSK_NAV_PLANNER switches to
 *   OBSTACLE AVOIDANCE). Two raw readings < 8 cm -> EVF_PROXIMITY (E-stop).
 * STAGE1 coarse sweep -60..+60 deg, 50 ms settle per step (DR A6), median of
 *   3 pings each; all clear (> 25 cm) -> back to line following.
 * STAGE2 fine scan in 2 deg steps around the obstacle.
 * STAGE3 profile (obstacle_logic.c) -> MBX_OBSTACLE: every echo beside the
 *   obstacle (not its own cluster) narrows that side; of the sides with
 *   room, the one needing the smaller sideways shift, i.e. away from the
 *   obstacle's offset ("no echo" only means room, not more room); shifts
 *   within OBST_SHIFT_TIE_MM (nothing tried yet) -> a 90 deg look each way
 *   (pivot 30 + servo 60), else the smaller shift (L when equal);
 *   no room -> reverse 100 mm and scan again (once). Then reverse only if
 *   the first 45 deg leg would pass the obstacle's near corner closer than
 *   half the car + OBST_MARGIN_MM (obstacle_standoff_mm). Standoff + rescan
 *   reverses per obstacle are capped at OBST_REVERSE_MAX_MM
 *   (obstacle_reverse_budget, OBST_REVERSE_CAP event): a standoff that does
 *   not fit aborts, a rescan is shortened (skipped + abort below 5 mm).
 *   Every straight reverse (back-out, return) is added to g_rev_total_mm.
 * STAGE4 bypass: pivot 45 away, leg d1, pivot back, leg d2 past it, each
 *   waiting for EVF_MOTION_DONE. Leg d2 (28 Oct) follows the obstacle's
 *   side at 60 deg (the servo's limit) and keeps going until the side has
 *   ended, so long obstacles work. Side seen and then seen to end: drive
 *   obstacle_side_trail_mm past the first miss and turn back at once.
 *   Side never seen: before turning back the car pivots 30 deg towards it
 *   and looks square at its side (servo 60), driving on in 100 mm steps
 *   while it is still there, then one obstacle_side_trail90_mm.
 *   OBST_LEGACY = 1: the main-branch behaviour (more clearance first, trail from
 *   the last sighting, always the look + second trail). Every forward leg (and the STAGE5 return)
 *   is watched: the sensor sweeps 0 / 30 / 0 / 60 deg towards the obstacle
 *   side; two pings at the same angle inside the car's path closer than
 *   OBST_LEG_STOP_CM = this side is blocked (an angled or soft obstacle the
 *   scan missed, a second obstacle): stop. On leg 2 or the return, first
 *   recover locally (BYPASS_SIDESTEP / return retry, OBST_SIDESTEP_MAX each);
 *   then back out along the legs to the scan pose and go round the other
 *   side if it was not tried (OBST_LEGACY: only if the scan showed room), else reverse
 *   100 mm and scan again without the blocked side (once). The 8 cm
 *   proximity E-stop stays as the last resort.
 * STAGE5 (28 Oct) the bypass keeps an encoder dead-reckoned pose relative to
 *   the original path; pivot back to 45 deg towards it and drive diagonally
 *   until TSK_LINE_BARCODE reports EVF_LINE_FOUND (re-armed watch), or until
 *   OBST_RETURN_PAST_MM past the path. Found: forward to put the axle on
 *   the line and turn parallel to the path. Not seen: reverse back onto the
 *   path, turn parallel, and line following's search finds the line
 *   (instead of the DR's 300 mm abort) -> EVF_BYPASS_COMPLETE.
 * OBST_DYNAMIC_TURN = 1 (and OBST_LEGACY = 0): standoff_dyn() reverses only
 *   OBST_REVERSE_SCALE of the standoff and picks the smallest first-turn
 *   angle in [OBST_TURN_MIN_DEG, OBST_TURN_MAX_DEG] that clears the corner
 *   (obstacle_dyn_standoff), reversing more only when no angle does;
 *   bypass_plan_dyn() and obstacle_leg_for_shift() size legs 1 / 2 and the
 *   sidestep for that angle. OBST_DYNAMIC_TURN = 0: the original path.
 * Anything failing -> AVOIDANCE_ABORT: stop, EVF_BYPASS_ABORT (-> ERROR).
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "status.h"
#include "motion_api.h"
#include "obstacle_logic.h"
#include "app_params.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define PING_GAP_MS       (70)        /* HC-SR04: > 60 ms between pings */
#define ECHO_WAIT_MS      (30)
#define FINE_HALF_DEG     (30.0f)
#define SCAN_LIMIT_DEG    (60.0f)
#define FINE_EXT_CLUSTER_CM (15.0f)   /* = CLUSTER_CM in obstacle_logic.c */

LOCAL us_filter_t        g_filt;
LOCAL scan_t             g_fine;
LOCAL obstacle_profile_t g_prof;
LOCAL uint32_t           g_prox_n;
LOCAL float              g_servo_now;         /* last commanded robot angle */

#define SERVO_STEP_DEG    (10.0f)     /* ramp the pan in small steps: a 60-120 deg  */
#define SERVO_STEP_MS     (25)        /* jump whips the HC-SR04 and its cable loose */

/* robot angle (+ right) -> servo physical angle */
LOCAL void servo_to(float robot_deg)
{
    float phys = params_servo_phys(robot_deg);          /* SERVO_TRIM_DEG included */

    hal_servo_deg(phys);
    g_servo_now = robot_deg;
}

LOCAL float ping(void)
{
    float cm;

    hal_us_trigger();
    (void)tk_dly_tsk(ECHO_WAIT_MS);
    cm = hal_us_read_cm();
    if ((cm < 0.0f) != fault_has(FAULT_US_NO_ECHO))   /* D1: no-echo bit follows the last ping */
    {
        if (cm < 0.0f)
        {
            fault_set(FAULT_US_NO_ECHO);
        }
        else
        {
            fault_clear(FAULT_US_NO_ECHO);
        }
    }
    st_set_obst_cm(cm);

    /* critical proximity, checked on every ping (2 in a row) */
    g_prox_n = ((cm >= 0.0f) && (cm < (float)PROXIMITY_STOP_CM)) ? (g_prox_n + 1u) : 0u;
    if (g_prox_n >= 2u)
    {
        flag_set(EVF_PROXIMITY);
    }
    return cm;
}

LOCAL float ping_median3(void)
{
    float a = ping();
    float b;
    float c;

    (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
    b = ping();
    (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
    c = ping();
    /* no echo counts as "far" */
    a = (a < 0.0f) ? 1000.0f : a;
    b = (b < 0.0f) ? 1000.0f : b;
    c = (c < 0.0f) ? 1000.0f : c;
    a = median3(a, b, c);
    return (a >= 999.0f) ? -1.0f : a;
}

LOCAL void servo_move(float robot_deg, float from_deg)
{
    float a = g_servo_now;

    (void)from_deg;                           /* the real start is g_servo_now */
    while ((robot_deg - a) > SERVO_STEP_DEG)
    {
        a += SERVO_STEP_DEG;
        servo_to(a);
        (void)tk_dly_tsk(SERVO_STEP_MS);
    }
    while ((a - robot_deg) > SERVO_STEP_DEG)
    {
        a -= SERVO_STEP_DEG;
        servo_to(a);
        (void)tk_dly_tsk(SERVO_STEP_MS);
    }
    servo_to(robot_deg);
    (void)tk_dly_tsk((TMO)SERVO_SETTLE_MS + SERVO_STEP_MS);   /* DR A6 settle */
}

LOCAL bool aborted(void)
{
    return ((flag_peek() & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) ||
           (sys_state() != SYS_OBSTACLE_AVOIDANCE);
}

LOCAL void coarse_sweep(float *p_cm)
{
    float prev = 0.0f;

    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        servo_move(COARSE_DEG[i], prev);
        prev    = COARSE_DEG[i];
        p_cm[i] = ping_median3();
        st_set_scan(i, p_cm[i]);
    }
    servo_move(0.0f, prev);
}

LOCAL void fine_scan(const float *p_coarse)
{
    uint32_t i_near = 2u;
    float    r_near = 1.0e9f;
    float    lo;
    float    hi;
    float    prev = 0.0f;

    for (uint32_t i = 0u; i < COARSE_N; i++)
    {
        if ((p_coarse[i] >= 0.0f) && (p_coarse[i] < r_near))
        {
            r_near = p_coarse[i];
            i_near = i;
        }
    }
    lo = COARSE_DEG[i_near] - FINE_HALF_DEG;
    hi = COARSE_DEG[i_near] + FINE_HALF_DEG;
    lo = (lo < -SCAN_LIMIT_DEG) ? -SCAN_LIMIT_DEG : lo;
    hi = (hi > SCAN_LIMIT_DEG) ? SCAN_LIMIT_DEG : hi;

    g_fine.n = 0u;
    servo_move(lo, 0.0f);
    prev = lo;
    {
        uint32_t steps = (uint32_t)((hi - lo) / (float)FINE_STEP_DEG) + 1u;

        for (uint32_t k = 0u; (k < steps) && (g_fine.n < OBS_MAX_POINTS); k++)
        {
            float a = lo + ((float)k * (float)FINE_STEP_DEG);

            servo_move(a, prev);              /* >= 50 ms, so pings stay > 60 ms apart */
            prev = a;
            g_fine.deg[g_fine.n] = a;
            g_fine.cm[g_fine.n]  = ping();
            g_fine.n++;
        }
    }
    /* 31 Oct: an obstacle wider than the window (its echoes reach the end of
       it, e.g. the container at 15-17 cm fills +-30 deg): keep scanning that
       way in FINE_STEP_DEG steps, up to SCAN_LIMIT_DEG, while the echoes stay
       within the cluster distance, so its edge is measured, not assumed at
       the end of the window */
    {
        float r_min = 1.0e9f;

        for (uint32_t i = 0u; i < g_fine.n; i++)
        {
            if ((g_fine.cm[i] >= 0.0f) && (g_fine.cm[i] < r_min))
            {
                r_min = g_fine.cm[i];
            }
        }
        if (r_min < 1.0e8f)
        {
            float thr = r_min + FINE_EXT_CLUSTER_CM;

            while ((g_fine.n > 0u) && (g_fine.n < OBS_MAX_POINTS) &&
                   (g_fine.cm[g_fine.n - 1u] >= 0.0f) && (g_fine.cm[g_fine.n - 1u] <= thr) &&
                   ((g_fine.deg[g_fine.n - 1u] + (float)FINE_STEP_DEG) <= (SCAN_LIMIT_DEG + 0.01f)))
            {
                float a = g_fine.deg[g_fine.n - 1u] + (float)FINE_STEP_DEG;

                servo_move(a, g_servo_now);
                g_fine.deg[g_fine.n] = a;
                g_fine.cm[g_fine.n]  = ping();
                g_fine.n++;
            }
            while ((g_fine.n > 0u) && (g_fine.n < OBS_MAX_POINTS) &&
                   (g_fine.cm[0] >= 0.0f) && (g_fine.cm[0] <= thr) &&
                   ((g_fine.deg[0] - (float)FINE_STEP_DEG) >= -(SCAN_LIMIT_DEG + 0.01f)))
            {
                float a = g_fine.deg[0] - (float)FINE_STEP_DEG;
                float cm;

                servo_move(a, g_servo_now);
                cm = ping();
                for (uint32_t i = g_fine.n; i > 0u; i--)
                {
                    g_fine.deg[i] = g_fine.deg[i - 1u];
                    g_fine.cm[i]  = g_fine.cm[i - 1u];
                }
                g_fine.deg[0] = a;
                g_fine.cm[0]  = cm;
                g_fine.n++;
            }
        }
    }
    servo_move(0.0f, prev);
}

/* Diagnostics (31 Oct): the fine scan, 11 points per FINE_SCAN event
   ("<first angle>deg+2: cm cm ...", -1 = no echo) */
LOCAL void fine_scan_events(void)
{
    /* FINE_SCAN: first angle S7 (deg), count U4 (1..11), count x CM9
       (laptop: "<first>deg+2: cm cm ...", -1 = no echo) */
    for (uint32_t i0 = 0u; i0 < g_fine.n; i0 += WIRE_FINE_SCAN_MAX)
    {
        evt_b_t  b;
        uint32_t k = ((g_fine.n - i0) > WIRE_FINE_SCAN_MAX) ? WIRE_FINE_SCAN_MAX : (g_fine.n - i0);
        float    a = g_fine.deg[i0];

        evt_begin(&b, EVC_FINE_SCAN);
        evt_s(&b, (a < 0.0f) ? -(int32_t)(0.5f - a) : (int32_t)(a + 0.5f), 7u);
        evt_u(&b, k, 4u);
        for (uint32_t i = i0; i < (i0 + k); i++)
        {
            evt_cm9(&b, g_fine.cm[i]);
        }
        (void)evt_post(&b);
    }
}

/* ---- bypass legs: a log for backing out, and STAGE0-style watching ---- */
#define LEG_LOG_MAX       (16u)       /* 31 Oct: room for the sidestep / return-retry legs */
#define OBST_SIDESTEP_MAX      (2u)   /* local recoveries per leg before backing out to the start */
#define OBST_SIDESTEP_MIN_MM   (40)   /* sidestep at least this far out */
#define OBST_SIDESTEP_EXTRA_MM (30)   /* ... the shortfall plus this */
#define OBST_SIDE_CLOSE_MM     (30)   /* side seen this much closer than planned: sidestep */

typedef enum
{
    LEG_OK = 0,
    LEG_BLOCKED,          /* something ahead closer than OBST_LEG_STOP_CM: back out */
    LEG_SEARCH,           /* back on the original path without seeing the line: line following searches */
    LEG_FAILED            /* E-stop, proximity, timeout, state change */
} leg_res_t;

typedef struct
{
    bool  b_turn;
    float amount;         /* deg (+ clockwise) or mm (+ forward) actually done */
} leg_t;

LOCAL leg_t       g_legs[LEG_LOG_MAX];
LOCAL uint32_t    g_n_legs;
LOCAL float       g_block_cm;           /* distance when a leg was blocked */
LOCAL float       g_block_deg;          /* ... and the sensor angle */

/* leg watch: sensor angles (x the side the obstacle is on), one ping each */
#define WATCH_N           (4u)
LOCAL const float WATCH_DEG[WATCH_N] = { 0.0f, 30.0f, 0.0f, 60.0f };
LOCAL float       g_backed_mm;          /* standoff + rescan reversed so far in this avoidance (capped) */
LOCAL float       g_rev_total_mm;       /* every straight reverse of this avoidance (odometry) */
LOCAL float       g_rev_cap;            /* OBST_REVERSE_MAX_MM, read at the start of avoid() */
LOCAL bool        g_legacy;             /* OBST_LEGACY, read at the start of avoid() */
LOCAL bool        g_dyn;                /* OBST_DYNAMIC_TURN and not OBST_LEGACY, read at the start of avoid() */
LOCAL float       g_ext_mm[2];          /* how far obstacle_apply_min_width moved the left / right edge */
LOCAL float       g_seen_extra_mm;      /* ... on the side being passed: widens the seen / beside limits */
LOCAL float       g_half_obs;           /* half width on the side facing the obstacle (ROBOT_LEFT_HALF_MM
                                           when passing right), for the plan, the trails and the leg watch */
LOCAL obstacle_profile_t g_view;        /* the profile as the side choice sees it (see side_view) */

LOCAL float left_half_mm(void)
{
    return (g_params.robot_left_half_mm > 0.0f) ? g_params.robot_left_half_mm : g_params.robot_half_width_mm;
}

/* 31 Oct: the car is wider on the left (barcode reader). The side choice
   works on a copy of the profile in which passing right (the obstacle on
   the car's left) needs E = left half - half more shift, and either side
   needs E more room. */
LOCAL void side_view(obstacle_profile_t *p_q)
{
    float e_cm = (left_half_mm() - g_params.robot_half_width_mm) / 10.0f;

    *p_q = g_prof;
    if (g_legacy)
    {
        return;
    }
    p_q->x_r_cm         += e_cm;
    p_q->clear_left_cm  -= e_cm;
    p_q->clear_right_cm -= e_cm;
}
LOCAL abort_reason_t g_why;             /* why run_bypass() failed */

/* Dead reckoning from the encoders, relative to where the bypass started
   (on the line, heading along it): x + = right of the original path,
   y + = along it, h = heading (deg, + = clockwise). Updated in step(). */
typedef struct
{
    float x_mm;
    float y_mm;
    float h_deg;
    float h0_deg;
    float d_prev;
} pose_t;

LOCAL pose_t g_pose;


/* Diagnostics (31 Oct, no effect on the motion): what side-follow measured
   at 60 deg (pings, how many hit the side, the nearest echo against the
   "seen" limit, leg length, lateral distance from the original path) and
   what the pivot look saw. */
LOCAL void bypass_extend_event(uint32_t kind, float cm);

LOCAL void side_follow_event(uint32_t n60, uint32_t n_seen, float near_cm, float leg_mm, float gap_mm)
{
    /* SIDE_FOLLOW (design D10): pings U8, seen U8, near CM13 (0.1 cm,
       8191 = no echo, the baseline's "near -1.0cm"), limit U13 (0.1 cm,
       held below the 8191 code), leg U11 (mm), x U11 (mm); each saturates */
    evt_b_t b;
    float   x = (g_pose.x_mm < 0.0f) ? -g_pose.x_mm : g_pose.x_mm;
    float   lim = ((gap_mm + 150.0f) / 8.660254f) * 10.0f;

    evt_begin(&b, EVC_SIDE_FOLLOW);
    evt_u(&b, n60, 8u);
    evt_u(&b, n_seen, 8u);
    evt_cm13(&b, near_cm);
    evt_u(&b, (lim > 8190.0f) ? 8190u : ((lim > 0.0f) ? (uint32_t)(lim + 0.5f) : 0u), 13u);
    evt_u(&b, (leg_mm > 0.0f) ? (uint32_t)(leg_mm + 0.5f) : 0u, 11u);
    evt_u(&b, (uint32_t)(x + 0.5f), 11u);
    (void)evt_post(&b);
}

LOCAL void side_look_event(float d90, float d60, bool b_gone)
{
    /* SIDE_LOOK: d90 CM13, d60 CM13, gone U1 */
    evt_b_t b;

    evt_begin(&b, EVC_SIDE_LOOK);
    evt_cm13(&b, d90);
    evt_cm13(&b, d60);
    evt_u(&b, b_gone ? 1u : 0u, 1u);
    (void)evt_post(&b);
}

LOCAL void pose_reset(void)
{
    odometry_t o;

    odometry_get(&o);
    g_pose.x_mm   = 0.0f;
    g_pose.y_mm   = 0.0f;
    g_pose.h_deg  = 0.0f;
    g_pose.h0_deg = o.heading_deg;
    g_pose.d_prev = o.dist_mm;
}

LOCAL void pose_update(void)
{
    odometry_t o;
    float      d;
    float      h;

    odometry_get(&o);
    d = o.dist_mm;
    h = o.heading_deg - g_pose.h0_deg;
    obstacle_pose_advance(&g_pose.x_mm, &g_pose.y_mm, g_pose.h_deg, h, d - g_pose.d_prev);
    g_pose.h_deg  = h;
    g_pose.d_prev = d;
}

LOCAL float odo_fwd_mm(void)
{
    odometry_t o;

    odometry_get(&o);
    return o.dist_mm;
}

/* Runs one primitive; pings ahead while it moves (forward legs only).
   watch_sign != 0 (bypass legs): the sensor sweeps 0 / 30 / 0 / 60 deg
   towards the obstacle side (+1 = right) and a ping inside the car's path
   (obstacle_in_path: car width + OBST_PATH_SIDE_MM, closer than
   OBST_LEG_STOP_CM ahead), confirmed by a second ping at the same angle,
   stops it: LEG_BLOCKED. This sees an obstacle edge coming at the car's
   front corner, which a straight-ahead ping cannot. stop_bits (e.g.
   EVF_LINE_FOUND) end it early with LEG_OK and *p_stop_hit set. */
LOCAL leg_res_t step(uint16_t id, bool b_watch_ahead, float watch_sign, uint32_t stop_bits, uint32_t *p_stop_hit)
{
    uint32_t t0 = hal_time_ms();
    bool     b_sweep = b_watch_ahead && (watch_sign != 0.0f) && (g_params.obst_leg_stop_cm > 0.0f);
    uint32_t k = 0u;

    *p_stop_hit = 0u;
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        bool            b_done;

        b_done = motion_wait(id, stop_bits | EVF_ESTOP | EVF_PROXIMITY, b_sweep ? 10u : (uint32_t)PING_GAP_MS, &res, &other);
        pose_update();
        if (b_done)
        {
            return (res == MOTION_OK) ? LEG_OK : LEG_FAILED;
        }
        if ((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u)
        {
            return LEG_FAILED;
        }
        if ((other & stop_bits) != 0u)
        {
            *p_stop_hit = other & stop_bits;
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            return LEG_OK;
        }
        if (sys_state() != SYS_OBSTACLE_AVOIDANCE)
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }
        if (b_sweep)
        {
            float a = watch_sign * WATCH_DEG[k % WATCH_N];
            float cm;

            k++;
            servo_move(a, g_servo_now);       /* >= 75 ms: pings stay > 60 ms apart */
            cm = ping();
            if (obstacle_in_path(cm, a, g_half_obs, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
            {
                (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
                cm = ping();                  /* NFR8: one echo is not enough */
                if (obstacle_in_path(cm, a, g_half_obs, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
                {
                    motion_stop(SRC_OBSTACLE);
                    g_block_cm  = cm;
                    g_block_deg = a;
                    (void)tk_dly_tsk(300);    /* come to rest before the distance is read */
                    pose_update();
                    return LEG_BLOCKED;
                }
            }
        }
        else if (b_watch_ahead)
        {
            (void)ping();
        }
        else
        {
            /* turns, reversing: no pings */
        }
        if ((hal_time_ms() - t0) > 30000u)
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }
    }
}

LOCAL void leg_log(bool b_turn, float amount)
{
    if (g_n_legs < LEG_LOG_MAX)
    {
        g_legs[g_n_legs].b_turn = b_turn;
        g_legs[g_n_legs].amount = amount;
        g_n_legs++;
    }
}

/* pivot, logged for backing out */
LOCAL leg_res_t leg_turn(float deg)
{
    uint32_t  hit;
    leg_res_t r = step(motion_start_turn(deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);

    if (r == LEG_OK)
    {
        leg_log(true, deg);
    }
    return r;
}

/* forward leg, watched (sweep towards watch_sign), logged (the distance really driven) */
LOCAL leg_res_t leg_move(float mm, float watch_sign, uint32_t stop_bits, uint32_t *p_hit)
{
    float     s0 = odo_fwd_mm();
    leg_res_t r  = step(motion_start_move(mm, g_params.bypass_mm_s, SRC_OBSTACLE), true, watch_sign, stop_bits, p_hit);

    if (r != LEG_FAILED)
    {
        leg_log(false, odo_fwd_mm() - s0);
    }
    return r;
}

/* adds the distance driven since odometry read s0 to g_rev_total_mm */
LOCAL void rev_count(float s0)
{
    float d = odo_fwd_mm() - s0;

    g_rev_total_mm += (d < 0.0f) ? -d : d;
}

/* straight back (not logged: used before a scan, the scan pose moves).
   Standoff and rescan only: the caller checks obstacle_reverse_budget. */
LOCAL bool reverse_mm(float mm)
{
    uint32_t hit;
    float    s0 = odo_fwd_mm();
    bool     b_ok = step(motion_start_move(-mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) == LEG_OK;

    rev_count(s0);
    if (b_ok)
    {
        g_backed_mm += mm;
    }
    return b_ok;
}

/* undo the logged legs in reverse order: back to the pose of the last scan,
   along the path just driven (the only ground known to be free: no rear sensor).
   Always driven in full (no reverse budget), but counted in g_rev_total_mm. */
LOCAL bool undo_legs(uint32_t keep)
{
    uint32_t hit;

    while (g_n_legs > keep)
    {
        const leg_t *p_l = &g_legs[--g_n_legs];
        leg_res_t    r   = LEG_OK;

        if (p_l->b_turn)
        {
            r = step(motion_start_turn(-p_l->amount, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
        }
        else if ((p_l->amount > 2.0f) || (p_l->amount < -2.0f))
        {
            float s0 = odo_fwd_mm();

            r = step(motion_start_move(-p_l->amount, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
            rev_count(s0);
        }
        else
        {
            /* blocked at once: nothing to undo */
        }
        if (r != LEG_OK)
        {
            return false;
        }
    }
    return true;
}

/* STAGE4 leg 2, parallel to the path (28 Oct). The servo cannot look 90 deg
   sideways, so the obstacle's side is followed at 60 deg: the leg drives at
   least min_mm (the plan, with OBST_DEPTH_MM), then on while the 60 deg
   ping still hits the side (obstacle_side_seen), and, once at least 2
   60 deg pings in a row have missed it, trail_mm past the first of those
   misses (obstacle_side_trail_mm; OBST_LEGACY: past the last sighting), at
   most OBST_SIDE_MAX_MM (then: LEG_BLOCKED, the side never ended). Pings
   alternate 0 / 60 deg and anything inside the car's path stops it, as on
   the other legs. *p_ended = LEG_OK after the side was seen and then seen
   to end (the 45 deg turn back is then clear without a further look). */
LOCAL leg_res_t leg_side_follow(float min_mm, float gap_mm, float trail_mm, float ws, bool *p_ended)
{
    float     s0 = odo_fwd_mm();
    float     seen_at = -1.0f;              /* leg distance where the side was last seen */
    float     miss_at = -1.0f;              /* ... and where the first 60 deg ping after that missed it */
    uint32_t  clear_n = 0u;
    bool      b_trust = !g_legacy && prm_flag(PRM_OBST_TRUST_MISS);   /* OBST_TRUST_MISS */
    uint32_t  n60 = 0u;                   /* diagnostics: 60 deg pings, */
    uint32_t  n_seen = 0u;                /* ... how many hit the side, */
    float     near_cm = -1.0f;            /* ... the nearest echo */
    uint32_t  k = 0u;
    uint32_t  t0 = hal_time_ms();
    uint16_t  id = motion_start_move((float)OBST_SIDE_MAX_MM, g_params.bypass_mm_s, SRC_OBSTACLE);
    leg_res_t r;

    *p_ended = false;
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        float           a;
        float           cm;
        float           done;
        float           ref;

        if (motion_wait(id, EVF_ESTOP | EVF_PROXIMITY, 10u, &res, &other))
        {
            pose_update();
            if (res != MOTION_OK)
            {
                return LEG_FAILED;
            }
            g_block_cm  = (float)OBST_SIDE_MAX_MM / 10.0f;
            g_block_deg = 999.0f;               /* marker: the side did not end */
            r = LEG_BLOCKED;
            break;
        }
        pose_update();
        if (((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u) || (sys_state() != SYS_OBSTACLE_AVOIDANCE) ||
            ((hal_time_ms() - t0) > 40000u))
        {
            motion_stop(SRC_OBSTACLE);
            return LEG_FAILED;
        }

        a = ((k % 2u) == 0u) ? 0.0f : (ws * 60.0f);
        k++;
        servo_move(a, g_servo_now);
        cm = ping();
        if ((g_params.obst_leg_stop_cm > 0.0f) &&
            obstacle_in_path(cm, a, g_half_obs, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
        {
            (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
            cm = ping();
            if (obstacle_in_path(cm, a, g_half_obs, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
            {
                motion_stop(SRC_OBSTACLE);
                g_block_cm  = cm;
                g_block_deg = a;
                (void)tk_dly_tsk(300);
                pose_update();
                r = LEG_BLOCKED;
                break;
            }
        }
        done = odo_fwd_mm() - s0;
        if (a != 0.0f)
        {
            n60++;
            if ((cm >= 0.0f) && ((near_cm < 0.0f) || (cm < near_cm)))
            {
                near_cm = cm;
            }
            if (obstacle_side_seen(cm, gap_mm + g_seen_extra_mm))
            {
                seen_at = done;
                n_seen++;
                /* 31 Oct: the side is closer than planned: stop, run_bypass sidesteps */
                if (!g_legacy && ((cm * 8.660254f) < (gap_mm - (float)OBST_SIDE_CLOSE_MM)))
                {
                    motion_stop(SRC_OBSTACLE);
                    g_block_cm  = cm;
                    g_block_deg = a;
                    (void)tk_dly_tsk(300);
                    pose_update();
                    r = LEG_BLOCKED;
                    break;
                }
                miss_at = -1.0f;
                clear_n = 0u;
            }
            else
            {
                if ((clear_n == 0u) && ((seen_at >= 0.0f) || (b_trust && (miss_at < 0.0f))))
                {
                    miss_at = done;             /* the corner has left the beam by here */
                }
                clear_n++;
            }
        }
        ref = g_legacy ? seen_at : miss_at;
        /* OBST_TRUST_MISS: two misses in a row end the leg trail_mm after the first
           one, seen before or not; otherwise the planned leg (min_mm) unless a
           sighting was followed by two misses */
        if (b_trust ? ((clear_n >= 2u) && (miss_at >= 0.0f) && (done >= (miss_at + trail_mm + ((seen_at >= 0.0f) ? 0.0f : (gap_mm * 0.3094f)))))
                    : ((done >= min_mm) && ((seen_at < 0.0f) || ((clear_n >= 2u) && (done >= (ref + trail_mm))))))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            r = LEG_OK;
            break;
        }
    }
    leg_log(false, odo_fwd_mm() - s0);
    *p_ended = (r == LEG_OK) && ((seen_at >= 0.0f) || b_trust);
    side_follow_event(n60, n_seen, near_cm, odo_fwd_mm() - s0, gap_mm + g_seen_extra_mm);
    return r;
}

/* Before the 45 deg turn back (28 Oct). The servo stops at 60 deg and a
   flat side often gives no echo at 60, so the car pivots OBST_LOOK_PIVOT_DEG
   towards the obstacle (the way it is about to turn anyway) and pings with
   the servo at 60 (square to the side) and 30, then pivots back. Side still
   there: drive on OBST_SIDE_STEP_MM (watched) and look again, at most
   OBST_TURN_LOOK_STEPS times (then LEG_BLOCKED: the side did not end).
   Side gone: drive on obstacle_side_trail90_mm, then the turn is clear. */
LOCAL leg_res_t look_before_turn(float ws, float gap_mm, float margin_mm)
{
    uint32_t hit = 0u;
    float    piv = (float)OBST_LOOK_PIVOT_DEG;

    for (uint32_t n = 0u; ; n++)
    {
        float     d90;
        float     d60;
        leg_res_t r;
        bool      b_gone;

        if (step(motion_start_turn(ws * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
        {
            return LEG_FAILED;
        }
        servo_move(ws * 60.0f, g_servo_now);
        d90 = ping_median3();
        servo_move(ws * 30.0f, g_servo_now);
        d60 = ping_median3();
        servo_move(0.0f, g_servo_now);
        if ((step(motion_start_turn(-ws * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK) || aborted())
        {
            return LEG_FAILED;
        }
        b_gone = !obstacle_side_beside(d90, gap_mm + g_seen_extra_mm) &&
                 !((d60 >= 0.0f) && ((d60 * 10.0f * 0.8660254f) <= ((gap_mm + g_seen_extra_mm - 40.0f) + 100.0f)));
        side_look_event(d90, d60, b_gone);
        if (b_gone)
        {
            return leg_move(obstacle_side_trail90_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_half_obs, margin_mm),
                            ws, 0u, &hit);
        }
        if (n >= (uint32_t)OBST_TURN_LOOK_STEPS)
        {
            g_block_cm  = (float)OBST_SIDE_MAX_MM / 10.0f;
            g_block_deg = 999.0f;                   /* the side did not end */
            return LEG_BLOCKED;
        }
        bypass_extend_event(0u, (d90 >= 0.0f) ? d90 : d60);   /* "obstacle still beside the car (X cm): 100 mm further" */
        r = leg_move((float)OBST_SIDE_STEP_MM, ws, 0u, &hit);
        if (r != LEG_OK)
        {
            return r;
        }
    }
}

/* STAGE4 + STAGE5 for one side. The turns back use the measured heading
   (pose), and the return is aimed at the ORIGINAL PATH, not a fixed 300 mm:
   it drives until it is OBST_RETURN_PAST_MM past the path, stopping as soon
   as a line sensor sees the line. If it got past the path without seeing
   the line, it reverses back onto the path, turns parallel to it and hands
   over to line following, whose search finds the line (LEG_SEARCH). */
LOCAL leg_res_t run_bypass(const bypass_plan_t *p_plan, float gap_mm, float margin_mm)
{
    uint32_t  hit = 0u;
    leg_res_t r;
    float     ws = -(float)p_plan->side;      /* the obstacle is on the other side of the car */
    float     toward;
    float     s;
    float     dist;
    bool      b_ended = false;

    g_n_legs = 0u;
    g_why    = ABR_LEG_FAILED;
    pose_reset();
    if (((r = leg_turn(p_plan->turn1_deg)) != LEG_OK) ||
        ((r = leg_move(p_plan->d1_mm, ws, 0u, &hit)) != LEG_OK) ||
        ((r = leg_turn(-g_pose.h_deg)) != LEG_OK))                    /* parallel to the path */
    {
        return r;
    }
    /* leg 2. 31 Oct: the side is closer than planned, or the leg is blocked
       beside it: back along this leg only, step out and carry on, instead of
       backing out to the start (OBST_SIDESTEP_MAX times, then as before) */
    for (uint32_t n_side = 0u; ; n_side++)
    {
        r = leg_side_follow(p_plan->d2_mm, gap_mm,
                            obstacle_side_trail_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_half_obs, margin_mm),
                            ws, &b_ended);
        if ((r == LEG_BLOCKED) && !g_legacy && (n_side < OBST_SIDESTEP_MAX) && (g_block_deg < 500.0f) && (g_n_legs > 0u))
        {
            float deg = (g_block_deg < 0.0f) ? -g_block_deg : g_block_deg;
            float lat = g_block_cm * 10.0f * obstacle_sin_deg(deg);
            float out = (deg < 1.0f) ? (float)OBST_RETRY_WIDER_MM : ((gap_mm - lat) + (float)OBST_SIDESTEP_EXTRA_MM);

            out = (out < (float)OBST_SIDESTEP_MIN_MM) ? (float)OBST_SIDESTEP_MIN_MM : out;
            (void)evt_post1(EVC_BYPASS_SIDESTEP, (out > 0.0f) ? (uint32_t)(out + 0.5f) : 0u, 10u);   /* "too close beside the obstacle: back, then X mm further out" */
            if (!undo_legs(g_n_legs - 1u))
            {
                return LEG_FAILED;
            }
            if (((r = leg_turn(p_plan->turn1_deg)) != LEG_OK) ||
                ((r = leg_move(g_dyn ? obstacle_leg_for_shift(out, (float)p_plan->side * p_plan->turn1_deg)   /* OBST_DYNAMIC_TURN */
                                     : (out / 0.7071068f), ws, 0u, &hit)) != LEG_OK) ||
                ((r = leg_turn(-g_pose.h_deg)) != LEG_OK))
            {
                return r;
            }
            gap_mm += out;
            continue;
        }
        if (r != LEG_OK)
        {
            return r;
        }
        break;
    }
    /* side seen and seen to end: the trail already clears the rear corner */
    if ((g_legacy || !b_ended) &&
        ((r = look_before_turn(ws, gap_mm, margin_mm)) != LEG_OK))
    {
        return r;
    }

    /* STAGE5: 45 deg back towards the original path (x = 0). 31 Oct: blocked
       on the way back (the back corner is further ahead): back to where it
       turned, OBST_SIDE_STEP_MM further on, turn again (OBST_SIDESTEP_MAX) */
    for (uint32_t n_ret = 0u; ; n_ret++)
    {
        uint32_t n0 = g_n_legs;

        toward = (g_pose.x_mm < 0.0f) ? 1.0f : -1.0f;  /* left of it: turn right */
        if ((r = leg_turn((toward * 45.0f) - g_pose.h_deg)) != LEG_OK)
        {
            return r;
        }
        s    = obstacle_sin_deg((g_pose.h_deg < 0.0f) ? -g_pose.h_deg : g_pose.h_deg);
        s    = (s < 0.3f) ? 0.3f : s;
        dist = (((g_pose.x_mm < 0.0f) ? -g_pose.x_mm : g_pose.x_mm) + (float)OBST_RETURN_PAST_MM) / s;
        r    = LEG_OK;
        if (g_dyn)                                      /* OBST_DYNAMIC_TURN: return gate */
        {
            /* 4 Nov: a dark mark early on the return was taken as the line.
               Drive (watching for obstacles, not for the line) until the
               dead-reckoned offset is within the gate window, then re-arm
               the line watch for the rest of the leg. */
            float ax  = (g_pose.x_mm < 0.0f) ? -g_pose.x_mm : g_pose.x_mm;
            float win = OBST_RETURN_GATE_FRAC * ax;
            float blind;

            win   = (win < (float)OBST_RETURN_GATE_MIN_MM) ? (float)OBST_RETURN_GATE_MIN_MM : win;
            blind = (ax > win) ? ((ax - win) / s) : 0.0f;
            if (blind > 1.0f)
            {
                r     = leg_move(blind, ws, 0u, &hit);
                dist -= blind;
            }
        }
        if (r == LEG_OK)
        {
            g_reacq_seq++;                              /* TSK_LINE: fresh line watch */
            (void)tk_dly_tsk(30);
            flag_clear(EVF_LINE_FOUND);
            r = leg_move(dist, ws, EVF_LINE_FOUND, &hit);
        }
        if ((r == LEG_BLOCKED) && !g_legacy && (n_ret < OBST_SIDESTEP_MAX))
        {
            bypass_extend_event(1u, g_block_cm);   /* "return blocked (X cm): back to the turn, 100 mm further" */
            if (!undo_legs(n0))
            {
                return LEG_FAILED;
            }
            if ((r = leg_move((float)OBST_SIDE_STEP_MM, ws, 0u, &hit)) != LEG_OK)
            {
                return r;
            }
            continue;
        }
        if (r != LEG_OK)
        {
            return r;
        }
        break;
    }
    if ((hit & EVF_LINE_FOUND) != 0u)
    {
        /* axle onto the line, then parallel to the original path */
        if ((step(motion_start_move(g_params.ir_to_axle_mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK) ||
            (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK))
        {
            g_why = ABR_REALIGN;
            return LEG_FAILED;
        }
        if (g_dyn && (OBST_REACQ_SWEEP_DEG > 0.0f))   /* OBST_DYNAMIC_TURN: confirm the line */
        {
            (void)tk_dly_tsk(30);
            /* 4 Nov logs: with LINE_DO_BLACK_HIGH 0 (inverted on this car) the
               left sensor reported 1 when it was NOT on the line. Since 5 Nov
               the sense is right (LINE_DO_BLACK_HIGH 1): 0 = not on the line */
            if (!st_line_l())
            {
                /* 4 Nov: parallel but off the line. Pivot towards the path
                   side first (the usual miss is short), then the other way;
                   stop where a sensor finds the line and leave the small
                   angle to line following. Nothing: back to parallel. */
                static const float SWEEP[2] = { 1.0f, -2.0f };

                hit = 0u;
                for (uint32_t k = 0u; (k < 2u) && ((hit & EVF_LINE_FOUND) == 0u); k++)
                {
                    g_reacq_seq++;                  /* TSK_LINE: fresh line watch */
                    (void)tk_dly_tsk(30);
                    flag_clear(EVF_LINE_FOUND);
                    if (step(motion_start_turn(SWEEP[k] * toward * OBST_REACQ_SWEEP_DEG, g_params.turn_mm_s, SRC_OBSTACLE),
                             false, 0.0f, EVF_LINE_FOUND, &hit) != LEG_OK)
                    {
                        g_why = ABR_REALIGN;
                        return LEG_FAILED;
                    }
                }
                if (((hit & EVF_LINE_FOUND) == 0u) &&
                    (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK))
                {
                    g_why = ABR_REALIGN;
                    return LEG_FAILED;
                }
            }
        }
        return LEG_OK;
    }

    /* past the original path and no line seen: back onto the path (watching
       for the line again), parallel to it, and let line following search */
    {
        float past = toward * g_pose.x_mm;              /* > 0: beyond the path */

        (void)evt_post0(EVC_BYPASS_RETURN);   /* "line not seen crossing the original path: back to it, then search" */
        flag_clear(EVF_LINE_FOUND);
        if (past > 5.0f)
        {
            float     s0 = odo_fwd_mm();
            leg_res_t rr = step(motion_start_move(-past / s, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, EVF_LINE_FOUND, &hit);

            rev_count(s0);                              /* in full, not budgeted, but counted */
            if (rr != LEG_OK)
            {
                g_why = ABR_REVERSE_PATH;
                return LEG_FAILED;
            }
        }
        if (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
        {
            g_why = ABR_REALIGN;
            return LEG_FAILED;
        }
    }
    return LEG_SEARCH;
}

/* OBSTACLE_PROFILE: r CM9, width U9 (cm), offset S9 (cm), clear left S11 (cm),
   clear right S11 (cm), side E2 (0 none, 1 L, 2 R) */
LOCAL int32_t round_cm(float x)
{
    return (x < 0.0f) ? -(int32_t)(0.5f - x) : (int32_t)(x + 0.5f);
}

LOCAL void profile_event(const obstacle_profile_t *p)
{
    evt_b_t b;
    int32_t w = round_cm(p->width_cm);

    evt_begin(&b, EVC_OBSTACLE_PROFILE);
    evt_cm9(&b, p->r_min_cm);
    /* D10: r CM9, w U10 cm, off S10, clrL S10, clrR S10, side E2 */
    evt_u(&b, (w > 0) ? (uint32_t)w : 0u, 10u);
    evt_s(&b, round_cm(p->offset_cm), 10u);
    evt_s(&b, round_cm(p->clear_left_cm), 10u);
    evt_s(&b, round_cm(p->clear_right_cm), 10u);
    evt_u(&b, (p->side == 'L') ? 1u : ((p->side == 'R') ? 2u : 0u), 2u);
    (void)evt_post(&b);
}

LOCAL void abort_avoidance(abort_reason_t why)
{
    motion_stop(SRC_OBSTACLE);
    servo_move(0.0f, 0.0f);
    fault_set(FAULT_BYPASS_ABORT);
    (void)evt_post1(EVC_AVOIDANCE_ABORT, (uint32_t)why, 4u);
    flag_set(EVF_BYPASS_ABORT);
}

/* BYPASS_EXTEND: kind U1 (0 "obstacle still beside the car", 1 "return
   blocked"), distance CM9 */
LOCAL void bypass_extend_event(uint32_t kind, float cm)
{
    evt_b_t b;

    evt_begin(&b, EVC_BYPASS_EXTEND);
    evt_u(&b, kind, 1u);
    evt_cm9(&b, cm);
    (void)evt_post(&b);
}

/* After the scan (and before each bypass attempt): reverse only if the
   first 45 deg leg would pass the near corner closer than the car's half
   width + OBST_MARGIN_MM (obstacle_standoff_mm). The profile is moved with
   the car, so the plan and a later back-out stay consistent. A standoff
   that does not fit the reverse budget (OBST_REVERSE_MAX_MM) is not
   shortened (a short one could clip the near corner): STANDOFF_CAPPED. */
typedef enum
{
    STANDOFF_OK = 0,
    STANDOFF_FAILED,      /* the reverse failed (E-stop, proximity, timeout) */
    STANDOFF_CAPPED       /* refused by the reverse cap, the car did not move */
} standoff_res_t;

/* OBST_REVERSE_CAP: kind E2 (0 "standoff N mm refused", 1 "rescan skipped",
   2 "rescan reverse cut to N mm"), N U10, used U10, cap U10, total U12 (mm, D10):
   "<kind text>: U of C mm used, T mm total" */
LOCAL uint32_t mm_u(float mm)
{
    return (mm > 0.0f) ? (uint32_t)(mm + 0.5f) : 0u;
}

LOCAL void rev_cap_event(uint32_t kind, float x)
{
    evt_b_t b;

    evt_begin(&b, EVC_OBST_REVERSE_CAP);
    evt_u(&b, kind, 2u);
    evt_u(&b, mm_u(x), 10u);
    evt_u(&b, mm_u(g_backed_mm), 10u);
    evt_u(&b, mm_u(g_rev_cap), 10u);
    evt_u(&b, mm_u(g_rev_total_mm), 12u);
    (void)evt_post(&b);
}

LOCAL standoff_res_t standoff(char side, float margin_mm)
{
    float need = obstacle_standoff_mm(&g_prof, side, g_half_obs, margin_mm) -
                 (g_prof.r_min_cm * 10.0f);
    float allowed;

    if (need < 5.0f)
    {
        return STANDOFF_OK;
    }
    if (need > (float)OBST_BACKOFF_MAX_MM)
    {
        need = (float)OBST_BACKOFF_MAX_MM;
    }
    if (obstacle_reverse_budget(need, g_backed_mm, g_rev_cap, false, &allowed) == OBST_REV_REFUSED)
    {
        rev_cap_event(0u, need);
        return STANDOFF_CAPPED;
    }
    {
        /* OBSTACLE_BACKOFF: mm U8 (<= OBST_BACKOFF_MAX_MM = 250), side U1 (0 L, 1 R):
           "reversing N mm: a 45 deg turn would clip the left/right corner" */
        evt_b_t b;

        evt_begin(&b, EVC_OBSTACLE_BACKOFF);
        evt_u(&b, mm_u(need), 8u);
        evt_u(&b, (side == 'L') ? 0u : 1u, 1u);
        (void)evt_post(&b);
    }
    if (!reverse_mm(need))
    {
        return STANDOFF_FAILED;
    }
    g_prof.r_min_cm += need / 10.0f;
    return STANDOFF_OK;
}

/* ---- OBST_DYNAMIC_TURN (ultrasonic-reverse-turn-refinement): begin ---- */
/* The scaled standoff + first-turn angle (obstacle_dyn_standoff), driven as
   one reverse checked once against the reverse budget. 'base_margin_mm' is
   the margin without the retry extra (the pivot-swing gap). An invalid plan
   falls back to standoff() with theta = 45. */
LOCAL standoff_res_t standoff_dyn(char side, float margin_mm, float base_margin_mm, float *p_theta)
{
    obst_dyn_cfg_t  cfg;
    obst_dyn_plan_t dp;
    float           t;
    float           allowed;

    cfg.scale           = OBST_REVERSE_SCALE;
    cfg.min_deg         = OBST_TURN_MIN_DEG;
    cfg.max_deg         = OBST_TURN_MAX_DEG;
    cfg.cap_mm          = (float)OBST_BACKOFF_MAX_MM;
    cfg.us_ahead_mm     = (float)OBST_US_AHEAD_MM;
    cfg.swing_margin_mm = base_margin_mm;
    if (!obstacle_dyn_standoff(&g_prof, side, g_half_obs, margin_mm, &cfg, &dp))
    {
        *p_theta = 45.0f;
        return standoff(side, margin_mm);
    }
    t = dp.reverse_mm;
    if (t > 0.0f)
    {
        if (obstacle_reverse_budget(t, g_backed_mm, g_rev_cap, false, &allowed) == OBST_REV_REFUSED)
        {
            if (t >= 5.0f)
            {
                rev_cap_event(0u, t);
                return STANDOFF_CAPPED;
            }
            t = 0.0f;              /* skip, keep theta: shortfall < 5 mm, as the original skip */
        }
    }
    if (t > 0.0f)
    {
        /* OBSTACLE_BACKOFF: same format as standoff() */
        evt_b_t b;

        evt_begin(&b, EVC_OBSTACLE_BACKOFF);
        evt_u(&b, mm_u(t), 8u);
        evt_u(&b, (side == 'L') ? 0u : 1u, 1u);
        (void)evt_post(&b);
        if (!reverse_mm(t))
        {
            return STANDOFF_FAILED;
        }
        g_prof.r_min_cm += t / 10.0f;
    }
    *p_theta = dp.theta_deg;
    return STANDOFF_OK;
}
/* ---- OBST_DYNAMIC_TURN (ultrasonic-reverse-turn-refinement): end ---- */

/* The rescan reverse (OBST_RESCAN_REVERSE_MM), within the reverse budget:
   shortened to what is left (OBST_REVERSE_CAP), or skipped when less than
   OBST_REVERSE_MIN_MM is left (OBST_REVERSE_CAP, AVOIDANCE_ABORT, no
   OBSTACLE_RESCAN). false = the avoidance was aborted. */
LOCAL bool rescan_reverse(uint32_t rescan_kind)
{
    float      allowed;
    obst_rev_t rv = obstacle_reverse_budget((float)OBST_RESCAN_REVERSE_MM, g_backed_mm, g_rev_cap, true, &allowed);

    if (rv == OBST_REV_REFUSED)
    {
        rev_cap_event(1u, 0.0f);
        abort_avoidance(ABR_REVERSE_LIMIT);
        return false;
    }
    (void)evt_post1(EVC_OBSTACLE_RESCAN, rescan_kind, 1u);   /* 0 "no room", 1 "blocked": "...: reversing 100 mm and scanning again" */
    if (rv == OBST_REV_SHORTENED)
    {
        rev_cap_event(2u, allowed);
    }
    if (!reverse_mm(allowed))
    {
        abort_avoidance(ABR_REVERSE_FAILED);
        return false;
    }
    return true;
}

/* both sides have room, nothing tried yet, and about the same sideways
   shift (obstacle_is_tie; OBST_LEGACY: about the same clearance) */
LOCAL bool is_tie(float half, float margin, const uint32_t *p_tries)
{
    return (p_tries[0] == 0u) && (p_tries[1] == 0u) &&
           obstacle_is_tie(&g_view, half, margin, g_legacy ? OBST_CHOOSE_LEGACY : 0u);
}

/* Tie (28 Oct): instead of defaulting to the smaller detour, look 90 deg
   each way - the servo only reaches 60, so the car pivots OBST_TIE_PIVOT_DEG
   and the servo adds 60 - and take the side with more room there. Back to
   the starting heading afterwards (these pivots are not bypass legs). */
LOCAL char tie_look(char side)
{
    uint32_t hit;
    float    piv = (float)OBST_TIE_PIVOT_DEG;
    float    d_l;
    float    d_r;
    evt_b_t  b;

    if (step(motion_start_turn(-piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
    {
        return side;
    }
    servo_move(-60.0f, g_servo_now);
    d_l = ping_median3();
    if ((step(motion_start_turn(2.0f * piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK))
    {
        return side;
    }
    servo_move(60.0f, g_servo_now);
    d_r = ping_median3();
    (void)step(motion_start_turn(-piv, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
    servo_move(0.0f, g_servo_now);

    d_l = (d_l < 0.0f) ? 200.0f : d_l;      /* no echo = open */
    d_r = (d_r < 0.0f) ? 200.0f : d_r;
    if (d_l > (d_r + OBST_SIDE_TIE_CM))
    {
        side = 'L';
    }
    else if (d_r > (d_l + OBST_SIDE_TIE_CM))
    {
        side = 'R';
    }
    else
    {
        /* still equal: keep the smaller detour */
    }
    /* OBSTACLE_TIE: left U9 (cm), right U9 (cm), side E2 (1 L, 2 R) */
    evt_begin(&b, EVC_OBSTACLE_TIE);
    evt_u(&b, mm_u(d_l), 9u);
    evt_u(&b, mm_u(d_r), 9u);
    evt_u(&b, (side == 'L') ? 1u : 2u, 2u);
    (void)evt_post(&b);
    return side;
}

/* STAGE1..STAGE5. Returns when the avoidance is over (either way).
 *   scan -> side with room and the smaller shift -> reverse only if the 45 deg
 *   leg needs it (standoff, within the reverse cap) -> bypass, every forward leg watched.
 *   Blocked on a side -> back out along the legs to where it started, then
 *     1. the other side if it was not tried (OBST_LEGACY: and the scan shows room there)
 *        (reverse and bypass);
 *     2. else a side already tried, the one that got further, 100 mm wider
 *        (OBST_RETRY_WIDER_MM; each side at most twice);
 *     3. else reverse 100 mm and scan again (reverse and rescan, once);
 *     4. else AVOIDANCE_ABORT.
 *   No room on either side at the scan -> 3. */
LOCAL void avoid(void)
{
    float         coarse[COARSE_N];
    bypass_plan_t plan;
    uint32_t      rescans = 0u;
    uint32_t      tries[2] = { 0u, 0u };      /* [0] = left, [1] = right */
    uint32_t      progress[2] = { 0u, 0u };   /* legs done before the block */
    float         extra[2] = { 0.0f, 0.0f };  /* added side margin, mm */
    bool          b_scan = true;
    float         half = g_params.robot_half_width_mm;
    float         margin = g_params.obst_margin_mm;
    char          side = 0;
    float         theta = 45.0f;              /* OBST_DYNAMIC_TURN: first-turn angle of this attempt */

    /* a SET during an avoidance takes effect at the next obstacle */
    g_legacy       = prm_flag(PRM_OBST_LEGACY);
    g_dyn          = !g_legacy && (OBST_DYNAMIC_TURN != 0);   /* OBST_DYNAMIC_TURN */
    g_rev_cap      = g_params.obst_reverse_max_mm;
    g_backed_mm    = 0.0f;
    g_rev_total_mm = 0.0f;
    g_n_legs       = 0u;

    for (uint32_t round = 0u; round < 6u; round++)
    {
        leg_res_t res;
        uint32_t  si;

        if (b_scan)
        {
            b_scan = false;
            /* STAGE1: the obstacle is further away by whatever we reversed */
            coarse_sweep(coarse);
            if (aborted())
            {
                return;
            }
            if (coarse_all_clear(coarse, (float)OBSTACLE_CLEAR_CM + (g_backed_mm / 10.0f)))
            {
                (void)evt_post0(EVC_OBSTACLE_CLEARED);   /* "coarse sweep clear: resume line following" */
                flag_set(EVF_BYPASS_COMPLETE);
                return;
            }
            /* STAGE2 + STAGE3 */
            fine_scan(coarse);
            if (aborted())
            {
                return;
            }
            if (!obstacle_profile(&g_fine, coarse, half, margin, &g_prof))
            {
                abort_avoidance(ABR_PROFILE_FAILED);
                return;
            }
            fine_scan_events();
            /* 31 Oct: a flat face only shows the patch straight ahead: widen to
               OBST_MIN_WIDTH_MM on both sides (worst case), remember by how much */
            g_ext_mm[0] = 0.0f;
            g_ext_mm[1] = 0.0f;
            if (!g_legacy)
            {
                obstacle_apply_min_width(&g_prof, g_params.obst_min_width_mm, &g_ext_mm[0], &g_ext_mm[1]);
                if (g_params.obst_edge_pad_mm > 0.0f)      /* OBST_EDGE_PAD_MM: flat-face ends */
                {
                    g_prof.x_l_cm   -= g_params.obst_edge_pad_mm / 10.0f;
                    g_prof.x_r_cm   += g_params.obst_edge_pad_mm / 10.0f;
                    g_prof.width_cm += g_params.obst_edge_pad_mm / 5.0f;
                    g_ext_mm[0]     += g_params.obst_edge_pad_mm;
                    g_ext_mm[1]     += g_params.obst_edge_pad_mm;
                }
            }
            side_view(&g_view);
            /* sides blocked twice are out */
            side = obstacle_choose_side(&g_view, half, margin,
                                        ((tries[0] >= 2u) ? OBST_BLOCKED_L : 0u) | ((tries[1] >= 2u) ? OBST_BLOCKED_R : 0u) |
                                        (g_legacy ? OBST_CHOOSE_LEGACY : 0u));
            g_prof.side = side;
            st_set_scan_side(side);
            profile_event(&g_prof);
            (void)post_obstacle(&g_prof);
            if ((side != 0) && is_tie(half, margin, tries))
            {
                side = tie_look(side);
                if (aborted())
                {
                    return;
                }
                g_prof.side        = side;
                st_set_scan_side(side);
            }
            if (side == 0)
            {
                if (rescans >= 1u)
                {
                    abort_avoidance(ABR_NO_ROOM);
                    return;
                }
                rescans++;
                b_scan = true;
                if (!rescan_reverse(0u))
                {
                    return;
                }
                continue;
            }
        }

        si = (side == 'L') ? 0u : 1u;
        g_prof.side = side;
        if ((tries[si] >= 1u) && (extra[si] < 1.0f))
        {
            extra[si] = (float)OBST_RETRY_WIDER_MM;    /* after a rescan: a blocked side only goes wider */
        }
        theta = 45.0f;                             /* OBST_DYNAMIC_TURN */
        {
            standoff_res_t so;

            g_half_obs = (!g_legacy && (side == 'R')) ? left_half_mm() : half;   /* the half facing the obstacle */
            so = g_dyn ? standoff_dyn(side, margin + extra[si], margin, &theta)   /* OBST_DYNAMIC_TURN */
                       : standoff(side, margin + extra[si]);

            if (so == STANDOFF_CAPPED)
            {
                abort_avoidance(ABR_REVERSE_LIMIT);
                return;
            }
            if (so != STANDOFF_OK)
            {
                abort_avoidance(ABR_REVERSE_FAILED);
                return;
            }
        }
        if (!(g_dyn ? bypass_plan_dyn(&g_prof, g_half_obs, margin + extra[si], g_params.obst_depth_mm,   /* OBST_DYNAMIC_TURN */
                                      (float)REACQ_MAX_LATERAL_MM, theta, (float)OBST_SIDE_MAX_MM, &plan)
                    : bypass_plan(&g_prof, g_half_obs, margin + extra[si], g_params.obst_depth_mm, (float)REACQ_MAX_LATERAL_MM, &plan)))
        {
            abort_avoidance(ABR_NO_PLAN);
            return;
        }
        g_seen_extra_mm = g_ext_mm[si];            /* the real side may be this much further out */
        res = run_bypass(&plan, g_half_obs + margin + extra[si], margin);
        if ((res == LEG_OK) || (res == LEG_SEARCH))
        {
            st_obstacle_done();
            /* BYPASS_COMPLETE: U1 searching, U1 right ("passed on the left/right[; on the path, searching for the line]") */
            /* D10: right F1, search F1 */
            (void)evt_post1(EVC_BYPASS_COMPLETE, ((plan.side > 0) ? 2u : 0u) | ((res == LEG_SEARCH) ? 1u : 0u), 2u);
            flag_set(EVF_BYPASS_COMPLETE);
            return;
        }
        if (res != LEG_BLOCKED)
        {
            abort_avoidance(g_why);
            return;
        }

        /* blocked: note how far it got, back out to where it started */
        tries[si]++;
        progress[si] = (g_n_legs > progress[si]) ? g_n_legs : progress[si];
        {
            /* BYPASS_BLOCKED: right U1, long U1 (side did not end in 1 m),
               distance CM9, angle S8 (deg): "right side: something at X cm,
               A deg; backing out" */
            evt_b_t b;
            bool    b_long = (g_block_deg > 500.0f);

            evt_begin(&b, EVC_BYPASS_BLOCKED);
            evt_u(&b, (side == 'R') ? 1u : 0u, 1u);
            evt_u(&b, b_long ? 1u : 0u, 1u);
            evt_cm9(&b, b_long ? 0.0f : g_block_cm);
            evt_s(&b, b_long ? 0 : round_cm(g_block_deg), 8u);
            (void)evt_post(&b);
        }
        if (!undo_legs(0u))
        {
            abort_avoidance(ABR_BACKOUT_FAILED);
            return;
        }

        /* 1. the other side, untried, with room in this scan */
        {
            char     other = (side == 'L') ? 'R' : 'L';
            uint32_t oi    = 1u - si;

            /* 31 Oct: the block shows the obstacle reaches further to the side tried,
               so the other side is tried even if the scan saw no room there (the
               leg watch still guards it); OBST_LEGACY: only with room */
            if ((tries[oi] == 0u) && (!g_legacy || obstacle_side_room(&g_prof, other, half, margin)))
            {
                side               = other;
                st_set_scan_side(other);
                (void)evt_post1(EVC_BYPASS_RETRY, (other == 'R') ? 2u : 0u, 2u);   /* D10 right F1, wider F1 = 0: "trying the left/right side (same scan)" */
                continue;
            }
            /* 2. a side already tried, the one that got further, wider */
            {
                int32_t best = -1;

                for (uint32_t i = 0u; i < 2u; i++)
                {
                    if ((tries[i] == 1u) && obstacle_side_room(&g_prof, (i == 0u) ? 'L' : 'R', half, margin + extra[i] + (float)OBST_RETRY_WIDER_MM) &&
                        ((best < 0) || (progress[i] > progress[(uint32_t)best])))
                    {
                        best = (int32_t)i;
                    }
                }
                if (best >= 0)
                {
                    side = (best == 0) ? 'L' : 'R';
                    extra[best] += (float)OBST_RETRY_WIDER_MM;
                    st_set_scan_side(side);
                    (void)evt_post1(EVC_BYPASS_RETRY, (side == 'R') ? 3u : 1u, 2u);   /* wider = 1: "left/right side again, 100 mm wider" */
                    continue;
                }
            }
            /* 3. reverse and rescan, once */
            if ((rescans < 1u) && ((tries[0] < 2u) || (tries[1] < 2u)))
            {
                rescans++;
                b_scan = true;
                if (!rescan_reverse(1u))
                {
                    return;
                }
                continue;
            }
        }
        abort_avoidance(ABR_BLOCKED_BOTH);
        return;
    }
    abort_avoidance(ABR_TOO_MANY);
}

void tsk_obstacle(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    usf_reset(&g_filt);
    servo_to(0.0f);

    for (;;)
    {
        sys_state_t st = sys_state();

        if (st == SYS_LINE_FOLLOWING)
        {
            /* STAGE0: idle monitoring */
            float      cm = ping();
            float      f;
            odometry_t o;
            float      rate;

            /* In a tight curve the sensor points off the track (cones, walls,
               people at the edge of the board): those echoes must not stop
               the car. ping() still checks the 8 cm proximity E-stop. */
            odometry_get(&o);
            rate = (o.turn_dps < 0.0f) ? -o.turn_dps : o.turn_dps;
            if ((g_params.obst_max_turn_dps > 0.0f) && (rate > g_params.obst_max_turn_dps))
            {
                usf_reset(&g_filt);
                (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
                continue;
            }
            f = usf_add(&g_filt, cm);
            if (usf_trigger(&g_filt, f, g_params.obstacle_stop_cm))
            {
                uint64_t t0 = hal_time_us();
                evt_b_t  b;

                motion_stop(SRC_OBSTACLE);        /* NFR1: first avoidance command */
                flag_set(EVF_OBSTACLE_TRIGGER);
                usf_reset(&g_filt);
                /* OBSTACLE: distance CM13, STOP latency U16 (us) */
                evt_begin(&b, EVC_OBSTACLE);
                evt_cm13(&b, f);
                evt_u(&b, (uint32_t)(hal_time_us() - t0), 16u);
                (void)evt_post(&b);
            }
            (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
        }
        else if (st == SYS_OBSTACLE_AVOIDANCE)
        {
            avoid();
            servo_move(0.0f, 0.0f);
            /* wait for the planner to leave OBSTACLE AVOIDANCE */
            while (sys_state() == SYS_OBSTACLE_AVOIDANCE)
            {
                (void)tk_dly_tsk(20);
            }
            usf_reset(&g_filt);
        }
        else
        {
            usf_reset(&g_filt);
            g_prox_n = 0u;
            (void)tk_dly_tsk(50);
        }
    }
}
