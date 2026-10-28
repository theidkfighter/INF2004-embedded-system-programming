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
 *   room, the one with more clearance (28 Oct); equal -> a 90 deg look each
 *   way (pivot 30 + servo 60), then the smaller deviation;
 *   no room -> reverse 100 mm and scan again (once). Then reverse only if
 *   the first 45 deg leg would pass the obstacle's near corner closer than
 *   half the car + OBST_MARGIN_MM (obstacle_standoff_mm).
 * STAGE4 bypass: pivot 45 away, leg d1, pivot back, leg d2 past it, each
 *   waiting for EVF_MOTION_DONE. Leg d2 (28 Oct) follows the obstacle's
 *   side at 60 deg (the servo's limit) and keeps going until the side has
 *   ended, so long obstacles work; before turning back the car pivots 30
 *   deg towards it and looks square at its side (servo 60), driving on in
 *   100 mm steps while it is still there. Every forward leg (and the STAGE5 return)
 *   is watched: the sensor sweeps 0 / 30 / 0 / 60 deg towards the obstacle
 *   side; two pings at the same angle inside the car's path closer than
 *   OBST_LEG_STOP_CM = this side is blocked (an angled or soft obstacle the
 *   scan missed, a second obstacle): stop, back out along the legs to the scan pose, then
 *   go round the other side if the scan showed room there, else reverse
 *   100 mm and scan again without the blocked side (once). The 8 cm
 *   proximity E-stop stays as the last resort.
 * STAGE5 (28 Oct) the bypass keeps an encoder dead-reckoned pose relative to
 *   the original path; pivot back to 45 deg towards it and drive diagonally
 *   until TSK_LINE_BARCODE reports EVF_LINE_FOUND (re-armed watch), or until
 *   OBST_RETURN_PAST_MM past the path. Found: forward to put the axle on
 *   the line and turn parallel to the path. Not seen: reverse back onto the
 *   path, turn parallel, and line following's search finds the line
 *   (instead of the DR's 300 mm abort) -> EVF_BYPASS_COMPLETE.
 * Anything failing -> AVOIDANCE_ABORT: stop, EVF_BYPASS_ABORT (-> ERROR).
 */
#include <tk/tkernel.h>

#include "os_glue.h"
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
    g_status.obst_cm = cm;

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
        g_status.scan_cm[i] = p_cm[i];
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
    servo_move(0.0f, prev);
}

/* ---- bypass legs: a log for backing out, and STAGE0-style watching ---- */
#define LEG_LOG_MAX       (8u)

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
LOCAL float       g_backed_mm;          /* reversed so far in this avoidance */
LOCAL const char *g_why;                /* why run_bypass() failed */

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

LOCAL void event2(const char *p_name, const char *p_a, float x, const char *p_b);

LOCAL void pose_reset(void)
{
    odometry_t o;

    odometry_get(&o);
    g_pose.x_mm   = 0.0f;
    g_pose.y_mm   = 0.0f;
    g_pose.h_deg  = 0.0f;
    g_pose.h0_deg = o.heading_deg;
    g_pose.d_prev = 0.5f * (o.dist_r_mm + o.dist_l_mm);
}

LOCAL void pose_update(void)
{
    odometry_t o;
    float      d;
    float      h;

    odometry_get(&o);
    d = 0.5f * (o.dist_r_mm + o.dist_l_mm);
    h = o.heading_deg - g_pose.h0_deg;
    obstacle_pose_advance(&g_pose.x_mm, &g_pose.y_mm, g_pose.h_deg, h, d - g_pose.d_prev);
    g_pose.h_deg  = h;
    g_pose.d_prev = d;
}

LOCAL float odo_fwd_mm(void)
{
    odometry_t o;

    odometry_get(&o);
    return 0.5f * (o.dist_r_mm + o.dist_l_mm);
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
            if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
            {
                (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
                cm = ping();                  /* NFR8: one echo is not enough */
                if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
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

/* straight back (not logged: used before a scan, the scan pose moves) */
LOCAL bool reverse_mm(float mm)
{
    uint32_t hit;
    bool     b_ok = step(motion_start_move(-mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) == LEG_OK;

    if (b_ok)
    {
        g_backed_mm += mm;
    }
    return b_ok;
}

/* undo the logged legs in reverse order: back to the pose of the last scan,
   along the path just driven (the only ground known to be free: no rear sensor) */
LOCAL bool back_out(void)
{
    uint32_t hit;

    while (g_n_legs > 0u)
    {
        const leg_t *p_l = &g_legs[--g_n_legs];
        leg_res_t    r   = LEG_OK;

        if (p_l->b_turn)
        {
            r = step(motion_start_turn(-p_l->amount, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
        }
        else if ((p_l->amount > 2.0f) || (p_l->amount < -2.0f))
        {
            r = step(motion_start_move(-p_l->amount, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit);
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
   ping still hits the side (obstacle_side_seen), and trail_mm past where it
   was last seen (obstacle_side_trail_mm), at most OBST_SIDE_MAX_MM (then:
   LEG_BLOCKED, the side never ended). Pings alternate 0 / 60 deg and
   anything inside the car's path stops it, as on the other legs. */
LOCAL leg_res_t leg_side_follow(float min_mm, float gap_mm, float trail_mm, float ws)
{
    float     s0 = odo_fwd_mm();
    float     seen_at = -1.0f;              /* leg distance where the side was last seen */
    uint32_t  clear_n = 0u;
    uint32_t  k = 0u;
    uint32_t  t0 = hal_time_ms();
    uint16_t  id = motion_start_move((float)OBST_SIDE_MAX_MM, g_params.bypass_mm_s, SRC_OBSTACLE);
    leg_res_t r;

    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;
        float           a;
        float           cm;
        float           done;

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
            obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
        {
            (void)tk_dly_tsk(PING_GAP_MS - ECHO_WAIT_MS);
            cm = ping();
            if (obstacle_in_path(cm, a, g_params.robot_half_width_mm, (float)OBST_PATH_SIDE_MM, g_params.obst_leg_stop_cm))
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
            if (obstacle_side_seen(cm, gap_mm))
            {
                seen_at = done;
                clear_n = 0u;
            }
            else
            {
                clear_n++;
            }
        }
        if ((done >= min_mm) && ((seen_at < 0.0f) || ((clear_n >= 2u) && (done >= (seen_at + trail_mm)))))
        {
            motion_stop(SRC_OBSTACLE);
            (void)tk_dly_tsk(200);
            pose_update();
            r = LEG_OK;
            break;
        }
    }
    leg_log(false, odo_fwd_mm() - s0);
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
        if (!obstacle_side_beside(d90, gap_mm) &&
            !((d60 >= 0.0f) && ((d60 * 10.0f * 0.8660254f) <= ((gap_mm - 40.0f) + 100.0f))))
        {
            return leg_move(obstacle_side_trail90_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_params.robot_half_width_mm, margin_mm),
                            ws, 0u, &hit);
        }
        if (n >= (uint32_t)OBST_TURN_LOOK_STEPS)
        {
            g_block_cm  = (float)OBST_SIDE_MAX_MM / 10.0f;
            g_block_deg = 999.0f;                   /* the side did not end */
            return LEG_BLOCKED;
        }
        event2("BYPASS_EXTEND", "obstacle still beside the car (", (d90 >= 0.0f) ? d90 : d60, " cm): 100 mm further");
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

    g_n_legs = 0u;
    g_why    = "bypass leg failed (timeout, E-stop or proximity)";
    pose_reset();
    if (((r = leg_turn(p_plan->turn1_deg)) != LEG_OK) ||
        ((r = leg_move(p_plan->d1_mm, ws, 0u, &hit)) != LEG_OK) ||
        ((r = leg_turn(-g_pose.h_deg)) != LEG_OK) ||                  /* parallel to the path */
        ((r = leg_side_follow(p_plan->d2_mm, gap_mm,
                              obstacle_side_trail_mm(gap_mm, (float)OBST_US_AHEAD_MM, g_params.robot_half_width_mm, margin_mm),
                              ws)) != LEG_OK) ||
        ((r = look_before_turn(ws, gap_mm, margin_mm)) != LEG_OK))
    {
        return r;
    }

    /* STAGE5: 45 deg back towards the original path (x = 0) */
    toward = (g_pose.x_mm < 0.0f) ? 1.0f : -1.0f;      /* left of it: turn right */
    if ((r = leg_turn((toward * 45.0f) - g_pose.h_deg)) != LEG_OK)
    {
        return r;
    }
    s    = obstacle_sin_deg((g_pose.h_deg < 0.0f) ? -g_pose.h_deg : g_pose.h_deg);
    s    = (s < 0.3f) ? 0.3f : s;
    dist = (((g_pose.x_mm < 0.0f) ? -g_pose.x_mm : g_pose.x_mm) + (float)OBST_RETURN_PAST_MM) / s;
    g_reacq_seq++;                                      /* TSK_LINE: fresh line watch */
    (void)tk_dly_tsk(30);
    flag_clear(EVF_LINE_FOUND);
    r = leg_move(dist, ws, EVF_LINE_FOUND, &hit);
    if (r != LEG_OK)
    {
        return r;
    }
    if ((hit & EVF_LINE_FOUND) != 0u)
    {
        /* axle onto the line, then parallel to the original path */
        if ((step(motion_start_move(g_params.ir_to_axle_mm, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK) ||
            (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK))
        {
            g_why = "realign failed";
            return LEG_FAILED;
        }
        return LEG_OK;
    }

    /* past the original path and no line seen: back onto the path (watching
       for the line again), parallel to it, and let line following search */
    {
        float past = toward * g_pose.x_mm;              /* > 0: beyond the path */

        (void)post_event("BYPASS_RETURN", "line not seen crossing the original path: back to it, then search");
        flag_clear(EVF_LINE_FOUND);
        if ((past > 5.0f) &&
            (step(motion_start_move(-past / s, g_params.bypass_mm_s, SRC_OBSTACLE), false, 0.0f, EVF_LINE_FOUND, &hit) != LEG_OK))
        {
            g_why = "reverse onto the original path failed";
            return LEG_FAILED;
        }
        if (step(motion_start_turn(-g_pose.h_deg, g_params.turn_mm_s, SRC_OBSTACLE), false, 0.0f, 0u, &hit) != LEG_OK)
        {
            g_why = "realign failed";
            return LEG_FAILED;
        }
    }
    return LEG_SEARCH;
}

LOCAL void profile_event(const obstacle_profile_t *p)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "r ");
    jb_fixed(&jb, p->r_min_cm, 0u);
    jb_raw(&jb, "cm w ");
    jb_fixed(&jb, p->width_cm, 0u);
    jb_raw(&jb, "cm off ");
    jb_fixed(&jb, p->offset_cm, 0u);
    jb_raw(&jb, " clrL ");
    jb_fixed(&jb, p->clear_left_cm, 0u);
    jb_raw(&jb, " clrR ");
    jb_fixed(&jb, p->clear_right_cm, 0u);
    jb_raw(&jb, " side ");
    jb_char(&jb, (p->side != 0) ? p->side : '-');
    (void)post_event("OBSTACLE_PROFILE", text);
}

LOCAL void abort_avoidance(const char *p_why)
{
    motion_stop(SRC_OBSTACLE);
    servo_move(0.0f, 0.0f);
    fault_set(FAULT_BYPASS_ABORT);
    (void)post_event("AVOIDANCE_ABORT", p_why);
    flag_set(EVF_BYPASS_ABORT);
}

LOCAL void event2(const char *p_name, const char *p_a, float x, const char *p_b)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, p_a);
    jb_fixed(&jb, x, 0u);
    jb_raw(&jb, p_b);
    (void)post_event(p_name, text);
}

/* After the scan (and before each bypass attempt): reverse only if the
   first 45 deg leg would pass the near corner closer than the car's half
   width + OBST_MARGIN_MM (obstacle_standoff_mm). The profile is moved with
   the car, so the plan and a later back-out stay consistent. */
LOCAL bool standoff(char side, float margin_mm)
{
    float need = obstacle_standoff_mm(&g_prof, side, g_params.robot_half_width_mm, margin_mm) -
                 (g_prof.r_min_cm * 10.0f);

    if (need < 5.0f)
    {
        return true;
    }
    if (need > (float)OBST_BACKOFF_MAX_MM)
    {
        need = (float)OBST_BACKOFF_MAX_MM;
    }
    event2("OBSTACLE_BACKOFF", "reversing ", need,
           (side == 'L') ? " mm: a 45 deg turn would clip the left corner" : " mm: a 45 deg turn would clip the right corner");
    if (!reverse_mm(need))
    {
        return false;
    }
    g_prof.r_min_cm += need / 10.0f;
    return true;
}

/* both sides have room and about the same clearance (e.g. both "no echo") */
LOCAL bool is_tie(float half, float margin, const uint32_t *p_tries)
{
    float d = g_prof.clear_left_cm - g_prof.clear_right_cm;

    return (p_tries[0] == 0u) && (p_tries[1] == 0u) &&
           obstacle_side_room(&g_prof, 'L', half, margin) && obstacle_side_room(&g_prof, 'R', half, margin) &&
           (d <= OBST_SIDE_TIE_CM) && (d >= -OBST_SIDE_TIE_CM);
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
    char     text[TLM_EVENT_DETAIL_LEN];
    jbuf_t   jb;

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
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "tie: 90 deg look L ");
    jb_fixed(&jb, d_l, 0u);
    jb_raw(&jb, " cm, R ");
    jb_fixed(&jb, d_r, 0u);
    jb_raw(&jb, " cm -> side ");
    jb_char(&jb, side);
    (void)post_event("OBSTACLE_TIE", text);
    return side;
}

/* STAGE1..STAGE5. Returns when the avoidance is over (either way).
 *   scan -> side with room and more clearance -> reverse only if the 45 deg
 *   leg needs it (standoff) -> bypass, every forward leg watched.
 *   Blocked on a side -> back out along the legs to where it started, then
 *     1. the other side if it was not tried and the scan shows room there
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

    g_backed_mm = 0.0f;
    g_n_legs    = 0u;

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
                (void)post_event("OBSTACLE_CLEARED", "coarse sweep clear: resume line following");
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
                abort_avoidance("profile failed (no echo in the fine scan)");
                return;
            }
            /* sides blocked twice are out */
            side = obstacle_choose_side(&g_prof, half, margin,
                                        ((tries[0] >= 2u) ? OBST_BLOCKED_L : 0u) | ((tries[1] >= 2u) ? OBST_BLOCKED_R : 0u));
            g_prof.side = side;
            g_status.scan_side = side;
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
                g_status.scan_side = side;
            }
            if (side == 0)
            {
                if (rescans >= 1u)
                {
                    abort_avoidance("no room on either side");
                    return;
                }
                rescans++;
                b_scan = true;
                (void)post_event("OBSTACLE_RESCAN", "no room: reversing 100 mm and scanning again");
                if (!reverse_mm((float)OBST_RESCAN_REVERSE_MM))
                {
                    abort_avoidance("reverse failed (E-stop, proximity or timeout)");
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
        if (!standoff(side, margin + extra[si]))
        {
            abort_avoidance("reverse failed (E-stop, proximity or timeout)");
            return;
        }
        if (!bypass_plan(&g_prof, half, margin + extra[si], g_params.obst_depth_mm, (float)REACQ_MAX_LATERAL_MM, &plan))
        {
            abort_avoidance("no bypass plan");
            return;
        }
        res = run_bypass(&plan, half + margin + extra[si], margin);
        if ((res == LEG_OK) || (res == LEG_SEARCH))
        {
            g_status.obstacles++;
            (void)post_event("BYPASS_COMPLETE", (res == LEG_SEARCH) ? ((plan.side > 0) ? "passed on the right; on the path, searching for the line"
                                                                                       : "passed on the left; on the path, searching for the line")
                                                                    : ((plan.side > 0) ? "passed on the right" : "passed on the left"));
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
            char   text[TLM_EVENT_DETAIL_LEN];
            jbuf_t jb;

            jb_init(&jb, text, sizeof(text));
            if (g_block_deg > 500.0f)
            {
                jb_raw(&jb, (side == 'R') ? "right side: obstacle side did not end in 1 m; backing out"
                                          : "left side: obstacle side did not end in 1 m; backing out");
            }
            else
            {
                jb_raw(&jb, (side == 'R') ? "right side: something at " : "left side: something at ");
                jb_fixed(&jb, g_block_cm, 0u);
                jb_raw(&jb, " cm, ");
                jb_fixed(&jb, g_block_deg, 0u);
                jb_raw(&jb, " deg; backing out");
            }
            (void)post_event("BYPASS_BLOCKED", text);
        }
        if (!back_out())
        {
            abort_avoidance("backing out failed (E-stop, proximity or timeout)");
            return;
        }

        /* 1. the other side, untried, with room in this scan */
        {
            char     other = (side == 'L') ? 'R' : 'L';
            uint32_t oi    = 1u - si;

            if ((tries[oi] == 0u) && obstacle_side_room(&g_prof, other, half, margin))
            {
                side               = other;
                g_status.scan_side = other;
                (void)post_event("BYPASS_RETRY", (other == 'R') ? "trying the right side (same scan)" : "trying the left side (same scan)");
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
                    g_status.scan_side = side;
                    (void)post_event("BYPASS_RETRY", (side == 'R') ? "right side again, 100 mm wider" : "left side again, 100 mm wider");
                    continue;
                }
            }
            /* 3. reverse and rescan, once */
            if ((rescans < 1u) && ((tries[0] < 2u) || (tries[1] < 2u)))
            {
                rescans++;
                b_scan = true;
                (void)post_event("OBSTACLE_RESCAN", "blocked: reversing 100 mm and scanning again");
                if (!reverse_mm((float)OBST_RESCAN_REVERSE_MM))
                {
                    abort_avoidance("reverse failed (E-stop, proximity or timeout)");
                    return;
                }
                continue;
            }
        }
        abort_avoidance("blocked on both sides");
        return;
    }
    abort_avoidance("too many attempts");
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
                char     text[TLM_EVENT_DETAIL_LEN];
                jbuf_t   jb;

                motion_stop(SRC_OBSTACLE);        /* NFR1: first avoidance command */
                flag_set(EVF_OBSTACLE_TRIGGER);
                usf_reset(&g_filt);
                jb_init(&jb, text, sizeof(text));
                jb_raw(&jb, "at ");
                jb_fixed(&jb, f, 1u);
                jb_raw(&jb, " cm, STOP issued in ");
                jb_uint(&jb, (uint32_t)(hal_time_us() - t0));
                jb_raw(&jb, " us");
                (void)post_event("OBSTACLE", text);
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
