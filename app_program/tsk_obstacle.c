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
 * STAGE3 profile (obstacle_logic.c) -> MBX_OBSTACLE, side with the smaller
 *   deviation that has room; no room -> reverse 100 mm and scan again (once).
 * STAGE4 bypass: pivot 45 away, leg d1, pivot back, leg d2 past it, each
 *   waiting for EVF_MOTION_DONE, pinging ahead between steps.
 * STAGE5 pivot 45 towards the line and drive diagonally until TSK_LINE_BARCODE
 *   reports EVF_LINE_FOUND (max 300 mm sideways), then forward to put the
 *   axle on the line and pivot back -> EVF_BYPASS_COMPLETE.
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
    float phys = (g_params.servo_low_angle_is_right > 0.5f) ? (90.0f - robot_deg) : (90.0f + robot_deg);

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

/* Runs one primitive; pings ahead while it moves (forward legs only).
   Returns false if it did not finish normally. */
LOCAL bool step(uint16_t id, bool b_watch_ahead, uint32_t stop_bits, uint32_t *p_stop_hit)
{
    uint32_t t0 = hal_time_ms();

    *p_stop_hit = 0u;
    for (;;)
    {
        motion_result_t res;
        uint32_t        other = 0u;

        if (motion_wait(id, stop_bits | EVF_ESTOP | EVF_PROXIMITY, PING_GAP_MS, &res, &other))
        {
            return res == MOTION_OK;
        }
        if ((other & (EVF_ESTOP | EVF_PROXIMITY)) != 0u)
        {
            return false;
        }
        if ((other & stop_bits) != 0u)
        {
            *p_stop_hit = other & stop_bits;
            motion_stop(SRC_OBSTACLE);
            return false;
        }
        if (sys_state() != SYS_OBSTACLE_AVOIDANCE)
        {
            motion_stop(SRC_OBSTACLE);
            return false;
        }
        if (b_watch_ahead)
        {
            (void)ping();
        }
        if ((hal_time_ms() - t0) > 30000u)
        {
            motion_stop(SRC_OBSTACLE);
            return false;
        }
    }
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

/* STAGE1..STAGE5. Returns when the avoidance is over (either way). */
LOCAL void avoid(void)
{
    float         coarse[COARSE_N];
    bypass_plan_t plan;
    uint32_t      hit = 0u;
    float         v = g_params.bypass_mm_s;
    float         vt = g_params.turn_mm_s;

    for (uint32_t attempt = 0u; attempt < 2u; attempt++)
    {
        /* STAGE1 */
        coarse_sweep(coarse);
        if (aborted())
        {
            return;
        }
        if (coarse_all_clear(coarse, (float)OBSTACLE_CLEAR_CM))
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
        if (!obstacle_profile(&g_fine, coarse, g_params.robot_half_width_mm, g_params.obst_margin_mm, &g_prof))
        {
            abort_avoidance("profile failed (no echo in the fine scan)");
            return;
        }
        g_status.scan_side = g_prof.side;
        profile_event(&g_prof);
        (void)post_obstacle(&g_prof);
        if (g_prof.side != 0)
        {
            break;
        }
        /* Stage 4 option "reverse and reattempt" */
        if ((attempt > 0u) || !step(motion_start_move(-100.0f, v, SRC_OBSTACLE), false, 0u, &hit))
        {
            abort_avoidance("no room on either side");
            return;
        }
    }

    if (!bypass_plan(&g_prof, g_params.robot_half_width_mm, g_params.obst_margin_mm,
                     g_params.obst_depth_mm, (float)REACQ_MAX_LATERAL_MM, &plan))
    {
        abort_avoidance("no bypass plan");
        return;
    }

    /* STAGE4 */
    if (!step(motion_start_turn(plan.turn1_deg, vt, SRC_OBSTACLE), false, 0u, &hit) ||
        !step(motion_start_move(plan.d1_mm, v, SRC_OBSTACLE), true, 0u, &hit) ||
        !step(motion_start_turn(plan.turn2_deg, vt, SRC_OBSTACLE), false, 0u, &hit) ||
        !step(motion_start_move(plan.d2_mm, v, SRC_OBSTACLE), true, 0u, &hit) ||
        !step(motion_start_turn(plan.turn3_deg, vt, SRC_OBSTACLE), false, 0u, &hit))
    {
        abort_avoidance("bypass leg failed (timeout, E-stop or proximity)");
        return;
    }

    /* STAGE5: diagonal return until the line is seen */
    flag_clear(EVF_LINE_FOUND);
    (void)step(motion_start_move(plan.return_max_mm, v, SRC_OBSTACLE), true, EVF_LINE_FOUND, &hit);
    if ((hit & EVF_LINE_FOUND) == 0u)
    {
        abort_avoidance("line not found within 300 mm sideways");
        return;
    }
    if (!step(motion_start_move(g_params.ir_to_axle_mm, v, SRC_OBSTACLE), false, 0u, &hit) ||
        !step(motion_start_turn(plan.realign_deg, vt, SRC_OBSTACLE), false, 0u, &hit))
    {
        abort_avoidance("realign failed");
        return;
    }
    g_status.obstacles++;
    (void)post_event("BYPASS_COMPLETE", (plan.side > 0) ? "passed on the right" : "passed on the left");
    flag_set(EVF_BYPASS_COMPLETE);
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
