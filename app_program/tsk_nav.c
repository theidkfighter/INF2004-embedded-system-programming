/**
 * @file    tsk_nav.c
 * @brief   TSK_NAV_PLANNER (DR Section 3 / 5, event-driven): mission
 *          sequencer and system-level state transitions.
 *
 *   IDLE            --BTN1 / START, calibration complete--> LINE FOLLOWING
 *   LINE FOLLOWING  --barcode A/B/D--> MANEUVER (encoder turn) --> LINE FOLLOWING
 *                   --barcode C------> keep going (FR8)
 *                   --EVF_OBSTACLE_TRIGGER--> OBSTACLE AVOIDANCE
 *                   --EVF_LINE_LOST--> IDLE (stopped, operator re-places the car)
 *                   --BTN1 / STOP----> MISSION COMPLETE
 *   OBSTACLE AVOIDANCE --EVF_BYPASS_COMPLETE--> LINE FOLLOWING
 *                      --EVF_BYPASS_ABORT----> ERROR (via the safety monitor)
 *   MISSION COMPLETE --BTN1--> IDLE
 *   any driving state: EVF_HUMP_START -> SPEED_LIMIT 60 %, EVF_HUMP_END -> 100 %
 * EMERGENCY STOP and ERROR belong to TSK_SAFETY_MON.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "status.h"
#include "msg_codec.h"
#include "motion_api.h"
#include "app_params.h"
#include "app_check.h"
#include "hal.h"
#include "perf.h"
#include "jfmt.h"
#include "robot_config.h"
#include "nav_map.h"

LOCAL char     g_pending;              /* BARCODE_AT_JUNCTION mode */
LOCAL uint32_t g_t_start;

/* NAV: kind E2, letter U5 (0 = 'A'), action E3 */
LOCAL void nav_event(evt_nav_kind_t kind, char letter, nav_action_t act)
{
    evt_b_t b;

    evt_begin(&b, EVC_NAV);
    evt_u(&b, (uint32_t)kind, 2u);
    evt_u(&b, ((letter >= 'A') && (letter <= 'Z')) ? (uint32_t)(letter - 'A') : 0u, 5u);
    evt_u(&b, (uint32_t)act, 3u);
    (void)evt_post(&b);
}

LOCAL bool driving(sys_state_t st)
{
    return (st == SYS_LINE_FOLLOWING) || (st == SYS_MANEUVER) || (st == SYS_OBSTACLE_AVOIDANCE);
}

LOCAL void start_run(void)
{
    char missing[160];

    if (!config_check(missing, sizeof(missing)))
    {
        log_line("[nav] cannot start, calibration missing:");
        log_line(missing);
        (void)evt_post0(EVC_START_REFUSED);   /* "calibration missing (see console)" */
        buzzer_request(BUZZ_ERROR);
        return;
    }
    g_pending             = 0;
    g_t_start             = hal_time_ms();
    {
        uint8_t act[26];

        if (nav_map_actions(act) < 4u)
        {
            /* not a refusal: the map may legitimately arrive after START */
            evt_b_t b;

            evt_begin(&b, EVC_MAP_WARNING);
            evt_map26(&b, act);
            (void)evt_post(&b);
        }
    }
    st_run_reset();                           /* run counters, hump max, last barcode */
    flag_clear(EVF_OBSTACLE_TRIGGER | EVF_BYPASS_COMPLETE | EVF_BYPASS_ABORT | EVF_LINE_LOST |
               EVF_LINE_FOUND | EVF_HUMP_START | EVF_HUMP_END);
    motion_speed_limit(1.0f);
    buzzer_request(BUZZ_START);
    perf_reset();                             /* APP_PERF: the [perf] counts cover this run */
    (void)sys_set_state(SYS_LINE_FOLLOWING, false);
}

LOCAL void complete_run(void)
{
    char    text[96];
    jbuf_t  jb;
    evt_b_t b;
    float   dist_cm = st_dist_mm() / 10.0f;      /* 0.01 m */
    float   hmax    = (st_hump_max_mm() * 10.0f) + 0.5f;

    motion_stop(SRC_NAV);
    (void)sys_set_state(SYS_MISSION_COMPLETE, false);
    /* MISSION_COMPLETE (FR22, design D10): time U17 s, dist U14 0.01 m,
       barcodes U6, humps U4, hump max U14 0.1 mm, obstacles U4 (each saturates) */
    evt_begin(&b, EVC_MISSION_COMPLETE);
    evt_u(&b, (hal_time_ms() - g_t_start) / 1000u, 17u);
    evt_u(&b, (dist_cm > 0.0f) ? (uint32_t)(dist_cm + 0.5f) : 0u, 14u);
    evt_u(&b, st_barcodes_read(), 6u);
    evt_u(&b, st_humps(), 4u);
    evt_u(&b, (hmax > 15000.0f) ? 15000u : ((hmax > 0.0f) ? (uint32_t)hmax : 0u), 14u);
    evt_u(&b, st_obstacles(), 4u);
    (void)evt_post(&b);
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "t ");
    jb_uint(&jb, (hal_time_ms() - g_t_start) / 1000u);
    jb_raw(&jb, "s dist ");
    jb_fixed(&jb, st_dist_mm() / 1000.0f, 2u);
    jb_raw(&jb, "m bc ");
    jb_uint(&jb, st_barcodes_read());
    jb_raw(&jb, " humps ");
    jb_uint(&jb, st_humps());
    jb_raw(&jb, " max ");
    jb_fixed(&jb, st_hump_max_mm(), 1u);
    jb_raw(&jb, "mm obst ");
    jb_uint(&jb, st_obstacles());
    log_line(text);
    buzzer_request(BUZZ_COMPLETE);
}

LOCAL bool ir_black(bool b_level)
{
    return b_level == (prm_tri(PRM_LINE_DO_BLACK_HIGH) > 0);
}

/* TURN_SNAP: the encoder turn stops SNAP_EARLY_DEG short, then the car keeps
   pivoting slowly until the new line is between the two line sensors: the
   leading sensor (right one for a right turn) goes black and then white
   again. Encoder/track-width errors and wheel slip on the foam no longer
   decide the final heading; the line does.
   patch/junction_check: if that sweep (SNAP_MAX_DEG) misses the line (the
   car pivoted early or late, or the encoder part already overshot), it
   sweeps back the other way up to SNAP_BACK_DEG past the start of the slow
   sweep, like the obstacle bypass's reacquire sweep (OBST_REACQ_SWEEP_DEG).
   Still nothing: back to the encoder angle, and line following searches. */
#define SNAP_EARLY_DEG   (20.0f)
#define SNAP_MAX_DEG     (45.0f)
#define SNAP_BACK_DEG    (25.0f)
#define SNAP_MM_S        (50.0f)

typedef enum
{
    SWEEP_FOUND = 0,
    SWEEP_MISSED,
    SWEEP_STOPPED
} sweep_t;

/* Pivots dir (+1 right, -1 left) up to max_deg at SNAP_MM_S and stops once
   the line is between the sensors. */
LOCAL sweep_t snap_sweep(float dir, float max_deg)
{
    uint16_t id     = motion_start_turn(dir * max_deg, SNAP_MM_S, SRC_NAV);
    bool     b_seen = false;
    uint32_t t0     = hal_time_ms();

    for (;;)
    {
        motion_result_t r2;
        uint32_t        other;
        bool            b_lead  = ir_black((dir > 0.0f) ? hal_ir_right_level() : hal_ir_left_level());
        bool            b_trail = ir_black((dir > 0.0f) ? hal_ir_left_level() : hal_ir_right_level());

        if (b_lead)
        {
            b_seen = true;
        }
        if ((b_seen && !b_lead) || (b_seen && b_trail))
        {
            motion_stop(SRC_NAV);              /* line is between the sensors */
            return SWEEP_FOUND;
        }
        if (motion_wait(id, 0u, 10u, &r2, &other))
        {
            return (r2 == MOTION_STOPPED) ? SWEEP_STOPPED : SWEEP_MISSED;
        }
        if ((sys_state() != SYS_MANEUVER) || ((hal_time_ms() - t0) > 8000u))
        {
            motion_stop(SRC_NAV);
            return SWEEP_STOPPED;
        }
    }
}

LOCAL motion_result_t turn_on_line(float deg)
{
    float           dir = (deg < 0.0f) ? -1.0f : 1.0f;
    float           mag = (deg < 0.0f) ? -deg : deg;
    motion_result_t res = motion_turn(dir * (mag - SNAP_EARLY_DEG), g_params.turn_mm_s);
    sweep_t         sw;

    if ((res != MOTION_OK) || (sys_state() != SYS_MANEUVER))
    {
        return res;
    }
    sw = snap_sweep(dir, SNAP_MAX_DEG);
    if (sw == SWEEP_MISSED)
    {
        /* missed going forward: sweep back past where the slow sweep began */
        sw = snap_sweep(-dir, SNAP_MAX_DEG + SNAP_BACK_DEG);
        if (sw == SWEEP_MISSED)
        {
            /* nothing either way: back to the encoder angle (now
               SNAP_EARLY_DEG + SNAP_BACK_DEG short of it) */
            (void)evt_post1(EVC_TURN_SNAP, 1u, 1u);   /* not_seen 1: "line not seen, encoder angle kept" */
            return motion_turn(dir * (SNAP_EARLY_DEG + SNAP_BACK_DEG), g_params.turn_mm_s);
        }
    }
    if (sw == SWEEP_STOPPED)
    {
        return MOTION_STOPPED;
    }
    (void)evt_post1(EVC_TURN_SNAP, 0u, 1u);   /* not_seen 0: "stopped on the line" */
    return MOTION_OK;
}

/* TURN_PAUSE_MS: stop and let the car settle (no roll from the move or the
   pivot) so the next motion starts from where the last one ended. */
LOCAL void settle(void)
{
    if (g_params.turn_pause_ms >= 1.0f)
    {
        motion_stop(SRC_NAV);
        (void)tk_dly_tsk((TMO)g_params.turn_pause_ms);
    }
}

/* FR6 / FR8: barcode -> navigation command. At a junction the car first
   drives the sensor-to-axle distance, so it pivots on the crossing point. */
LOCAL void execute(char code, bool b_at_junction)
{
    nav_action_t    act = nav_map_get(code);      /* set over MQTT, never hard-coded */
    float           deg = nav_action_deg(act);
    motion_result_t res;
    evt_b_t         b;

    evt_begin(&b, EVC_NAV_ACTION);              /* letter U5, action E3 */
    evt_u(&b, (uint32_t)(code - 'A'), 5u);
    evt_u(&b, (uint32_t)act, 3u);
    (void)evt_post(&b);
    if (deg == 0.0f)
    {
        return;                                /* FORWARD or unmapped: keep following */
    }
    (void)sys_set_state(SYS_MANEUVER, false);
    motion_stop(SRC_NAV);
    res = MOTION_OK;
    if (b_at_junction)
    {
        /* the junction is now reported JUNCTION_MIN_MM after the sensors
           first saw the cross, so drive that much less to pivot on the
           same point as before */
        float move_mm = g_params.ir_to_axle_mm - g_params.junction_min_mm;

        if (move_mm > 0.0f)
        {
            res = motion_move(move_mm, g_params.bypass_mm_s);
        }
    }
    if ((res == MOTION_OK) && (sys_state() == SYS_MANEUVER))
    {
        settle();                              /* stopped on the crossing point before the pivot */
        res = (prm_flag(PRM_TURN_SNAP) && b_at_junction) ? turn_on_line(deg)
                                                               : motion_turn(deg, g_params.turn_mm_s);
    }
    if ((res == MOTION_OK) && (sys_state() == SYS_MANEUVER))
    {
        settle();                              /* square on the new line before following it */
    }
    if (sys_state() != SYS_MANEUVER)
    {
        return;                                /* E-stop or error during the turn */
    }
    if (res == MOTION_OK)
    {
        nav_event(NAVK_DONE, code, act);
        (void)sys_set_state(SYS_LINE_FOLLOWING, false);
    }
    else
    {
        nav_event(NAVK_FAILED, code, act);
        flag_set(EVF_ERROR_REQ);
    }
}

LOCAL void on_nav_event(const nav_event_t *p_ev)
{
    sys_state_t st = sys_state();

    switch (p_ev->type)
    {
        case NAV_BTN1:
        case NAV_START:
        case NAV_STOP:
            if ((st == SYS_IDLE) && (p_ev->type != NAV_STOP))
            {
                start_run();
            }
            else if (driving(st))
            {
                complete_run();
            }
            else if ((st == SYS_MISSION_COMPLETE) && (p_ev->type != NAV_STOP))
            {
                (void)sys_set_state(SYS_IDLE, false);
            }
            else
            {
                /* nothing to do in this state */
            }
            break;

        case NAV_BARCODE:
        case NAV_REMOTE_BARCODE:
            if (st == SYS_LINE_FOLLOWING)
            {
                if (prm_flag(PRM_BARCODE_AT_JUNCTION) && (p_ev->type == NAV_BARCODE))
                {
                    g_pending = p_ev->code;
                    nav_event(NAVK_STORED, g_pending, nav_map_get(g_pending));
                }
                else
                {
                    execute(p_ev->code, false);
                }
            }
            break;

        case NAV_JUNCTION:
            if ((st == SYS_LINE_FOLLOWING) && (g_pending != 0))
            {
                char code = g_pending;

                g_pending = 0;
                execute(code, true);
            }
            break;

        case NAV_BARCODE_FAULT:
            nav_event(NAVK_BC_FAULT, 'A', ACT_NONE);
            break;

        default:
            break;
    }
}

void tsk_nav(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        app_msg_t  *p_msg = msg_recv(g_mbx_nav, 20);
        uint32_t    got = 0u;
        sys_state_t st;

        if (p_msg != NULL)
        {
            nav_event_t ev;

            msg_nav_unpack(&p_msg->body.nav, &ev);

            msg_free(p_msg);
            on_nav_event(&ev);
        }
        /* obstacle profiles: kept for the record (published by the scan task) */
        while ((p_msg = msg_recv(g_mbx_obstacle, TMO_POL)) != NULL)
        {
            msg_free(p_msg);
        }

        st = sys_state();
        if (!driving(st))
        {
            g_pending = 0;
        }
        (void)flag_take(EVF_HUMP_START | EVF_HUMP_END | EVF_BYPASS_COMPLETE |
                        EVF_BYPASS_ABORT | EVF_LINE_LOST, &got);
        got |= flag_peek() & EVF_OBSTACLE_TRIGGER;   /* cleared only after the state change */

        /* hump speed limit (DR A2) */
        if ((got & EVF_HUMP_START) != 0u)
        {
            motion_speed_limit((float)HUMP_SPEED_PCT / 100.0f);
        }
        if ((got & EVF_HUMP_END) != 0u)
        {
            motion_speed_limit(1.0f);
        }

        if (st == SYS_LINE_FOLLOWING)
        {
            if ((got & EVF_OBSTACLE_TRIGGER) != 0u)
            {
                /* state first: TSK_LINE_BARCODE stops steering while the flag is set */
                (void)sys_set_state(SYS_OBSTACLE_AVOIDANCE, false);
                flag_clear(EVF_OBSTACLE_TRIGGER);
            }
            else if ((got & EVF_LINE_LOST) != 0u)
            {
                motion_stop(SRC_NAV);
                (void)sys_set_state(SYS_IDLE, false);
                buzzer_request(BUZZ_WARN);
                log_line("[nav] line lost: put the car back on the line and press BTN1");
            }
            else
            {
                /* keep following */
            }
        }
        else if (st == SYS_OBSTACLE_AVOIDANCE)
        {
            if ((got & EVF_BYPASS_COMPLETE) != 0u)
            {
                (void)sys_set_state(SYS_LINE_FOLLOWING, false);
            }
            else if ((got & EVF_BYPASS_ABORT) != 0u)
            {
                flag_set(EVF_ERROR_REQ);
            }
            else
            {
                /* bypass in progress */
            }
        }
        else
        {
            flag_clear(EVF_OBSTACLE_TRIGGER);       /* other states: flags are dropped */
        }
    }
}
