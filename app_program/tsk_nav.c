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
#include "motion_api.h"
#include "app_params.h"
#include "app_check.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

LOCAL char     g_pending;              /* BARCODE_AT_JUNCTION mode */
LOCAL uint32_t g_t_start;

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
        (void)post_event("START_REFUSED", "calibration missing (see console)");
        buzzer_request(BUZZ_ERROR);
        return;
    }
    g_pending             = 0;
    g_t_start             = hal_time_ms();
    g_status.hump_max_mm  = 0.0f;
    g_status.humps        = 0u;
    g_status.obstacles    = 0u;
    g_status.barcodes_read = 0u;
    g_status.barcode      = 0;
    flag_clear(EVF_OBSTACLE_TRIGGER | EVF_BYPASS_COMPLETE | EVF_BYPASS_ABORT | EVF_LINE_LOST |
               EVF_LINE_FOUND | EVF_HUMP_START | EVF_HUMP_END);
    motion_speed_limit(1.0f);
    buzzer_request(BUZZ_START);
    (void)sys_set_state(SYS_LINE_FOLLOWING, false);
}

LOCAL void complete_run(void)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    motion_stop(SRC_NAV);
    (void)sys_set_state(SYS_MISSION_COMPLETE, false);
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "t ");
    jb_uint(&jb, (hal_time_ms() - g_t_start) / 1000u);
    jb_raw(&jb, "s dist ");
    jb_fixed(&jb, g_status.dist_mm / 1000.0f, 2u);
    jb_raw(&jb, "m bc ");
    jb_uint(&jb, g_status.barcodes_read);
    jb_raw(&jb, " humps ");
    jb_uint(&jb, g_status.humps);
    jb_raw(&jb, " max ");
    jb_fixed(&jb, g_status.hump_max_mm, 1u);
    jb_raw(&jb, "mm obst ");
    jb_uint(&jb, g_status.obstacles);
    (void)post_event("MISSION_COMPLETE", text);       /* final summary (FR22) */
    log_line(text);
    buzzer_request(BUZZ_COMPLETE);
}

/* FR6 / FR8: barcode -> navigation command. At a junction the car first
   drives the sensor-to-axle distance, so it pivots on the crossing point. */
LOCAL void execute(char code, bool b_at_junction)
{
    float           deg = 0.0f;
    motion_result_t res;

    switch (code)
    {
        case 'A': deg = -(float)BARCODE_TURN_DEG; break;   /* turn left  */
        case 'B': deg = (float)BARCODE_TURN_DEG;  break;   /* turn right */
        case 'D': deg = 180.0f;                   break;   /* U-turn     */
        default:  deg = 0.0f;                     break;   /* C: straight */
    }
    if (deg == 0.0f)
    {
        (void)post_event("NAV", "go straight (C)");
        return;
    }
    (void)sys_set_state(SYS_MANEUVER, false);
    motion_stop(SRC_NAV);
    res = MOTION_OK;
    if (b_at_junction && (g_params.ir_to_axle_mm > 0.0f))
    {
        res = motion_move(g_params.ir_to_axle_mm, g_params.bypass_mm_s);
    }
    if ((res == MOTION_OK) && (sys_state() == SYS_MANEUVER))
    {
        res = motion_turn(deg, g_params.turn_mm_s);
    }
    if (sys_state() != SYS_MANEUVER)
    {
        return;                                /* E-stop or error during the turn */
    }
    if (res == MOTION_OK)
    {
        (void)post_event("NAV", (code == 'A') ? "turned left" : ((code == 'B') ? "turned right" : "U-turn"));
        (void)sys_set_state(SYS_LINE_FOLLOWING, false);
    }
    else
    {
        (void)post_event("NAV", "turn did not complete");
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
                if ((g_params.barcode_at_junction > 0.5f) && (p_ev->type == NAV_BARCODE))
                {
                    g_pending = p_ev->code;
                    (void)post_event("NAV", "command stored for the next junction");
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
            (void)post_event("NAV", "barcode fault: going straight (C)");
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
            nav_event_t ev = p_msg->body.nav;

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
