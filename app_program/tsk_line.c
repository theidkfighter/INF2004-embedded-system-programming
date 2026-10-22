/**
 * @file    tsk_line.c
 * @brief   TSK_LINE_BARCODE (DR Section 3 / 6.1, 10 ms): line following,
 *          junction detection, line search, barcode decoding, and line
 *          detection for the obstacle bypass.
 *
 * LINE FOLLOWING: line_logic.c turns the two straddling sensors into a
 *   steering value, sent to TSK_MOTION_CTRL as LINE wheel speeds every
 *   10 ms. A lost line starts the search (alternating pivots of growing
 *   amplitude); found -> EVF_LINE_FOUND, not found -> EVF_LINE_LOST.
 *   The barcode sensor's edges (queued by the 1 kHz timer interrupt) are
 *   decoded by barcode_logic.c; a decoded letter goes to TSK_NAV_PLANNER on
 *   MBX_NAV_EVENT.
 * OBSTACLE AVOIDANCE: no motion output (the bypass owns the wheels); the
 *   task only reports EVF_LINE_FOUND when a line sensor sees black
 *   (the light-weight reacquisition path of DR 6.1).
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "line_logic.h"
#include "barcode_logic.h"
#include "app_params.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define DT_S            (0.01f)

LOCAL line_ctx_t   g_line;
LOCAL search_ctx_t g_search;
LOCAL bool         g_searching;
LOCAL bc_ctx_t     g_bc;
LOCAL bc_params_t  g_bc_par;

LOCAL bool is_black(bool b_level)
{
    return b_level == (g_params.line_do_black_high > 0.5f);
}

LOCAL void barcode_arm(void)
{
    float    w = g_params.ir_white_adc;
    float    b = g_params.ir_black_adc;
    float    mid = 0.5f * (w + b);
    float    hys = 0.1f * ((b > w) ? (b - w) : (w - b));
    bool     b_high = (b > w);
    bool     b_ok = (w >= 0.0f) && (b >= 0.0f) && (hys > 10.0f);
    uint32_t thr_black = (uint32_t)(b_high ? (mid + hys) : (mid - hys));
    uint32_t thr_white = (uint32_t)(b_high ? (mid - hys) : (mid + hys));

    g_bc_par.min_elements = BARCODE_MIN_ELEMENTS;
    g_bc_par.max_len_um   = (uint32_t)BARCODE_MAX_LEN_MM * 1000u;
    g_bc_par.stray_gap_um = 30000u;
    g_bc_par.max_gap_um   = 60000u;
    bc_reset(&g_bc);
    hal_barcode_config(thr_black, thr_white, b_high, b_ok);
}

LOCAL uint32_t odo_to_um(uint32_t odo_counts)
{
    /* hal_odometer() counts both wheels: mean distance = counts / 2 */
    return (uint32_t)(((float)odo_counts * params_mm_per_count() * 1000.0f) * 0.5f);
}

LOCAL void report_barcode(bc_result_t res, char code, bool b_rev)
{
    char    text[TLM_EVENT_DETAIL_LEN];
    jbuf_t  jb;

    jb_init(&jb, text, sizeof(text));
    if (res == BC_DECODED)
    {
        jb_char(&jb, code);
        jb_raw(&jb, b_rev ? " (read backwards)" : " (read forwards)");
        g_status.barcode = code;
        g_status.barcodes_read++;
        if ((code >= 'A') && (code <= 'D'))
        {
            (void)post_nav(NAV_BARCODE, code);
            buzzer_request(BUZZ_BARCODE);
            (void)post_event("BARCODE", text);
        }
        else
        {
            (void)post_event("BARCODE_UNKNOWN", text);   /* decoded, but not a command */
        }
        log_line(text);
    }
    else if (res == BC_FAULT)
    {
        fault_set(FAULT_BARCODE);
        (void)post_nav(NAV_BARCODE_FAULT, 'C');
        (void)post_event("BARCODE_FAULT", "3 ambiguous barcodes: going straight (C)");
    }
    else if (res == BC_FAILED)
    {
        (void)post_event("BARCODE_FAILED", "window did not decode");
    }
    else
    {
        /* nothing */
    }
}

LOCAL void barcode_service(bool b_active)
{
    hal_bc_edge_t e;
    char          code = 0;
    bool          b_rev = false;
    bc_result_t   res;

    while (hal_barcode_pop(&e))
    {
        if (b_active)
        {
            res = bc_edge(&g_bc, &g_bc_par, odo_to_um(e.odo), e.b_black, &code, &b_rev);
            report_barcode(res, code, b_rev);
        }
    }
    if (b_active)
    {
        res = bc_poll(&g_bc, &g_bc_par, odo_to_um(hal_odometer()), &code, &b_rev);
        report_barcode(res, code, b_rev);
    }
    else
    {
        bc_cancel(&g_bc);
    }
}

LOCAL void follow(bool b_l, bool b_r, float ds_mm, float dheading)
{
    line_params_t par;
    line_out_t    out;
    float         left;
    float         right;

    if (g_searching)
    {
        int8_t dir;

        if (b_l || b_r)
        {
            g_searching = false;
            line_reset(&g_line);
            flag_set(EVF_LINE_FOUND);
            (void)post_event("LINE_FOUND", "search");
        }
        else
        {
            dir = search_step(&g_search, dheading);
            if (dir == 0)
            {
                g_searching = false;
                motion_stop(SRC_LINE);
                fault_set(FAULT_LINE_LOST);
                flag_set(EVF_LINE_LOST);
                (void)post_event("LINE_LOST", "search found nothing");
            }
            else
            {
                (void)motion_line((float)dir * (float)LINE_SEARCH_MM_S, -(float)dir * (float)LINE_SEARCH_MM_S);
            }
            return;
        }
    }

    par.steer_base = g_params.line_steer;
    par.steer_ramp = g_params.line_steer_ramp;
    par.hold_steer = g_params.line_hold_steer;
    par.hold_mm    = g_params.line_hold_mm;
    par.curve_mm   = (float)LINE_CURVE_MM;
    par.lost_mm    = g_params.line_lost_mm;
    out = line_step(&g_line, &par, b_l, b_r, ds_mm, DT_S);

    if (out.b_junction)
    {
        (void)post_nav(NAV_JUNCTION, 0);
    }
    if (out.b_lost)
    {
        g_searching = true;
        search_start(&g_search, (g_line.side < 0) ? -1 : 1, (float)LINE_SEARCH_DEG, (float)LINE_SEARCH_MAX_DEG);
        (void)post_event("LINE_SEARCH", (g_line.side < 0) ? "left first" : "right first");
        return;
    }
    line_wheels(g_params.line_base_mm_s, out.steer, &left, &right);
    (void)motion_line(left, right);
}

void tsk_line(INT stacd, void *exinf)
{
    odometry_t  odo;
    uint32_t    odo_prev = 0u;
    float       heading_prev = 0.0f;
    sys_state_t prev_state = SYS_INIT;
    uint32_t    found_n = 0u;

    (void)stacd;
    (void)exinf;
    line_reset(&g_line);
    barcode_arm();

    for (;;)
    {
        sys_state_t st;
        bool        b_l;
        bool        b_r;
        float       ds_mm;
        float       dheading;

        (void)tk_slp_tsk(TMO_FEVR);          /* released every 10 ms by the cyclic tick */
        g_hb[HB_LINE]++;

        st  = sys_state();
        b_l = is_black(hal_ir_left_level());
        b_r = is_black(hal_ir_right_level());
        odometry_get(&odo);
        ds_mm        = (float)(odo.odo_um - odo_prev) / 1000.0f;
        dheading     = odo.heading_deg - heading_prev;
        odo_prev     = odo.odo_um;
        heading_prev = odo.heading_deg;

        g_status.line_l     = b_l ? 1 : 0;
        g_status.line_r     = b_r ? 1 : 0;
        g_status.ir_adc     = hal_ir_adc();
        g_status.line_state = (uint32_t)g_line.state;

        if (st != prev_state)
        {
            if (st == SYS_LINE_FOLLOWING)
            {
                line_reset(&g_line);
                g_searching = false;
                barcode_arm();
                /* coming back from a turn or a bypass: the line should be
                   close, so treat a long white stretch as "lost" */
                g_line.b_last_curve = (prev_state == SYS_MANEUVER) || (prev_state == SYS_OBSTACLE_AVOIDANCE);
            }
            if (st == SYS_OBSTACLE_AVOIDANCE)
            {
                found_n = 0u;
            }
            prev_state = st;
        }

        barcode_service(st == SYS_LINE_FOLLOWING);

        if (st == SYS_LINE_FOLLOWING)
        {
            if ((flag_peek() & EVF_OBSTACLE_TRIGGER) == 0u)
            {
                follow(b_l, b_r, ds_mm, dheading);
            }
            /* else: the obstacle task has stopped the car; wait for the planner */
        }
        else if (st == SYS_OBSTACLE_AVOIDANCE)
        {
            /* reacquisition watch: 2 samples in a row, so one speck does not count */
            found_n = (b_l || b_r) ? (found_n + 1u) : 0u;
            if (found_n == 2u)
            {
                flag_set(EVF_LINE_FOUND);
            }
        }
        else
        {
            g_searching = false;
        }
    }
}
