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
 *   The barcode sensor is sampled at 1 kHz by the timer interrupt; its
 *   samples are turned into bar edges by bc_detect.c (BARCODE_DETECT = 1,
 *   default) or the interrupt's own two levels (= 0), then decoded by
 *   barcode_logic.c; a decoded letter goes to TSK_NAV_PLANNER on
 *   MBX_NAV_EVENT.
 * OBSTACLE AVOIDANCE: no motion output (the bypass owns the wheels); the
 *   task only reports EVF_LINE_FOUND when a line sensor sees black
 *   (the light-weight reacquisition path of DR 6.1); after the bypass
 *   re-arms it (g_reacq_seq) a sensor must read white first.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "line_logic.h"
#include "barcode_logic.h"
#include "bc_detect.h"
#include "bc_multi.h"
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
LOCAL bcm_t        g_bcm;          /* BARCODE_DETECT = 1: BARCODE_LEVELS detectors (bc_multi.c) */
LOCAL bool         g_det_on;
/* the barcode sensor's own travel (BARCODE_SENSOR_Y_MM), never decreasing */
LOCAL bool         g_path_init;
LOCAL int32_t      g_path_r;
LOCAL int32_t      g_path_l;
LOCAL uint32_t     g_path_um;
LOCAL float        g_path_frac;

LOCAL bool is_black(bool b_level)
{
    return b_level == (g_params.line_do_black_high > 0.5f);
}

LOCAL void barcode_arm(void)
{
    float    w = g_params.ir_white_adc;
    float    b = g_params.ir_black_adc;
    float    mid = 0.5f * (w + b);
    float    hys = g_params.barcode_hys * ((b > w) ? (b - w) : (w - b));
    bool     b_high = (b > w);
    bool     b_ok = (w >= 0.0f) && (b >= 0.0f) && (hys > 5.0f);
    uint32_t thr_black = (uint32_t)(b_high ? (mid + hys) : (mid - hys));
    uint32_t thr_white = (uint32_t)(b_high ? (mid - hys) : (mid + hys));

    g_bc_par.min_elements = BARCODE_MIN_ELEMENTS;
    g_bc_par.max_len_um   = (uint32_t)BARCODE_MAX_LEN_MM * 1000u;
    g_bc_par.stray_gap_um = 30000u;
    g_bc_par.max_gap_um   = 60000u;
    bc_reset(&g_bc);
    g_det_on = b_ok && (g_params.barcode_detect > 0.5f);
    if (g_det_on)
    {
        /* bc_detect.c on distance bins (BARCODE_DETECT = 1) */
        float dh = g_params.barcode_det_hys * ((b > w) ? (b - w) : (w - b));

        bcm_init(&g_bcm, (uint32_t)w, (uint32_t)b, g_params.barcode_det_hys,
                 (uint32_t)(g_params.barcode_levels + 0.5f));
        bcm_set_adapt_swing(&g_bcm, g_params.barcode_adapt_swing);
        thr_black = (uint32_t)(b_high ? (mid + dh) : (mid - dh));
        thr_white = (uint32_t)(b_high ? (mid - dh) : (mid + dh));
        hal_barcode_config(thr_black, thr_white, b_high, false);
        hal_barcode_raw(true);
    }
    else
    {
        hal_barcode_raw(false);
        hal_barcode_config(thr_black, thr_white, b_high, b_ok);
    }
    g_status.bc_thr_black = thr_black;
    g_status.bc_thr_white = thr_white;
    g_status.bc_enabled   = b_ok ? 1u : 0u;
    if (!b_ok)
    {
        (void)post_event("BC_DBG", "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close");
    }
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
        if ((code >= 'A') && (code <= 'Z'))
        {
            (void)post_nav(NAV_BARCODE, code);
            buzzer_request(BUZZ_BARCODE);
            (void)post_event("BARCODE", text);
        }
        else
        {
            (void)post_event("BARCODE_UNKNOWN", text);   /* digit or symbol: not a command */
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

/* BARCODE_DEBUG: after every closed window (3+ elements) publish what the
   decoder saw: a summary, then the element widths in 0.1 mm, bars first
   ("B" = bar, "s" = space are implied: the list alternates bar, space, ...
   starting with a bar). Narrow should be ~3x smaller than wide. */
LOCAL void debug_widths(const bc_ctx_t *p_bc, uint32_t n);

LOCAL void debug_window(bc_result_t res, char code, bool b_rev)
{
    char     text[TLM_EVENT_DETAIL_LEN];
    jbuf_t   jb;
    uint32_t n = g_bc.last_n;

    if ((g_params.barcode_debug < 0.5f) || (n < 3u))
    {
        return;
    }
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "n=");
    jb_uint(&jb, n);
    if (res == BC_DECODED)
    {
        jb_raw(&jb, " decoded ");
        jb_char(&jb, code);
        jb_raw(&jb, b_rev ? " (backwards)" : " (forwards)");
    }
    else if (res == BC_NONE)
    {
        jb_raw(&jb, (n < BARCODE_MIN_ELEMENTS) ? " stray mark (fewer than 9)" : " closed");
    }
    else
    {
        jb_raw(&jb, " NOT decoded");
    }
    jb_raw(&jb, ", thr ");
    jb_uint(&jb, g_status.bc_thr_black);
    jb_char(&jb, '/');
    jb_uint(&jb, g_status.bc_thr_white);
    jb_raw(&jb, ", ovr ");
    jb_uint(&jb, hal_barcode_overruns());
    (void)post_event("BC_DBG", text);
    debug_widths(&g_bc, n);
}

/* the element widths of one window, in 0.1 mm, bar first (at most 6 lines) */
LOCAL void debug_widths(const bc_ctx_t *p_bc, uint32_t n)
{
    char     text[TLM_EVENT_DETAIL_LEN];
    jbuf_t   jb;
    uint32_t i = 0u;
    uint32_t part = 1u;

    while (i < n)
    {
        jb_init(&jb, text, sizeof(text));
        jb_raw(&jb, "w");
        jb_uint(&jb, part);
        jb_raw(&jb, " (0.1mm, bar first):");
        while ((i < n) && (jb_len(&jb) < (TLM_EVENT_DETAIL_LEN - 8u)))
        {
            jb_char(&jb, ' ');
            jb_uint(&jb, (p_bc->w[i] + 50u) / 100u);
            i++;
        }
        (void)post_event("BC_DBG", text);
        part++;
        if (part > 6u)
        {
            break;                            /* at most 6 lines per window */
        }
    }
}

/* BARCODE_DEBUG with several levels: after every pass (3+ elements at some
   level) a summary, the result per read ("50%:Z29" = decoded Z from 29
   elements, "x27" = 27 elements, no letter, "-3" = stray mark of 3), then
   the widths of the read that gave the letter (else of the longest window).
   "adapt" is the local-contrast read (BARCODE_LEVELS = 6). */
LOCAL void debug_pass(bc_result_t res, char code, bool b_rev)
{
    char     text[TLM_EVENT_DETAIL_LEN];
    jbuf_t   jb;
    int32_t  k = g_bcm.used;
    uint32_t n;

    if (k < 0)
    {
        k = 0;
        for (uint32_t i = 1u; i < g_bcm.n_levels; i++)
        {
            if (g_bcm.last_n[i] > g_bcm.last_n[k]) { k = (int32_t)i; }
        }
    }
    n = g_bcm.last_n[k];
    if ((g_params.barcode_debug < 0.5f) || (n < 3u))
    {
        return;
    }
    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "n=");
    jb_uint(&jb, n);
    if (res == BC_DECODED)
    {
        jb_raw(&jb, " decoded ");
        jb_char(&jb, code);
        jb_raw(&jb, b_rev ? " (backwards) at " : " (forwards) at ");
        jb_raw(&jb, bcm_level_name((uint32_t)k));
    }
    else if (res == BC_NONE)
    {
        jb_raw(&jb, " stray mark (fewer than 9)");
    }
    else
    {
        jb_raw(&jb, g_bcm.b_conflict ? " NOT decoded: levels disagree" : " NOT decoded");
    }
    jb_raw(&jb, ", thr ");
    jb_uint(&jb, g_status.bc_thr_black);
    jb_char(&jb, '/');
    jb_uint(&jb, g_status.bc_thr_white);
    jb_raw(&jb, ", ovr ");
    jb_uint(&jb, hal_barcode_overruns());
    (void)post_event("BC_DBG", text);

    if (g_bcm.n_levels > 1u)
    {
        jb_init(&jb, text, sizeof(text));
        jb_raw(&jb, "levels");
        for (uint32_t i = 0u; i < g_bcm.n_levels; i++)
        {
            jb_char(&jb, ' ');
            jb_raw(&jb, bcm_level_name(i));
            jb_char(&jb, ':');
            if (g_bcm.last_res[i] == BC_DECODED)
            {
                jb_char(&jb, g_bcm.last_code[i]);
            }
            else
            {
                jb_char(&jb, (g_bcm.last_res[i] == BC_NONE) ? '-' : 'x');
            }
            jb_uint(&jb, g_bcm.last_n[i]);
        }
        (void)post_event("BC_DBG", text);
    }
    debug_widths(&g_bcm.bc[k], n);
}

LOCAL void feed_edge(uint32_t pos_um, bool b_black)
{
    char        code = 0;
    bool        b_rev = false;
    bool        b_was_open = bc_window_open(&g_bc);
    bc_result_t res = bc_edge(&g_bc, &g_bc_par, pos_um, b_black, &code, &b_rev);

    if (b_was_open && !bc_window_open(&g_bc))
    {
        debug_window(res, code, b_rev);
    }
    report_barcode(res, code, b_rev);
}

LOCAL uint32_t sensor_path_um(int32_t enc_r, int32_t enc_l)
{
    float ds;

    if (!g_path_init)
    {
        g_path_init = true;
        g_path_r    = enc_r;
        g_path_l    = enc_l;
        return g_path_um;
    }
    ds = det_sensor_ds(enc_r - g_path_r, enc_l - g_path_l, params_mm_per_count(),
                       g_params.track_width_mm, g_params.barcode_sensor_y_mm);
    g_path_r = enc_r;
    g_path_l = enc_l;
    if (ds > 0.0f)                            /* the detector needs a position that never goes back */
    {
        float    um = (ds * 1000.0f) + g_path_frac;
        uint32_t whole = (uint32_t)um;

        g_path_frac = um - (float)whole;
        g_path_um  += whole;
    }
    return g_path_um;
}

LOCAL void barcode_service(bool b_active)
{
    hal_bc_edge_t e;
    char          code = 0;
    bool          b_rev = false;
    bc_result_t   res;
    int32_t       enc_r;
    int32_t       enc_l;
    uint32_t      adc;

    /* BARCODE_DETECT = 1: samples -> distance bins (along the sensor's own
       path) -> edges (bc_detect.c) */
    while (hal_barcode_pop_sample(&enc_r, &enc_l, &adc))
    {
        uint32_t pos = sensor_path_um(enc_r, enc_l);

        if (b_active && g_det_on)
        {
            bool b_was = g_bcm.b_pass;

            res = bcm_sample(&g_bcm, &g_bc_par, pos, adc, &code, &b_rev);
            if (b_was && !g_bcm.b_pass)
            {
                debug_pass(res, code, b_rev);  /* a pass just ended */
            }
            report_barcode(res, code, b_rev);
        }
    }
    /* BARCODE_DETECT = 0: edges from the 1 kHz two-level interrupt */
    while (hal_barcode_pop(&e))
    {
        if (b_active)
        {
            feed_edge(odo_to_um(e.odo), e.b_black);
        }
    }
    if (!b_active)
    {
        bcm_cancel(&g_bcm);
    }
    if (b_active && !g_det_on)
    {
        bool b_was_open = bc_window_open(&g_bc);

        res = bc_poll(&g_bc, &g_bc_par, odo_to_um(hal_odometer()), &code, &b_rev);
        if (b_was_open && !bc_window_open(&g_bc))
        {
            debug_window(res, code, b_rev);
        }
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
    par.bias_ki       = g_params.line_bias_ki;
    par.bias_decay_mm = g_params.line_bias_decay_mm;
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
    {
        /* slow down in tight curves: at full steer the outer wheel would
           otherwise need 2 x base speed, which the PI loop cannot reach */
        float mag  = (out.steer < 0.0f) ? -out.steer : out.steer;
        float base = g_params.line_base_mm_s * (1.0f - (g_params.line_curve_slow * mag));

        line_wheels(base, out.steer, &left, &right);

        /* inner wheel in a curve: brake it rather than ask for a creep speed
           (the feed-forward start-up duty would make it run too fast) */
        if (g_params.line_min_wheel_mm_s > 0.0f)
        {
            float al = (left < 0.0f) ? -left : left;
            float ar = (right < 0.0f) ? -right : right;

            if ((al < g_params.line_min_wheel_mm_s) && (ar > al))
            {
                left = 0.0f;
            }
            else if ((ar < g_params.line_min_wheel_mm_s) && (al > ar))
            {
                right = 0.0f;
            }
            else
            {
                /* both wheels fast enough */
            }
        }
    }
    (void)motion_line(left, right);
}

void tsk_line(INT stacd, void *exinf)
{
    odometry_t  odo;
    uint32_t    odo_prev = 0u;
    float       heading_prev = 0.0f;
    sys_state_t prev_state = SYS_INIT;
    uint32_t    found_n = 0u;
    uint32_t    reacq_seen = 0u;
    uint32_t    white_l = 0u;
    uint32_t    white_r = 0u;

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
            /* reacquisition watch: 2 samples in a row, so one speck does not
               count. Re-armed by the bypass (g_reacq_seq) before the return:
               a sensor then counts only after it has read white, so one
               sitting on something dark (off the mat, a seam) cannot hide the
               line from the other one. */
            if (reacq_seen != g_reacq_seq)
            {
                reacq_seen = g_reacq_seq;
                found_n    = 0u;
                white_l    = 0u;
                white_r    = 0u;
            }
            white_l = b_l ? white_l : ((white_l < 1000u) ? (white_l + 1u) : white_l);
            white_r = b_r ? white_r : ((white_r < 1000u) ? (white_r + 1u) : white_r);
            found_n = ((b_l && (white_l >= 3u)) || (b_r && (white_r >= 3u))) ? (found_n + 1u) : 0u;
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
