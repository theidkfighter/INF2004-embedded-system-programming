/**
 * @file    tsk_telemetry.c
 * @brief   TSK_TELEMETRY (DR Section 3 / 6.4, lowest priority, 500 ms):
 *          telemetry frames, heartbeat, events, command dispatch, status LEDs,
 *          battery monitoring, console status line.
 *
 * Frames keep the format the laptop monitor (laptop_tools/telemetry_monitor.py)
 * already checks, plus robot-only fields:
 *   {"seq","t","state","spd":[l,r],"duty":[l,r],"enc":[l,r],"dist",
 *    "line":{"l","r","ir","irmn","irmx","thr"},"bc","hump":{"pitch","h","peak","max"},
 *    "obst":{"cm","scan":[5],"side"},"imu":{"ev","tr"},"faults","vbat",
 *    "rssi","buf"}
 *
 * Commands on is02/cmd (CMD_DISPATCH): ESTOP goes straight to the safety
 * monitor from the network context (fast path); the rest are queued here:
 *   RESUME            refused: recovery needs BTN1 on the robot (DR 5)
 *   BARCODE:A..D      FR19 override -> MBX_NAV_EVENT
 *   START / STOP      same as BTN1 in IDLE / while driving
 *   PING:<n>          round-trip check
 *   SET:NAME=VALUE    run-time parameter (NFR14, NFR15)
 *   GET:PARAMS        all parameters (several acks, "part":1..n)
 * is02/config (retained, re-sent by the broker at every connect): the
 * calibration as "NAME=VALUE" lines, acked as cmd "CONFIG".
 * Each command is answered on is02/ack; unknown ones get a NACK.
 */
#include <tk/tkernel.h>

#include "code39.h"
#include "os_glue.h"
#include "status.h"
#include "app_net.h"
#include "app_params.h"
#include "hal.h"
#include "perf.h"
#include "jfmt.h"
#include "robot_config.h"
#include "nav_map.h"
#include "crit.h"
#include "wire.h"
#include "status.h"
#include "bitpack.h"


#define FRAME_MS        (500u)
#define HEARTBEAT_MS    (1000u)
#define CONSOLE_MS      (2000u)
#define CMD_SLOTS       (8u)

IMPORT volatile uint64_t g_estop_rx_us;      /* tsk_safety.c */
IMPORT volatile uint32_t g_estop_pwm_off_us;
IMPORT volatile bool     g_estop_reported;

/* Command queue: Command_Frames decoded in the radio task, handled here.
   Each slot is a wire_cmd_t (16 bytes) plus its decode status. */
typedef struct
{
    uint8_t    status;        /* wire_status_t of the decode */
    wire_cmd_t c;
} cmd_slot_t;

LOCAL cmd_slot_t        g_cmds[CMD_SLOTS];
LOCAL volatile uint32_t g_cmd_head;          /* radio task */
LOCAL volatile uint32_t g_cmd_tail;          /* this task */
LOCAL volatile bool     g_estop_ack_due;
LOCAL uint8_t           g_config[WIRE_CFG_LEN(WIRE_PARAMS_MAX)];
LOCAL volatile uint32_t g_config_len;
LOCAL volatile bool     g_config_due;
LOCAL uint8_t           g_out[NET_MSG_MAX];  /* one outgoing ack / event / heartbeat */
LOCAL uint16_t          g_seq;               /* Telemetry_Frame sequence, U15 (wraps) */
LOCAL uint16_t          g_hb_seq;            /* Heartbeat_Frame sequence, U14 (wraps) */

/* ------------------------------------------------------------------------ */
/* Network context: called by app_net.c in the radio-owner task              */
/* ------------------------------------------------------------------------ */

/* memset without <string.h> (micro T-Kernel defines size_t itself) */
LOCAL void zero(void *p, uint32_t n)
{
    uint8_t *p_b = (uint8_t *)p;

    for (uint32_t i = 0u; i < n; i++)
    {
        p_b[i] = 0u;
    }
}

LOCAL void queue_cmd(uint32_t status, const wire_cmd_t *p_c)
{
    if ((g_cmd_head - g_cmd_tail) < CMD_SLOTS)
    {
        cmd_slot_t *p_s = &g_cmds[g_cmd_head % CMD_SLOTS];

        p_s->status = (uint8_t)status;
        p_s->c      = *p_c;
        g_cmd_head++;
        (void)tk_wup_tsk(g_tsk_tlm);
    }
}

void app_net_command(const uint8_t *p, uint32_t len, uint64_t rx_us)
{
    wire_cmd_t c;
    int        st;

    if ((len == 1u) && (p[0] == (uint8_t)((WIRE_FORMAT_VERSION << 4) | (uint32_t)WCMD_ESTOP)))
    {
        /* NFR17 fast path: straight to the safety monitor, no queueing */
        g_estop_rx_us   = rx_us;
        g_estop_ack_due = true;
        flag_set(EVF_REMOTE_ESTOP);
        (void)tk_wup_tsk(g_tsk_safety);
        return;
    }
    st = wire_dec_cmd(p, len, &c);
    if ((st != WIRE_OK) && (st != WIRE_BAD_LETTER) && (st != WIRE_BAD_DUP_LETTER))
    {
        c.cmd = (uint8_t)WCMD_UNPARSED;
    }
    queue_cmd((uint32_t)st, &c);
}

void app_net_map(const uint8_t *p, uint32_t len)
{
    wire_cmd_t c;
    int        st;

    if (len == 0u)
    {
        return;                               /* retained map deleted */
    }
    st = wire_dec_map(p, len, &c);
    if ((st != WIRE_OK) && (st != WIRE_BAD_LETTER) && (st != WIRE_BAD_DUP_LETTER))
    {
        c.cmd = (uint8_t)WCMD_UNPARSED;
    }
    queue_cmd((uint32_t)st, &c);
}

void app_net_config(const uint8_t *p, uint32_t len)
{
    if (!g_config_due && (len <= sizeof(g_config)))
    {
        for (uint32_t i = 0u; i < len; i++)
        {
            g_config[i] = p[i];
        }
        g_config_len = len;
        g_config_due = true;
        (void)tk_wup_tsk(g_tsk_tlm);
    }
}

/* Pushes the parameters the drivers keep their own copy of. */
void params_apply(void)
{
    hal_enc_set_reversed(prm_flag(PRM_ENC_R_REVERSED), prm_flag(PRM_ENC_L_REVERSED));
    hal_servo_range((uint32_t)g_params.servo_min_us, (uint32_t)g_params.servo_max_us);
}

/* ------------------------------------------------------------------------ */
/* Acks (Ack_Frame)                                                          */
/* ------------------------------------------------------------------------ */

LOCAL void ack_init(wire_ack_t *p_a, wire_cmd_code_t cmd, wire_reason_t reason)
{
    zero(p_a, sizeof(*p_a));
    p_a->t_ms   = hal_time_ms();
    p_a->cmd    = (uint8_t)cmd;
    p_a->reason = (uint8_t)reason;
}

LOCAL void ack_send(const wire_ack_t *p_a)
{
    uint32_t n = wire_enc_ack(p_a, g_out, sizeof(g_out));

    if (n > 0u)
    {
        (void)net_post(NET_T_ACK, g_out, n);
    }
}

LOCAL void ack_simple(wire_cmd_code_t cmd, wire_reason_t reason)
{
    wire_ack_t a;

    ack_init(&a, cmd, reason);
    ack_send(&a);
}

/* MAP, MAP:CLEAR, GET:MAP acks carry the whole map (26 x E3 + count). */
LOCAL void map_ack(wire_cmd_code_t cmd, wire_reason_t reason)
{
    wire_ack_t a;

    ack_init(&a, cmd, reason);
    a.a.map.n = (uint8_t)nav_map_actions(a.a.map.act);
    ack_send(&a);
}

LOCAL void map_event(void)
{
    uint8_t act[26];
    evt_b_t b;

    (void)nav_map_actions(act);
    evt_begin(&b, EVC_MAP);
    evt_map26(&b, act);
    (void)evt_post(&b);
}

LOCAL wire_reason_t status_reason(uint32_t st, uint32_t cmd)
{
    switch (st)
    {
        case WIRE_BAD_VERSION:    return RSN_BAD_VERSION;
        case WIRE_BAD_LENGTH:     return RSN_BAD_LENGTH;
        case WIRE_BAD_LETTER:     return (cmd == (uint32_t)WCMD_BARCODE) ? RSN_BAD_LETTER : RSN_MAP_LETTERS;
        case WIRE_BAD_DUP_LETTER: return RSN_MAP_DUP_LETTER;
        case WIRE_BAD_COUNT:      return RSN_BAD_COUNT;
        default:                  return RSN_UNKNOWN_COMMAND;
    }
}

LOCAL void get_params(void)
{
    uint32_t total = params_count();

    /* parts of WIRE_GETP_PER_PART (12): part U3 1..5, each value in its
       Parameter_Store representation (D11/D15: F32, or the 1..3-bit code) */
    for (uint32_t first = 0u; first < total; first += WIRE_GETP_PER_PART)
    {
        wire_ack_t a;

        ack_init(&a, WCMD_GET_PARAMS, RSN_OK);
        a.a.getp.part = (uint8_t)((first / WIRE_GETP_PER_PART) + 1u);
        for (uint32_t i = first; (i < total) && (i < (first + WIRE_GETP_PER_PART)); i++)
        {
            wire_pval_t *p_v = &a.a.getp.v[a.a.getp.count];

            p_v->id   = (uint8_t)i;
            p_v->w    = (uint8_t)params_store_width(i);
            p_v->code = params_store_code(i);
            a.a.getp.count++;
        }
        ack_send(&a);
    }
}

LOCAL void handle_command(const cmd_slot_t *p_s)
{
    sys_state_t       st = sys_state();
    const wire_cmd_t *p_c = &p_s->c;
    wire_ack_t        a;

    if (p_s->status != (uint8_t)WIRE_OK)
    {
        if ((p_c->cmd == (uint8_t)WCMD_BARCODE) && (p_s->status == (uint8_t)WIRE_BAD_LETTER))
        {
            ack_init(&a, WCMD_BARCODE, RSN_BAD_LETTER);
            a.a.letter = p_c->letter;
            ack_send(&a);
        }
        else if (p_c->cmd == (uint8_t)WCMD_MAP)
        {
            map_ack(WCMD_MAP, status_reason(p_s->status, p_c->cmd));
        }
        else
        {
            ack_simple(WCMD_UNPARSED, status_reason(p_s->status, p_c->cmd));
        }
        return;
    }
    switch (p_c->cmd)
    {
        case WCMD_ESTOP:                      /* normally the fast path */
            g_estop_ack_due = true;
            flag_set(EVF_REMOTE_ESTOP);
            (void)tk_wup_tsk(g_tsk_safety);
            break;
        case WCMD_RESUME:
            ack_simple(WCMD_RESUME, RSN_RESUME_REFUSED);
            break;
        case WCMD_BARCODE:
            ack_init(&a, WCMD_BARCODE, (st == SYS_LINE_FOLLOWING) ? RSN_QUEUED : RSN_ONLY_LINE_FOLLOW);
            a.a.letter = p_c->letter;
            if (st == SYS_LINE_FOLLOWING)
            {
                (void)post_nav(NAV_REMOTE_BARCODE, (char)('A' + p_c->letter));
            }
            ack_send(&a);
            break;
        case WCMD_START:
            (void)post_nav(NAV_START, 0);
            ack_simple(WCMD_START, (st == SYS_IDLE) ? RSN_STARTING : RSN_ONLY_FROM_IDLE);
            break;
        case WCMD_STOP:
            (void)post_nav(NAV_STOP, 0);
            ack_simple(WCMD_STOP, RSN_ENDING_RUN);
            break;
        case WCMD_MAP:
        {
            uint32_t n = nav_map_set_slots(p_c->map_present, p_c->map_letter);

            map_ack(WCMD_MAP, (n == 4u) ? RSN_OK : ((n == 0u) ? RSN_CLEARED : RSN_OK_NOT_4));
            map_event();
            break;
        }
        case WCMD_MAP_CLEAR:
            nav_map_clear();
            map_ack(WCMD_MAP_CLEAR, RSN_CLEARED);
            map_event();
            break;
        case WCMD_GET_MAP:
            map_ack(WCMD_GET_MAP, RSN_OK);
            break;
        case WCMD_PING:
            ack_init(&a, WCMD_PING, RSN_PONG);
            a.a.ping = p_c->ping;
            ack_send(&a);
            break;
        case WCMD_SET:
        {
            union { uint32_t u; float f; } v;
            int32_t r;

            v.u = p_c->value_bits;
            r   = params_set_id(p_c->param_id, v.f);
            params_apply();
            ack_init(&a, WCMD_SET, (r == 0) ? RSN_OK : ((r == -1) ? RSN_UNKNOWN_PARAM
                                  : ((r == -2) ? RSN_BAD_NUMBER : RSN_OUT_OF_RANGE)));
            a.a.param_id = p_c->param_id;
            ack_send(&a);
            break;
        }
        case WCMD_GET_PARAMS:
            get_params();
            break;
        default:
            ack_simple(WCMD_UNPARSED, RSN_UNKNOWN_COMMAND);
            break;
    }
}

LOCAL void estop_ack(void)
{
    wire_ack_t a;

    if (!g_estop_ack_due || (g_estop_rx_us != 0u))
    {
        return;                               /* the safety monitor has not acted yet */
    }
    g_estop_ack_due = false;
    ack_init(&a, WCMD_ESTOP, RSN_MOTORS_OFF);
    a.a.pwm_off_us = (g_estop_pwm_off_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)g_estop_pwm_off_us;
    ack_send(&a);
}

/* Config_Frame: each (Parameter_ID, F32) pair in order. */
typedef struct
{
    uint8_t applied;
    uint8_t n_rej;
    uint8_t rej[WIRE_CFG_REJ_MAX];
} cfg_result_t;

LOCAL void cfg_apply_one(uint32_t id, uint32_t value_bits, void *p_ctx)
{
    cfg_result_t *p_r = (cfg_result_t *)p_ctx;
    union { uint32_t u; float f; } v;

    v.u = value_bits;
    if (params_set_id(id, v.f) == 0)
    {
        p_r->applied++;
    }
    else if (p_r->n_rej < WIRE_CFG_REJ_MAX)
    {
        p_r->rej[p_r->n_rej] = (uint8_t)id;
        p_r->n_rej++;
    }
    else
    {
        /* cannot happen: at most 55 pairs */
    }
}

LOCAL void handle_config(void)
{
    cfg_result_t r = { 0u, 0u, { 0u } };
    wire_ack_t   a;
    int          st = wire_dec_config(g_config, g_config_len, cfg_apply_one, &r);

    g_config_due = false;
    params_apply();
    if (st != WIRE_OK)
    {
        ack_init(&a, WCMD_CONFIG, status_reason((uint32_t)st, WCMD_CONFIG));
        log_line("[config] is02/config rejected (not a valid Config_Frame)");
    }
    else
    {
        ack_init(&a, WCMD_CONFIG, (r.n_rej == 0u) ? RSN_OK : RSN_CONFIG_REJECTED);
        log_line((r.n_rej == 0u) ? "[config] is02/config applied" : "[config] is02/config applied, some rejected");
    }
    a.a.cfg.applied = r.applied;
    a.a.cfg.n_rej   = r.n_rej;
    for (uint32_t i = 0u; i < r.n_rej; i++)
    {
        a.a.cfg.rej[i] = r.rej[i];
    }
    ack_send(&a);
}

/* ------------------------------------------------------------------------ */
/* Telemetry_Frame and Heartbeat_Frame                                       */
/* ------------------------------------------------------------------------ */

/* Values of a frame that do not come from g_status */
typedef struct
{
    uint16_t    seq;
    uint32_t    t_ms;
    sys_state_t state;
    uint32_t    ir_min;
    uint32_t    ir_max;
    int32_t     rssi;
    uint32_t    buf;
} frame_extra_t;

/* Fills the Telemetry_Frame from a Status_Snapshot (g_status, or a fixed
   copy for the APP_PERF benchmark) and p_x. The snapshot fields already use
   the frame's resolutions (status.h, wire.h), so codes are copied, not
   requantized. Each unit is read once. */
LOCAL void frame_fill(wire_tlm_t *p_t, const robot_status_t *p_s, const frame_extra_t *p_x)
{
    uint32_t sys = p_s->sys;
    uint32_t ma  = p_s->mot_a;
    uint32_t mb  = p_s->mot_b;
    uint32_t la  = p_s->line_a;
    uint32_t lb  = p_s->line_b;
    uint32_t ia  = p_s->imu_a;
    uint32_t ib  = p_s->imu_b;
    uint32_t ic  = p_s->imu_c;
    uint32_t oa  = p_s->obs_a;
    uint32_t ob  = p_s->obs_b;
    uint32_t oc  = p_s->obs_c;
    bool     b_thr = (BP_GET(la, 23u, 1u) != 0u);

    p_t->seq       = (uint16_t)(p_x->seq & 0x7FFFu);
    p_t->t_ms      = p_x->t_ms & 0xFFFFFFu;
    p_t->state     = (uint8_t)p_x->state;
    p_t->spd[0]    = (int16_t)bp_sext(BP_GET(ma, 0u, 15u), 15u);
    p_t->spd[1]    = (int16_t)bp_sext(BP_GET(ma, 15u, 15u), 15u);
    p_t->duty[0]   = (int16_t)bp_sext(BP_GET(mb, 0u, 11u), 11u);
    p_t->duty[1]   = (int16_t)bp_sext(BP_GET(mb, 11u, 11u), 11u);
    p_t->enc[0]    = bp_sext(BP_GET(p_s->mot_c, 0u, 24u), 24u);
    p_t->enc[1]    = bp_sext(BP_GET(p_s->mot_d, 0u, 24u), 24u);
    p_t->dist      = BP_GET(p_s->mot_e, 0u, 20u);
    p_t->line_l    = (int8_t)BP_GET(la, 0u, 1u);
    p_t->line_r    = (int8_t)BP_GET(la, 1u, 1u);
    p_t->ir        = (uint16_t)BP_GET(la, 2u, 12u);
    p_t->ir_min    = (uint16_t)((p_x->ir_min > 4095u) ? 4095u : p_x->ir_min);
    p_t->ir_max    = (uint16_t)((p_x->ir_max > 4095u) ? 4095u : p_x->ir_max);
    p_t->thr_valid = b_thr ? 1u : 0u;
    p_t->thr_b     = (uint16_t)BP_GET(lb, 0u, 12u);
    p_t->thr_w     = (uint16_t)BP_GET(lb, 12u, 12u);
    {
        uint32_t c = BP_GET(la, 17u, 6u);      /* Status_Snapshot: ASCII - 32, 0 = none */

        p_t->bc = (uint8_t)((c == 0u) ? C39_NONE : c39_to_index((char)(c + 32u)));   /* wire: D8 Code 39 index */
    }
    p_t->side      = (uint8_t)BP_GET(oa, 13u, 2u);
    p_t->pitch     = (int16_t)bp_sext(BP_GET(ia, 0u, 11u), 11u);
    p_t->hump_h    = (int16_t)bp_sext(BP_GET(ia, 11u, 15u), 15u);
    p_t->hump_peak = (uint16_t)BP_GET(ib, 0u, 14u);
    p_t->hump_max  = (uint16_t)BP_GET(ib, 14u, 14u);
    p_t->us        = (uint16_t)BP_GET(oa, 0u, 13u);
    p_t->scan[0]   = (uint16_t)BP_GET(oa, 15u, 13u);
    p_t->scan[1]   = (uint16_t)BP_GET(ob, 0u, 13u);
    p_t->scan[2]   = (uint16_t)BP_GET(ob, 13u, 13u);
    p_t->scan[3]   = (uint16_t)BP_GET(oc, 0u, 13u);
    p_t->scan[4]   = (uint16_t)BP_GET(oc, 13u, 13u);
    p_t->imu_ev    = (uint8_t)BP_GET(sys, ST_SH_MEV, ST_W_MEV);
    p_t->rssi      = (uint8_t)((p_x->rssi >= 0) ? 0 : ((p_x->rssi < -255) ? 255 : -p_x->rssi));
    p_t->turn      = (int16_t)bp_sext(BP_GET(ic, 0u, 14u), 14u);
    p_t->faults    = (uint16_t)BP_GET(sys, ST_SH_FAULTS, ST_W_FAULTS);
    p_t->vbat      = (uint16_t)BP_GET(p_s->misc, 0u, 13u);
    p_t->buf       = (uint8_t)((p_x->buf > 127u) ? 127u : p_x->buf);
}

LOCAL void frame(void)
{
    static uint8_t s_frame[WIRE_TLM_LEN];
    wire_tlm_t     t;
    frame_extra_t  x;
    uint32_t       n;
    PERF_BEGIN(t_perf);                       /* PERF_TLM_ENCODE: snapshot read to complete payload */

    x.seq   = g_seq;
    g_seq   = (uint16_t)((g_seq + 1u) & 0x7FFFu);
    x.t_ms  = hal_time_ms();
    x.state = sys_state();
    hal_ir_adc_range(&x.ir_min, &x.ir_max);   /* barcode sensor range over the last 0.5 s */
    x.rssi  = net_rssi();
    x.buf   = net_frames_waiting();
    frame_fill(&t, &g_status, &x);
    n = wire_enc_telemetry(&t, s_frame);
    PERF_END(PERF_TLM_ENCODE, t_perf, PERF_NO_LIMIT);
    if (n > 0u)
    {
        (void)net_post_frame(s_frame, n);
    }
}

#if APP_PERF
#define TLM_BENCH_AT_MS  (10000u)
#define TLM_BENCH_N      (1000u)

/* Requirement 20.4 benchmark: TLM_BENCH_N encodes of one fixed snapshot into
   a scratch buffer (nothing is published), each timed on its own. */
LOCAL void tlm_bench(void)
{
    /* Task 16.9: times the Telemetry_Frame build. "fill" = Status_Snapshot
       copy read through its packed fields (frame_fill) + binary encode;
       "enc" = wire_enc_telemetry() alone. */
    static robot_status_t s_snap;           /* a copy: the encoder never sees a live update */
    static uint8_t        s_scratch[WIRE_TLM_LEN];
    frame_extra_t         x;
    wire_tlm_t            t;
    jbuf_t                jb;
    uint32_t              max_us = 0u;
    uint32_t              sum_us = 0u;
    uint32_t              enc_max = 0u;
    uint32_t              enc_sum = 0u;
    char                  line[96];

    crit_dsp_enter();
    s_snap = g_status;                        /* one consistent copy of the 15 units */
    crit_dsp_exit();
    x.seq    = g_seq;
    x.t_ms   = hal_time_ms();
    x.state  = sys_state();
    x.ir_min = st_ir_adc();                   /* hal_ir_adc_range() would reset the live range */
    x.ir_max = x.ir_min;
    x.rssi   = net_rssi();
    x.buf    = net_frames_waiting();
    for (uint32_t i = 0u; i < TLM_BENCH_N; i++)
    {
        uint32_t t0 = hal_time_us32();
        uint32_t t1;
        uint32_t us;

        frame_fill(&t, &s_snap, &x);
        t1 = hal_time_us32();
        (void)wire_enc_telemetry(&t, s_scratch);
        us = hal_time_us32() - t0;
        t1 = hal_time_us32() - t1;
        max_us   = (us > max_us) ? us : max_us;
        sum_us  += us;
        enc_max  = (t1 > enc_max) ? t1 : enc_max;
        enc_sum += t1;
    }
    jb_init(&jb, line, sizeof(line));
    jb_raw(&jb, "[perf] tlm_bench n=");
    jb_uint(&jb, TLM_BENCH_N);
    jb_raw(&jb, " fill+enc max=");
    jb_uint(&jb, max_us);
    jb_raw(&jb, " avg=");
    jb_fixed(&jb, (float)sum_us / (float)TLM_BENCH_N, 1u);
    jb_raw(&jb, " enc max=");
    jb_uint(&jb, enc_max);
    jb_raw(&jb, " avg=");
    jb_fixed(&jb, (float)enc_sum / (float)TLM_BENCH_N, 1u);
    log_line(line);
}
#endif /* APP_PERF */

LOCAL void heartbeat(void)
{
    wire_hb_t h;
    uint32_t  n;
    int32_t   rssi = net_rssi();

    zero(&h, sizeof(h));
    h.seq        = g_hb_seq;
    g_hb_seq     = (uint16_t)((g_hb_seq + 1u) & 0x3FFFu);
    h.state      = (uint8_t)sys_state();
    h.rssi       = (uint8_t)((rssi >= 0) ? 0 : ((rssi < -255) ? 255 : -rssi));
    h.t_ms       = hal_time_ms() & 0xFFFFFFu;
    h.uptime_s   = hal_time_ms() / 1000u;
    h.reconnects = (uint16_t)((net_reconnects() > 0xFFFFu) ? 0xFFFFu : net_reconnects());
    h.dropped    = (uint16_t)((net_frames_dropped() > 0xFFFFu) ? 0xFFFFu : net_frames_dropped());
    h.sat        = sat_count();
    h.faults     = (uint16_t)st_faults();
    h.param_crc  = params_table_crc();
    h.buf        = (uint8_t)((net_frames_waiting() > 127u) ? 127u : net_frames_waiting());
    h.n_params   = (uint8_t)params_count();
    (void)net_broker_ip4(h.ip);
    n = wire_enc_heartbeat(&h, g_out);
    if (n > 0u)
    {
        (void)net_post(NET_T_STATUS, g_out, n);
    }
}

LOCAL void publish_event(const evt_msg_t *p_m)
{
    uint32_t n = wire_enc_event(p_m->t_ms & 0xFFFFFFu, &p_m->e, g_out, sizeof(g_out));

    if (n > 0u)
    {
        (void)net_post(NET_T_EVENT, g_out, n);
    }
}

LOCAL void status_leds(bool b_blink)
{
    sys_state_t st = sys_state();
    bool        b_comms = net_mqtt_up();

    switch (st)
    {
        case SYS_IDLE:
            if (((st_faults() & FAULT_IMU_DEAD) != 0u) && b_blink)
            {
                hal_neopixel(40u, 30u, 0u);   /* yellow blink: running without the IMU */
            }
            else if (b_comms)
            {
                hal_neopixel(0u, 0u, 40u);
            }
            else
            {
                hal_neopixel(30u, 0u, 30u);
            }
            break;
        case SYS_LINE_FOLLOWING:     hal_neopixel(0u, 40u, 0u);   break;
        case SYS_MANEUVER:           hal_neopixel(0u, 30u, 30u);  break;
        case SYS_OBSTACLE_AVOIDANCE: hal_neopixel(40u, 25u, 0u);  break;
        case SYS_MISSION_COMPLETE:   hal_neopixel(30u, 30u, 30u); break;
        case SYS_ESTOP:              hal_neopixel(60u, 0u, 0u);   break;
        case SYS_ERROR:
            if (b_blink) { hal_neopixel(60u, 0u, 0u); } else { hal_neopixel(0u, 0u, 0u); }
            break;
        default:                     hal_neopixel(5u, 5u, 5u);    break;
    }
    net_set_led(b_blink);                     /* on-board LED: alive */
}

LOCAL void battery(void)
{
#if VBAT_ENABLED
    uint32_t mv = hal_vbat_mv();

    st_set_vbat(mv);
    if ((mv > 1000u) && (mv < VBAT_CRITICAL_MV))
    {
        flag_set(EVF_BATTERY_CRITICAL);
    }
    else if ((mv > 1000u) && (mv < VBAT_LOW_MV) && ((st_faults() & FAULT_BATTERY_LOW) == 0u))
    {
        fault_set(FAULT_BATTERY_LOW);
        buzzer_request(BUZZ_LOW_BATTERY);
        (void)evt_post0(EVC_BATTERY_LOW);   /* "below VBAT_LOW_MV" */
    }
    else
    {
        /* fine */
    }
#endif
}

LOCAL void console_status(void)
{
    static char line[256];
    jbuf_t      jb;

    jb_init(&jb, line, sizeof(line));
    jb_raw(&jb, "[status] ");
    jb_raw(&jb, sys_state_name(sys_state()));
    jb_raw(&jb, net_mqtt_up() ? "  mqtt up " : (net_wifi_up() ? "  wifi up, mqtt down " : "  wifi down "));
    jb_raw(&jb, net_broker_ip());
    jb_raw(&jb, "  dist ");
    jb_fixed(&jb, st_dist_mm(), 0u);
    jb_raw(&jb, " mm  LR ");
    jb_int(&jb, st_line_l() ? 1 : 0);
    jb_char(&jb, '/');
    jb_int(&jb, st_line_r() ? 1 : 0);
    jb_raw(&jb, "  ir ");
    jb_uint(&jb, st_ir_adc());
    jb_raw(&jb, "  obst ");
    jb_fixed(&jb, st_obst_cm(), 1u);
    jb_raw(&jb, "  pitch ");
    jb_fixed(&jb, st_pitch_deg(), 1u);
    {
        static const char * const NAMES[] =
        {
            "IMU_I2C", "IMU_DEAD", "ENCODER_LOSS", "MOTION_TIMEOUT", "ULTRASONIC", "LINE_LOST",
            "BYPASS_ABORT", "WATCHDOG", "BATTERY_LOW", "BATTERY_CRIT", "CONFIG", "BARCODE", "TELEMETRY"
        };
        uint32_t f = st_faults() & FAULT_BASE_MASK;   /* console keeps the 13 baseline names */

        jb_raw(&jb, "  faults:");
        if (f == 0u)
        {
            jb_raw(&jb, " none");
        }
        for (uint32_t i = 0u; i < (sizeof(NAMES) / sizeof(NAMES[0])); i++)
        {
            if ((f & (1u << i)) != 0u)
            {
                jb_char(&jb, ' ');
                jb_raw(&jb, NAMES[i]);
            }
        }
    }
    if ((sys_state() == SYS_ERROR) || (sys_state() == SYS_ESTOP))
    {
        jb_raw(&jb, "  reason: ");
        jb_raw(&jb, wire_stop_reason_text(st_stop()));
    }
    log_line(line);
    perf_report();                            /* APP_PERF: one [perf] line per probe */
}

void tsk_telemetry(INT stacd, void *exinf)
{
    uint32_t t_frame = hal_time_ms();
    uint32_t t_hb = t_frame;
    uint32_t t_console = t_frame;
    bool     b_blink = false;
#if APP_PERF
    bool     b_bench_done = false;
#endif

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        app_msg_t *p_msg;
        uint32_t   now;

        (void)tk_slp_tsk(100);               /* also woken by commands */
        now = hal_time_ms();

        while (g_cmd_tail != g_cmd_head)
        {
            handle_command(&g_cmds[g_cmd_tail % CMD_SLOTS]);
            g_cmd_tail++;
        }
        estop_ack();
        if (g_config_due)
        {
            handle_config();
        }

        while ((p_msg = msg_recv(g_mbx_tlm, TMO_POL)) != NULL)
        {
            publish_event(&p_msg->body.event);
            msg_free(p_msg);
        }

        if ((now - t_frame) >= FRAME_MS)
        {
            t_frame += FRAME_MS;
            if ((now - t_frame) > FRAME_MS)
            {
                t_frame = now;                /* fell behind: do not burst */
            }
            frame();
            b_blink = !b_blink;
            status_leds(b_blink);
            battery();
        }
        if ((now - t_hb) >= HEARTBEAT_MS)
        {
            t_hb = now;
            heartbeat();
        }
        if ((now - t_console) >= CONSOLE_MS)
        {
            t_console = now;
            console_status();
        }
#if APP_PERF
        if (!b_bench_done && (now >= TLM_BENCH_AT_MS) && (sys_state() == SYS_IDLE))
        {
            b_bench_done = true;              /* once, 10 s after boot, while IDLE */
            tlm_bench();
        }
#endif
    }
}
