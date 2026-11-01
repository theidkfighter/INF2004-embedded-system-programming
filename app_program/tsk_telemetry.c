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

#include "os_glue.h"
#include "app_net.h"
#include "app_params.h"
#include "hal.h"
#include "perf.h"
#include "jfmt.h"
#include "robot_config.h"
#include "nav_map.h"

#define FRAME_MS        (500u)
#define HEARTBEAT_MS    (1000u)
#define CONSOLE_MS      (2000u)
#define CMD_SLOTS       (8u)
#define CMD_LEN         (96u)

IMPORT volatile uint64_t g_estop_rx_us;      /* tsk_safety.c */
IMPORT volatile uint32_t g_estop_pwm_off_us;
IMPORT volatile bool     g_estop_reported;

LOCAL char              g_cmds[CMD_SLOTS][CMD_LEN];
LOCAL volatile uint32_t g_cmd_head;          /* radio task */
LOCAL volatile uint32_t g_cmd_tail;          /* this task */
LOCAL volatile bool     g_estop_ack_due;
LOCAL char              g_config[1024];
LOCAL volatile bool     g_config_due;
LOCAL char              g_buf[NET_MSG_MAX];
LOCAL uint32_t          g_seq;
LOCAL uint32_t          g_hb_seq;

/* ------------------------------------------------------------------------ */
/* Network context: called by app_net.c in the radio-owner task              */
/* ------------------------------------------------------------------------ */

void app_net_command(const char *p_cmd, uint64_t rx_us)
{
    if (jf_starts_with(p_cmd, "ESTOP"))
    {
        /* NFR17 fast path: straight to the safety monitor, no queueing */
        g_estop_rx_us   = rx_us;
        g_estop_ack_due = true;
        flag_set(EVF_REMOTE_ESTOP);
        (void)tk_wup_tsk(g_tsk_safety);
        return;
    }
    if ((g_cmd_head - g_cmd_tail) < CMD_SLOTS)
    {
        jf_copy(g_cmds[g_cmd_head % CMD_SLOTS], p_cmd, CMD_LEN);
        g_cmd_head++;
        (void)tk_wup_tsk(g_tsk_tlm);
    }
}

void app_net_config(const char *p_text)
{
    if (!g_config_due)
    {
        jf_copy(g_config, p_text, sizeof(g_config));
        g_config_due = true;
        (void)tk_wup_tsk(g_tsk_tlm);
    }
}

/* Pushes the parameters the drivers keep their own copy of. */
void params_apply(void)
{
    hal_enc_set_reversed(g_params.enc_r_reversed > 0.5f, g_params.enc_l_reversed > 0.5f);
    hal_servo_range((uint32_t)g_params.servo_min_us, (uint32_t)g_params.servo_max_us);
}

/* ------------------------------------------------------------------------ */

LOCAL void ack(const char *p_cmd, const char *p_result, const char *p_extra_json)
{
    jbuf_t jb;

    jb_init(&jb, g_buf, sizeof(g_buf));
    jb_raw(&jb, "{\"t\":");
    jb_uint(&jb, hal_time_ms());
    jb_raw(&jb, ",\"cmd\":");
    jb_str(&jb, p_cmd);
    jb_raw(&jb, ",\"result\":");
    jb_str(&jb, p_result);
    if (p_extra_json != NULL)
    {
        jb_char(&jb, ',');
        jb_raw(&jb, p_extra_json);
    }
    jb_char(&jb, '}');
    (void)net_post(NET_T_ACK, g_buf);
}

/* Acks with the current map: {"map":"F=FWD,K=LEFT,...","n":4} */
LOCAL void map_ack(const char *p_cmd, const char *p_result)
{
    char     m[80];
    char     extra[112];
    jbuf_t   jb;
    uint32_t n = nav_map_describe(m, sizeof(m));

    jb_init(&jb, extra, sizeof(extra));
    jb_raw(&jb, "\"map\":");
    jb_str(&jb, m);
    jb_raw(&jb, ",\"n\":");
    jb_uint(&jb, n);
    ack(p_cmd, p_result, extra);
}

/* MAP:... from is02/cmd, or the retained is02/map payload */
LOCAL void map_apply(const char *p_cmd_name, const char *p_text)
{
    char    err[64];
    int32_t n = nav_map_parse(p_text, err, sizeof(err));

    if (n < 0)
    {
        char reason[80];
        jbuf_t jb;

        jb_init(&jb, reason, sizeof(reason));
        jb_raw(&jb, "nack: ");
        jb_raw(&jb, err);
        ack(p_cmd_name, reason, NULL);
        return;
    }
    map_ack(p_cmd_name, (n == 4) ? "ok" : ((n == 0) ? "cleared" : "ok (not 4 letters)"));
    {
        char m[80];

        (void)nav_map_describe(m, sizeof(m));
        (void)post_event("MAP", (m[0] != '\0') ? m : "cleared");
    }
}

LOCAL void handle_command(const char *p_cmd)
{
    sys_state_t st = sys_state();

    if (jf_starts_with(p_cmd, "RESUME"))
    {
        ack(p_cmd, "refused: clear the hazard and press BTN1 on the robot", NULL);
    }
    else if (jf_starts_with(p_cmd, "BARCODE:"))
    {
        char c = p_cmd[8];

        if ((c < 'A') || (c > 'Z'))
        {
            ack(p_cmd, "nack: use BARCODE:<A-Z>", NULL);
        }
        else if (st != SYS_LINE_FOLLOWING)
        {
            ack(p_cmd, "nack: only while LINE_FOLLOWING", NULL);
        }
        else
        {
            (void)post_nav(NAV_REMOTE_BARCODE, c);
            ack(p_cmd, "queued", NULL);
        }
    }
    else if (jf_starts_with(p_cmd, "START"))
    {
        (void)post_nav(NAV_START, 0);
        ack(p_cmd, (st == SYS_IDLE) ? "starting" : "nack: only from IDLE", NULL);
    }
    else if (jf_starts_with(p_cmd, "STOP"))
    {
        (void)post_nav(NAV_STOP, 0);
        ack(p_cmd, "ending the run", NULL);
    }
    else if (jf_starts_with(p_cmd, "MAP:"))
    {
        map_apply(p_cmd, p_cmd);
    }
    else if (jf_starts_with(p_cmd, "GET:MAP"))
    {
        map_ack(p_cmd, "ok");
    }
    else if (jf_starts_with(p_cmd, "PING"))
    {
        ack(p_cmd, "pong", NULL);
    }
    else if (jf_starts_with(p_cmd, "SET:"))
    {
        int32_t r = params_set(&p_cmd[4]);

        params_apply();
        ack(p_cmd, (r == 0) ? "ok" : ((r == -1) ? "nack: unknown parameter"
                 : ((r == -2) ? "nack: bad number" : "nack: out of range")), NULL);
    }
    else if (jf_starts_with(p_cmd, "GET:PARAMS"))
    {
        static char s_params[600];
        static char s_extra[680];
        jbuf_t      jb;
        uint32_t    total = params_count();

        /* sent in parts of 12 so each ack stays short */
        for (uint32_t first = 0u; first < total; first += 12u)
        {
            (void)params_to_json(s_params, sizeof(s_params), first, 12u);
            jb_init(&jb, s_extra, sizeof(s_extra));
            jb_raw(&jb, "\"part\":");
            jb_uint(&jb, (first / 12u) + 1u);
            jb_raw(&jb, ",\"params\":");
            jb_raw(&jb, s_params);
            ack(p_cmd, "ok", s_extra);
        }
    }
    else
    {
        ack(p_cmd, "nack: unknown command", NULL);
    }
}

LOCAL void estop_ack(void)
{
    char   extra[48];
    jbuf_t jb;

    if (!g_estop_ack_due || (g_estop_rx_us != 0u))
    {
        return;                               /* the safety monitor has not acted yet */
    }
    g_estop_ack_due = false;
    jb_init(&jb, extra, sizeof(extra));
    jb_raw(&jb, "\"pwm_off_us\":");
    jb_uint(&jb, g_estop_pwm_off_us);
    ack("ESTOP", "motors off (press BTN1 on the robot to recover)", extra);
}

/* ------------------------------------------------------------------------ */

/* Values of a frame that do not come from g_status */
typedef struct
{
    uint32_t    seq;
    uint32_t    t_ms;
    sys_state_t state;
    uint32_t    ir_min;
    uint32_t    ir_max;
    int32_t     rssi;
    uint32_t    buf;
} frame_extra_t;

/* Builds one JSON telemetry frame from p_s (g_status, or a fixed copy for
   the APP_PERF benchmark) and p_x into p_jb. */
LOCAL void frame_build(jbuf_t *p_jb, const robot_status_t *p_s, const frame_extra_t *p_x)
{
    char bc[2] = { p_s->barcode, '\0' };
    char side[2] = { p_s->scan_side, '\0' };

    jb_raw(p_jb, "{\"seq\":");      jb_uint(p_jb, p_x->seq);
    jb_raw(p_jb, ",\"t\":");        jb_uint(p_jb, p_x->t_ms);
    jb_raw(p_jb, ",\"state\":");    jb_str(p_jb, sys_state_name(p_x->state));
    jb_raw(p_jb, ",\"spd\":[");     jb_fixed(p_jb, p_s->spd_l, 1u);
    jb_char(p_jb, ',');             jb_fixed(p_jb, p_s->spd_r, 1u);
    jb_raw(p_jb, "],\"duty\":[");   jb_fixed(p_jb, 100.0f * p_s->duty_l, 1u);
    jb_char(p_jb, ',');             jb_fixed(p_jb, 100.0f * p_s->duty_r, 1u);
    jb_raw(p_jb, "],\"enc\":[");    jb_int(p_jb, p_s->enc_l);
    jb_char(p_jb, ',');             jb_int(p_jb, p_s->enc_r);
    jb_raw(p_jb, "],\"dist\":");    jb_fixed(p_jb, p_s->dist_mm, 1u);
    jb_raw(p_jb, ",\"line\":{\"l\":"); jb_int(p_jb, p_s->line_l);
    jb_raw(p_jb, ",\"r\":");        jb_int(p_jb, p_s->line_r);
    jb_raw(p_jb, ",\"ir\":");       jb_uint(p_jb, p_s->ir_adc);
    jb_raw(p_jb, ",\"irmn\":");     jb_uint(p_jb, p_x->ir_min);
    jb_raw(p_jb, ",\"irmx\":");     jb_uint(p_jb, p_x->ir_max);
    jb_raw(p_jb, ",\"thr\":[");     jb_uint(p_jb, (p_s->bc_enabled != 0u) ? p_s->bc_thr_black : 0u);
    jb_char(p_jb, ',');             jb_uint(p_jb, (p_s->bc_enabled != 0u) ? p_s->bc_thr_white : 0u);
    jb_char(p_jb, ']');             /* [black, white] switch levels; [0,0] = sampling off */
    jb_raw(p_jb, "},\"bc\":");      jb_str(p_jb, bc);
    jb_raw(p_jb, ",\"hump\":{\"pitch\":"); jb_fixed(p_jb, p_s->pitch_deg, 1u);
    jb_raw(p_jb, ",\"h\":");        jb_fixed(p_jb, p_s->hump_mm, 1u);
    jb_raw(p_jb, ",\"peak\":");     jb_fixed(p_jb, p_s->hump_peak_mm, 1u);
    jb_raw(p_jb, ",\"max\":");      jb_fixed(p_jb, p_s->hump_max_mm, 1u);
    jb_raw(p_jb, "},\"obst\":{\"cm\":"); jb_fixed(p_jb, p_s->obst_cm, 1u);
    jb_raw(p_jb, ",\"scan\":[");
    for (uint32_t i = 0u; i < 5u; i++)
    {
        if (i > 0u)
        {
            jb_char(p_jb, ',');
        }
        jb_fixed(p_jb, p_s->scan_cm[i], 1u);
    }
    jb_raw(p_jb, "],\"side\":");    jb_str(p_jb, side);
    jb_raw(p_jb, "},\"imu\":{\"ev\":"); jb_str(p_jb, motion_event_name((motion_event_t)p_s->motion_event));
    jb_raw(p_jb, ",\"tr\":");       jb_fixed(p_jb, p_s->turn_rate_dps, 1u);
    jb_raw(p_jb, "},\"faults\":");  jb_uint(p_jb, p_s->faults);
    jb_raw(p_jb, ",\"vbat\":");     jb_uint(p_jb, p_s->vbat_mv);
    jb_raw(p_jb, ",\"rssi\":");     jb_int(p_jb, p_x->rssi);
    jb_raw(p_jb, ",\"buf\":");      jb_uint(p_jb, p_x->buf);
    jb_char(p_jb, '}');
}

LOCAL void frame(void)
{
    static char   s_frame[NET_FRAME_MAX];
    jbuf_t        jb;
    frame_extra_t x;
    PERF_BEGIN(t_perf);                       /* PERF_TLM_ENCODE: snapshot read to complete payload */

    x.seq   = g_seq++;
    x.t_ms  = hal_time_ms();
    x.state = sys_state();
    hal_ir_adc_range(&x.ir_min, &x.ir_max);   /* barcode sensor range over the last 0.5 s */
    x.rssi  = net_rssi();
    x.buf   = net_frames_waiting();
    jb_init(&jb, s_frame, sizeof(s_frame));
    frame_build(&jb, &g_status, &x);
    PERF_END(PERF_TLM_ENCODE, t_perf, PERF_NO_LIMIT);
    if (!jb.b_overflow)
    {
        (void)net_post_frame(s_frame, jb_len(&jb));
    }
}

#if APP_PERF
#define TLM_BENCH_AT_MS  (10000u)
#define TLM_BENCH_N      (1000u)

/* Requirement 20.4 benchmark: TLM_BENCH_N builds of one fixed snapshot into
   a scratch buffer (nothing is published), each timed on its own. */
LOCAL void tlm_bench(void)
{
    static robot_status_t s_snap;
    static char           s_scratch[NET_FRAME_MAX];
    frame_extra_t         x;
    jbuf_t                jb;
    uint32_t              max_us = 0u;
    uint32_t              sum_us = 0u;
    char                  line[80];

    s_snap   = g_status;
    x.seq    = g_seq;
    x.t_ms   = hal_time_ms();
    x.state  = sys_state();
    x.ir_min = s_snap.ir_adc;                 /* hal_ir_adc_range() would reset the live range */
    x.ir_max = s_snap.ir_adc;
    x.rssi   = net_rssi();
    x.buf    = net_frames_waiting();
    for (uint32_t i = 0u; i < TLM_BENCH_N; i++)
    {
        uint32_t t0 = hal_time_us32();
        uint32_t us;

        jb_init(&jb, s_scratch, sizeof(s_scratch));
        frame_build(&jb, &s_snap, &x);
        us = hal_time_us32() - t0;
        max_us = (us > max_us) ? us : max_us;
        sum_us += us;
    }
    jb_init(&jb, line, sizeof(line));
    jb_raw(&jb, "[perf] tlm_bench n=");
    jb_uint(&jb, TLM_BENCH_N);
    jb_raw(&jb, " max=");
    jb_uint(&jb, max_us);
    jb_raw(&jb, " avg=");
    jb_fixed(&jb, (float)sum_us / (float)TLM_BENCH_N, 1u);
    log_line(line);
}
#endif /* APP_PERF */

LOCAL void heartbeat(void)
{
    jbuf_t jb;

    jb_init(&jb, g_buf, sizeof(g_buf));
    jb_raw(&jb, "{\"seq\":");      jb_uint(&jb, g_hb_seq++);
    jb_raw(&jb, ",\"t\":");        jb_uint(&jb, hal_time_ms());
    jb_raw(&jb, ",\"state\":");    jb_str(&jb, sys_state_name(sys_state()));
    jb_raw(&jb, ",\"rssi\":");     jb_int(&jb, net_rssi());
    jb_raw(&jb, ",\"buf\":");      jb_uint(&jb, net_frames_waiting());
    jb_raw(&jb, ",\"up\":");       jb_uint(&jb, hal_time_ms() / 1000u);
    jb_raw(&jb, ",\"reconnects\":"); jb_uint(&jb, net_reconnects());
    jb_raw(&jb, ",\"dropped\":");  jb_uint(&jb, net_frames_dropped());
    jb_raw(&jb, ",\"faults\":");   jb_uint(&jb, g_status.faults);
    jb_raw(&jb, ",\"broker\":");   jb_str(&jb, net_broker_ip());
    jb_char(&jb, '}');
    (void)net_post(NET_T_STATUS, g_buf);
}

LOCAL void publish_event(const tlm_event_t *p_ev)
{
    jbuf_t jb;

    jb_init(&jb, g_buf, sizeof(g_buf));
    jb_raw(&jb, "{\"t\":");        jb_uint(&jb, hal_time_ms());
    jb_raw(&jb, ",\"event\":");    jb_str(&jb, p_ev->name);
    jb_raw(&jb, ",\"detail\":");   jb_str(&jb, p_ev->detail);
    jb_char(&jb, '}');
    (void)net_post(NET_T_EVENT, g_buf);
}

LOCAL void status_leds(bool b_blink)
{
    sys_state_t st = sys_state();
    bool        b_comms = net_mqtt_up();

    switch (st)
    {
        case SYS_IDLE:
            if (((g_status.faults & FAULT_IMU_DEAD) != 0u) && b_blink)
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

    g_status.vbat_mv = mv;
    if ((mv > 1000u) && (mv < VBAT_CRITICAL_MV))
    {
        flag_set(EVF_BATTERY_CRITICAL);
    }
    else if ((mv > 1000u) && (mv < VBAT_LOW_MV) && ((g_status.faults & FAULT_BATTERY_LOW) == 0u))
    {
        fault_set(FAULT_BATTERY_LOW);
        buzzer_request(BUZZ_LOW_BATTERY);
        (void)post_event("BATTERY_LOW", "below VBAT_LOW_MV");
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
    jb_fixed(&jb, g_status.dist_mm, 0u);
    jb_raw(&jb, " mm  LR ");
    jb_int(&jb, g_status.line_l);
    jb_char(&jb, '/');
    jb_int(&jb, g_status.line_r);
    jb_raw(&jb, "  ir ");
    jb_uint(&jb, g_status.ir_adc);
    jb_raw(&jb, "  obst ");
    jb_fixed(&jb, g_status.obst_cm, 1u);
    jb_raw(&jb, "  pitch ");
    jb_fixed(&jb, g_status.pitch_deg, 1u);
    {
        static const char * const NAMES[] =
        {
            "IMU_I2C", "IMU_DEAD", "ENCODER_LOSS", "MOTION_TIMEOUT", "ULTRASONIC", "LINE_LOST",
            "BYPASS_ABORT", "WATCHDOG", "BATTERY_LOW", "BATTERY_CRIT", "CONFIG", "BARCODE", "TELEMETRY"
        };
        uint32_t f = g_status.faults;

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
        jb_raw(&jb, g_stop_reason);
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
            handle_command(g_cmds[g_cmd_tail % CMD_SLOTS]);
            g_cmd_tail++;
        }
        estop_ack();
        if (g_config_due)
        {
            uint32_t    n_ok;
            uint32_t    n_bad;
            static char s_bad[200];
            char        extra[64];
            jbuf_t      jb;

            params_set_many(g_config, &n_ok, &n_bad, s_bad, sizeof(s_bad));
            g_config_due = false;
            params_apply();
            jb_init(&jb, extra, sizeof(extra));
            jb_raw(&jb, "\"applied\":");
            jb_uint(&jb, n_ok);
            jb_raw(&jb, ",\"rejected\":");
            jb_uint(&jb, n_bad);
            ack("CONFIG", (n_bad == 0u) ? "ok" : s_bad, extra);
            log_line((n_bad == 0u) ? "[config] is02/config applied" : "[config] is02/config applied, some rejected:");
            if (n_bad > 0u)
            {
                log_line(s_bad);
            }
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
