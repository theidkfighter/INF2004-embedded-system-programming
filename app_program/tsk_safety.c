/**
 * @file    tsk_safety.c
 * @brief   TSK_SAFETY_MON (DR Section 3 / 5, highest application priority,
 *          10 ms): emergency stop, error state, watchdog, buttons, buzzer.
 *
 * E-stop sources (NFR21, FR23, NFR17): IMU collision (INT1 ISR or the
 * software backup) while driving, critical proximity (< 8 cm), a control
 * task heartbeat missing for 300 ms while driving, a remote MQTT ESTOP, or
 * BTN2. Action: motor outputs low at once (direct PWM write, before anything
 * else), EVF_ESTOP latched, alarm. Recovery: hazard gone + BTN1 + automated
 * re-check (proximity clear, INT1 released, IMU answering) -> IDLE.
 *
 * ERROR: entered on EVF_ERROR_REQ (encoder loss, motion timeout, bypass
 * abort, ...), EVF_IMU_FAULT or EVF_BATTERY_CRITICAL. Motors off, error
 * tone. BTN1 runs the re-check (calibration complete, IMU answering) -> IDLE.
 *
 * This task is the only one that drives the buzzer (DR): other tasks ask
 * for a pattern with buzzer_request().
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "app_check.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define TICK_MS          (10u)
#define DEBOUNCE_TICKS   (3u)

/* buzzer patterns: {frequency Hz (0 = silence), duration ms}, ended by {0,0} */
typedef struct
{
    uint16_t hz;
    uint16_t ms;
} tone_t;

LOCAL const tone_t T_START[]    = { { 1500, 80 }, { 0, 60 }, { 2000, 120 }, { 0, 0 } };
LOCAL const tone_t T_BARCODE[]  = { { 2500, 60 }, { 0, 0 } };
LOCAL const tone_t T_COMPLETE[] = { { 1500, 120 }, { 0, 60 }, { 2000, 120 }, { 0, 60 }, { 2500, 250 }, { 0, 0 } };
LOCAL const tone_t T_ERROR[]    = { { 400, 400 }, { 0, 150 }, { 400, 400 }, { 0, 0 } };
LOCAL const tone_t T_LOWBAT[]   = { { 800, 100 }, { 0, 100 }, { 800, 100 }, { 0, 0 } };
LOCAL const tone_t T_WARN[]     = { { 1000, 200 }, { 0, 0 } };
LOCAL const tone_t T_ALARM[]    = { { 3000, 150 }, { 0, 150 }, { 0, 0 } };   /* repeats while latched */

LOCAL const tone_t *gp_tone;
LOCAL uint32_t      g_tone_left_ms;
LOCAL uint32_t      g_t_estop;

#define ALARM_MS     (3000u)          /* E-stop alarm sounds for 3 s, then the LED stays red */

const char * volatile g_stop_reason = "";

volatile uint64_t g_estop_rx_us;          /* set by the MQTT command handler */
volatile uint32_t g_estop_pwm_off_us;     /* read by the telemetry ack */
volatile bool     g_estop_reported;

LOCAL void tone_start(const tone_t *p_t)
{
    gp_tone        = p_t;
    g_tone_left_ms = p_t->ms;
    hal_buzzer(p_t->hz);
}

LOCAL void tone_tick(bool b_alarm)
{
    if (gp_tone == NULL)
    {
        if (b_alarm)
        {
            tone_start(T_ALARM);
        }
        return;
    }
    if (g_tone_left_ms > TICK_MS)
    {
        g_tone_left_ms -= TICK_MS;
        return;
    }
    gp_tone++;
    if (gp_tone->ms == 0u)
    {
        gp_tone = NULL;
        hal_buzzer(0u);
        return;
    }
    g_tone_left_ms = gp_tone->ms;
    hal_buzzer(gp_tone->hz);
}

LOCAL const tone_t *pattern(uint32_t p)
{
    switch (p)
    {
        case BUZZ_START:       return T_START;
        case BUZZ_BARCODE:     return T_BARCODE;
        case BUZZ_COMPLETE:    return T_COMPLETE;
        case BUZZ_ERROR:       return T_ERROR;
        case BUZZ_LOW_BATTERY: return T_LOWBAT;
        case BUZZ_WARN:        return T_WARN;
        default:               return NULL;
    }
}

LOCAL bool driving(sys_state_t st)
{
    return (st == SYS_LINE_FOLLOWING) || (st == SYS_MANEUVER) || (st == SYS_OBSTACLE_AVOIDANCE);
}

LOCAL void clear_int1(void)
{
    uint8_t src;

    if (tk_loc_mtx(g_mtx_i2c, 20) >= E_OK)
    {
        (void)hal_imu_int1_src(&src);         /* reading INT1_SRC_A releases the latch */
        (void)tk_unl_mtx(g_mtx_i2c);
    }
}

LOCAL void kill(void)
{
    hal_motor_kill();                         /* direct PWM write first (NFR21) */
    flag_set(EVF_ESTOP);
    motion_stop(SRC_SAFETY);
    hal_servo_deg(90.0f);
}

LOCAL void enter_estop(const char *p_reason)
{
    uint64_t t_kill;

    kill();
    t_kill = hal_time_us();
    if (g_estop_rx_us != 0u)
    {
        g_estop_pwm_off_us = (uint32_t)(t_kill - g_estop_rx_us);
        g_estop_rx_us      = 0u;
        g_estop_reported   = false;
    }
    g_stop_reason = p_reason;
    (void)sys_set_state(SYS_ESTOP, true);
    g_t_estop = hal_time_ms();
    (void)post_event("EMERGENCY_STOP", p_reason);
    log_line(p_reason);
    tone_start(T_ALARM);
}

LOCAL void enter_error(const char *p_reason)
{
    kill();
    g_stop_reason = p_reason;
    (void)sys_set_state(SYS_ERROR, true);
    (void)post_event("ERROR", p_reason);
    log_line(p_reason);
    tone_start(T_ERROR);
}

/* Automated re-check before leaving E-STOP / ERROR (DR Section 5). */
LOCAL bool recheck(char *p_why, uint32_t size)
{
    jbuf_t jb;
    bool   b_ok = true;
    float  cm = g_status.obst_cm;

    jb_init(&jb, p_why, size);
    if ((cm >= 0.0f) && (cm < (float)PROXIMITY_STOP_CM))
    {
        jb_raw(&jb, "obstacle closer than 8 cm. ");
        b_ok = false;
    }
    if ((g_status.faults & FAULT_IMU_DEAD) == 0u)
    {
        clear_int1();
        if (hal_imu_int1_level())
        {
            jb_raw(&jb, "IMU INT1 still active. ");
            b_ok = false;
        }
    }
    if (!config_check(&p_why[jb_len(&jb)], size - jb_len(&jb)))
    {
        b_ok = false;                         /* the list of missing values was appended */
    }
    return b_ok;
}

LOCAL void try_recover(void)
{
    char why[200];

    if (recheck(why, sizeof(why)))
    {
        flag_clear(EVF_ESTOP | EVF_COLLISION | EVF_PROXIMITY | EVF_REMOTE_ESTOP | EVF_LOCAL_STOP |
                   EVF_ERROR_REQ | EVF_BATTERY_CRITICAL | EVF_IMU_FAULT);
        fault_clear(~FAULT_IMU_DEAD);           /* IMU state is owned by TSK_TERRAIN_IMU */
        hal_motor_unkill();
        gp_tone = NULL;
        hal_buzzer(0u);
        (void)sys_set_state(SYS_IDLE, true);
        (void)post_event("RECOVERED", "re-check passed");
    }
    else
    {
        (void)post_event("RECOVERY_REFUSED", why);
        log_line("[safety] recovery refused:");
        log_line(why);
        tone_start(T_WARN);
    }
}

void tsk_safety(INT stacd, void *exinf)
{
    uint32_t hb_last[HB_COUNT] = { 0u, 0u, 0u };
    uint32_t hb_quiet_ms[HB_COUNT] = { 0u, 0u, 0u };
    uint32_t btn1_n = 0u;
    uint32_t btn2_n = 0u;
    char     missing[160];

    (void)stacd;
    (void)exinf;

    /* INITIALISING -> IDLE, or ERROR if the calibration is incomplete */
    if (config_check(missing, sizeof(missing)))
    {
        (void)sys_set_state(SYS_IDLE, true);
    }
    else
    {
        fault_set(FAULT_CONFIG);
        log_line("[safety] calibration values missing (robot_config.h or SET:...):");
        log_line(missing);
        enter_error("calibration missing");
    }

    for (;;)
    {
        sys_state_t st;
        uint32_t    f;
        bool        b_btn1 = false;
        bool        b_btn2 = false;

        (void)tk_slp_tsk(TMO_FEVR);          /* every 10 ms, or at once on a hazard */
        st = sys_state();
        f  = flag_peek();
        if (((f & EVF_COLLISION) != 0u) && ((g_status.faults & FAULT_IMU_DEAD) != 0u))
        {
            flag_clear(EVF_COLLISION);            /* no IMU: INT1 is not trustworthy */
            f &= ~EVF_COLLISION;
        }

        /* ---- hazards first ---- */
        if ((st == SYS_ESTOP) && ((f & EVF_REMOTE_ESTOP) != 0u) && (g_estop_rx_us != 0u))
        {
            g_estop_pwm_off_us = 0u;              /* already stopped: ack at once */
            g_estop_rx_us      = 0u;
        }
        if (st != SYS_ESTOP)
        {
            if ((f & EVF_REMOTE_ESTOP) != 0u)
            {
                enter_estop("remote E-stop (MQTT)");
            }
            else if ((f & EVF_LOCAL_STOP) != 0u)
            {
                enter_estop("BTN2 stop");
            }
            else if (((f & EVF_COLLISION) != 0u) && driving(st))
            {
                enter_estop("collision (IMU)");
                clear_int1();
                g_status.motion_event = (uint32_t)MEV_IMPACT;
            }
            else if (((f & EVF_PROXIMITY) != 0u) && driving(st))
            {
                enter_estop("obstacle closer than 8 cm");
            }
            else
            {
                if ((f & EVF_COLLISION) != 0u)
                {
                    flag_clear(EVF_COLLISION);   /* bump while not driving: report only */
                    clear_int1();
                    (void)post_event("IMPACT", "while not driving");
                }
                if ((f & EVF_PROXIMITY) != 0u)
                {
                    flag_clear(EVF_PROXIMITY);
                }
            }
        }
        st = sys_state();
        if ((st != SYS_ESTOP) && (st != SYS_ERROR))
        {
            if ((f & EVF_IMU_FAULT) != 0u)
            {
                enter_error("IMU fault (3 failed I2C recoveries)");
            }
            else if ((f & EVF_BATTERY_CRITICAL) != 0u)
            {
                fault_set(FAULT_BATTERY_CRIT);
                enter_error("battery critical");
            }
            else if ((f & EVF_ERROR_REQ) != 0u)
            {
                enter_error("fault reported (see faults in telemetry)");
            }
            else
            {
                /* no error */
            }
        }

        /* ---- watchdog on the periodic control tasks ---- */
        st = sys_state();
        for (uint32_t i = 0u; i < HB_COUNT; i++)
        {
            uint32_t hb = g_hb[i];

            hb_quiet_ms[i] = (hb == hb_last[i]) ? (hb_quiet_ms[i] + TICK_MS) : 0u;
            hb_last[i]     = hb;
            if ((hb_quiet_ms[i] >= HEARTBEAT_TIMEOUT_MS) && driving(st))
            {
                hb_quiet_ms[i] = 0u;
                fault_set(FAULT_WATCHDOG);
                enter_estop("watchdog: a control task stopped running");
                st = sys_state();
            }
        }

        /* ---- ERROR only because of missing calibration: leave it by itself
                once the values arrive (SET: or the retained is02/config) ---- */
        if ((st == SYS_ERROR) && (g_status.faults == FAULT_CONFIG) && config_check(missing, sizeof(missing)))
        {
            try_recover();
            st = sys_state();
        }

        /* ---- buttons (debounced, act on the press) ---- */
        btn1_n = hal_btn1_down() ? (btn1_n + 1u) : 0u;
        btn2_n = hal_btn2_down() ? (btn2_n + 1u) : 0u;
        b_btn1 = (btn1_n == DEBOUNCE_TICKS);
        b_btn2 = (btn2_n == DEBOUNCE_TICKS);
        if (b_btn2 && (st != SYS_ESTOP))
        {
            flag_set(EVF_LOCAL_STOP);
        }
        if (b_btn1)
        {
            if ((st == SYS_ESTOP) || (st == SYS_ERROR))
            {
                try_recover();
            }
            else
            {
                (void)post_nav(NAV_BTN1, 0);
            }
        }

        /* ---- buzzer (sole driver of GP22) ---- */
        if ((flag_peek() & EVF_BUZZER_REQ) != 0u)
        {
            const tone_t *p_t;

            flag_clear(EVF_BUZZER_REQ);
            p_t = pattern(buzzer_pattern_take());
            if ((p_t != NULL) && (sys_state() != SYS_ESTOP))
            {
                tone_start(p_t);
            }
        }
        tone_tick((sys_state() == SYS_ESTOP) && ((hal_time_ms() - g_t_estop) < ALARM_MS));
    }
}
