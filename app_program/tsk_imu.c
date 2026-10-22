/**
 * @file    tsk_imu.c
 * @brief   TSK_TERRAIN_IMU (DR Section 3 / 6.3, 20 ms): pitch, hump
 *          detection and height, collision backup, motion events.
 *
 * Every 20 ms, under MTX_I2C_BUS: read the accelerometer, pitch =
 * atan2(a_fwd_axis - a_fwd, sqrt(other^2 + z^2)) - theta0, EMA filtered.
 * Region A (hump profiler, imu_logic.c) sets EVF_HUMP_START / PEAK / END and
 * updates the run maximum (FR21, FR22). Region B classifies motion events
 * (FR24). Software collision backup: |a - lowpass(a)| >= 2 g -> EVF_COLLISION.
 * I2C fault: bus recovery (9 SCL clocks + re-init). After 3 failed recoveries
 * (or if the IMU did not answer at start-up) the robot keeps driving without
 * hump / collision detection (FAULT_IMU_DEAD in telemetry, NeoPixel blinks
 * blue/yellow in IDLE) and the IMU is re-tried every 2 s. 21 Oct change:
 * the DR sends this case to ERROR; degraded operation was chosen so a loose
 * IMU wire does not stop the whole run.
 *
 * theta0 (mounting offset) is averaged over 50 samples whenever the robot
 * is in IDLE and standing still, so it is fresh at every start.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "imu_logic.h"
#include "app_params.h"
#include "hal.h"
#include "jfmt.h"
#include "robot_config.h"

#define OFFSET_SAMPLES   (50u)
#define MAX_RECOVERIES   (3u)
#define HP_ALPHA         (0.05f)     /* slow low-pass = gravity estimate */

LOCAL bool imu_read(float *p_x, float *p_y, float *p_z)
{
    bool b_ok;

    if (tk_loc_mtx(g_mtx_i2c, 5) < E_OK)
    {
        return false;
    }
    b_ok = hal_imu_accel_g(p_x, p_y, p_z);
    (void)tk_unl_mtx(g_mtx_i2c);
    return b_ok;
}

LOCAL bool imu_recover(void)
{
    bool b_ok;

    if (tk_loc_mtx(g_mtx_i2c, 50) < E_OK)
    {
        return false;
    }
    hal_i2c_recover();
    b_ok = hal_imu_init();
    (void)tk_unl_mtx(g_mtx_i2c);
    return b_ok;
}

LOCAL void hump_event(const char *p_name, float value_mm)
{
    char   text[TLM_EVENT_DETAIL_LEN];
    jbuf_t jb;

    jb_init(&jb, text, sizeof(text));
    jb_raw(&jb, "height ");
    jb_fixed(&jb, value_mm, 1u);
    jb_raw(&jb, " mm, run max ");
    jb_fixed(&jb, g_status.hump_max_mm, 1u);
    jb_raw(&jb, " mm");
    (void)post_event(p_name, text);
}

void tsk_imu(INT stacd, void *exinf)
{
    hump_params_t hp;
    hump_ctx_t    hump;
    mev_ctx_t     mev;
    odometry_t    odo;
    uint32_t      odo_prev = 0u;
    float         pitch = 0.0f;
    float         theta0 = 0.0f;
    float         off_sum = 0.0f;
    uint32_t      off_n = 0u;
    float         lp[3] = { 0.0f, 0.0f, 1.0f };
    uint32_t      fails = 0u;
    uint32_t      recoveries = 0u;
    bool          b_dead = ((g_status.faults & FAULT_IMU_DEAD) != 0u);
    uint32_t      retry_n = 0u;
    uint32_t      warm = 0u;

    (void)stacd;
    (void)exinf;
    hp.start_deg     = HUMP_START_DEG;
    hp.level_deg     = HUMP_LEVEL_DEG;
    hp.min_height_mm = HUMP_MIN_HEIGHT_MM;
    hp.start_samples = HUMP_START_SAMPLES;
    hp.end_samples   = HUMP_END_SAMPLES;
    hp.max_len_mm    = 1500.0f;
    hump_reset(&hump);
    mev_reset(&mev);

    for (;;)
    {
        float       ax;
        float       ay;
        float       az;
        float       raw;
        float       ds;
        float       hp_mag;
        bool        b_impact = false;
        sys_state_t st;
        hump_out_t  ho;

        (void)tk_slp_tsk(TMO_FEVR);          /* released every 20 ms by the cyclic tick */
        g_hb[HB_IMU]++;
        st = sys_state();
        odometry_get(&odo);
        ds       = (float)(odo.odo_um - odo_prev) / 1000.0f;
        odo_prev = odo.odo_um;

        if (b_dead)
        {
            retry_n++;
            if (retry_n >= 100u)                /* every 2 s */
            {
                retry_n = 0u;
                if (imu_recover())
                {
                    b_dead     = false;
                    recoveries = 0u;
                    warm       = 0u;
                    fault_clear(FAULT_IMU_DEAD | FAULT_IMU_I2C);
                    (void)post_event("IMU_OK", "GY-511 answering again");
                    log_line("[imu] GY-511 answering: hump and collision detection on");
                }
            }
            continue;
        }
        if (!imu_read(&ax, &ay, &az))
        {
            fails++;
            fault_set(FAULT_IMU_I2C);
            hump_reset(&hump);               /* discard any hump in progress */
            if (fails >= 3u)
            {
                fails = 0u;
                (void)post_event("FAULT_IMU_I2C", "I2C1 timeout: bus recovery");
                if (imu_recover())
                {
                    recoveries = 0u;
                }
                else
                {
                    recoveries++;
                    if (recoveries >= MAX_RECOVERIES)
                    {
                        b_dead = true;
                        fault_set(FAULT_IMU_DEAD);
                        (void)post_event("FAULT_IMU", "3 failed recoveries: driving without the IMU");
                        log_line("[imu] GY-511 lost: driving without hump/collision detection");
                    }
                }
            }
            continue;
        }
        fails = 0u;

        /* software collision backup: high-pass magnitude (gravity removed) */
        lp[0] = ema(lp[0], ax, HP_ALPHA);
        lp[1] = ema(lp[1], ay, HP_ALPHA);
        lp[2] = ema(lp[2], az, HP_ALPHA);
        {
            float dx = ax - lp[0];
            float dy = ay - lp[1];
            float dz = az - lp[2];

            hp_mag = (dx * dx) + (dy * dy) + (dz * dz);
        }
        if (warm < OFFSET_SAMPLES)
        {
            warm++;                          /* let the gravity estimate settle first */
        }
        else if (hp_mag >= (IMPACT_G * IMPACT_G))
        {
            b_impact = true;
            flag_set(EVF_COLLISION);
            hump_reset(&hump);
        }

        if ((g_params.imu_forward_axis < 0.5f) ||
            ((g_params.imu_forward_sign < 0.5f) && (g_params.imu_forward_sign > -0.5f)))
        {
            continue;                        /* no pitch until test_imu U5 is copied in */
        }
        raw = imu_pitch_deg(ax, ay, az, (g_params.imu_forward_axis > 1.5f) ? 2 : 1,
                            (g_params.imu_forward_sign < 0.0f) ? -1 : 1, odo.a_fwd_g);

        /* mounting offset: re-learnt while idle and still */
        if ((st == SYS_IDLE) && ((odo.v_mm_s < 5.0f) && (odo.v_mm_s > -5.0f)))
        {
            off_sum += raw;
            off_n++;
            if (off_n >= OFFSET_SAMPLES)
            {
                theta0  = off_sum / (float)off_n;
                off_sum = 0.0f;
                off_n   = 0u;
            }
        }
        else
        {
            off_sum = 0.0f;
            off_n   = 0u;
        }
        pitch = ema(pitch, raw - theta0, IMU_EMA_ALPHA);

        /* Region A: humps, only while driving */
        if ((st == SYS_LINE_FOLLOWING) || (st == SYS_OBSTACLE_AVOIDANCE) || (st == SYS_MANEUVER))
        {
            ho = hump_step(&hump, &hp, pitch, ds);
            if (ho.b_start)
            {
                flag_set(EVF_HUMP_START);
                hump_event("HUMP_START", 0.0f);
            }
            if (ho.b_peak)
            {
                flag_set(EVF_HUMP_PEAK);
                hump_event("HUMP_PEAK", hump.peak_mm);
            }
            if (ho.b_end)
            {
                if (ho.b_valid)
                {
                    g_status.humps++;
                    if (ho.height_mm > g_status.hump_max_mm)
                    {
                        g_status.hump_max_mm = ho.height_mm;
                    }
                    g_status.hump_peak_mm = ho.height_mm;
                    hump_event("HUMP_END", ho.height_mm);
                }
                else
                {
                    (void)post_event("HUMP_END", "false slope (no height reported)");
                }
                flag_set(EVF_HUMP_END);
            }
        }
        else if (hump.state != HUMP_FLAT)
        {
            hump_reset(&hump);
            flag_set(EVF_HUMP_END);
        }
        else
        {
            /* flat and not driving */
        }

        /* Region B: motion events (turn rate from the encoders, DR A10 fallback) */
        g_status.motion_event  = (uint32_t)mev_step(&mev, b_impact, hump.state, odo.turn_dps,
                                                    odo.a_fwd_g, odo.v_mm_s, 20u);
        g_status.turn_rate_dps = odo.turn_dps;
        g_status.pitch_deg     = pitch;
        g_status.hump_mm       = hump.h_mm;
        if (hump.state != HUMP_FLAT)
        {
            g_status.hump_peak_mm = hump.peak_mm;
        }
    }
}
