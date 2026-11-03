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
#include "status.h"
#include "imu_logic.h"
#include "app_params.h"
#include "hal.h"
#include "perf.h"
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

/* HUMP_START / HUMP_PEAK / HUMP_END: valid U1, height U14, run max U14
   (both 0.1 mm, 0..1500.0 mm). b_valid = false: false slope, no height. */
LOCAL uint32_t mm_u14(float mm)
{
    float c = (mm * 10.0f) + 0.5f;

    return (mm != mm) ? 0u : ((c <= 0.0f) ? 0u : ((c >= 15000.0f) ? 15000u : (uint32_t)c));
}

LOCAL void hump_event(evt_code_t code, bool b_valid, float value_mm)
{
    evt_b_t b;

    evt_begin(&b, code);
    evt_u(&b, b_valid ? 1u : 0u, 1u);
    evt_u(&b, b_valid ? mm_u14(value_mm) : 0u, 14u);
    evt_u(&b, mm_u14(st_hump_max_mm()), 14u);
    (void)evt_post(&b);
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
    bool          b_dead = ((st_faults() & FAULT_IMU_DEAD) != 0u);
    uint32_t      retry_n = 0u;
    uint32_t      warm = 0u;
#if HUMP_TILT_DETECT
    /* tilt hump detector (robot_config.h HUMP_TILT_DETECT, imu_logic.h) */
    hump_params_t tp;
    tilt_hump_t   th;
    float         gb[3] = { 0.0f, 0.0f, 1.0f };    /* baseline: slow-moving driving posture */
    float         gf[3] = { 0.0f, 0.0f, 0.0f };    /* low-passed gravity vector */
    bool          b_gf = false;
    uint32_t      drive_ms = 0u;                   /* time since driving started */
#endif

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
#if HUMP_TILT_DETECT
    tp               = hp;
    tp.start_deg     = HUMP_TILT_START_DEG;
    tp.level_deg     = HUMP_TILT_LEVEL_DEG;
    tp.min_height_mm = HUMP_TILT_MIN_MM;
    tilt_hump_reset(&th);
#endif

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

        PERF_CYCLE_END(PERF_TSK_IMU, PERF_LIMIT_20MS_US);      /* end of the previous cycle (also after "continue") */
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
                    {
                        evt_b_t b;                    /* chip E2, accel address U7 */

                        evt_begin(&b, EVC_IMU_OK);
                        evt_u(&b, (uint32_t)hal_imu_chip(), 2u);
                        evt_u(&b, hal_imu_addr(), 7u);
                        (void)evt_post(&b);
                    }
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
#if HUMP_TILT_DETECT
            tilt_hump_reset(&th);
#endif
            if (fails >= 3u)
            {
                fails = 0u;
                (void)evt_post0(EVC_FAULT_IMU_I2C);   /* "I2C1 timeout: bus recovery" */
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
                        (void)evt_post0(EVC_FAULT_IMU);       /* "3 failed recoveries: driving without the IMU" */
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
#if HUMP_TILT_DETECT
            tilt_hump_reset(&th);
#endif
        }

        if ((prm_int(PRM_IMU_FORWARD_AXIS) < 1) ||
            (prm_tri(PRM_IMU_FORWARD_SIGN) == 0))
        {
            continue;                        /* no pitch until test_imu U5 is copied in */
        }
        raw = imu_pitch_deg(ax, ay, az, (prm_int(PRM_IMU_FORWARD_AXIS) > 1) ? 2 : 1,
                            (prm_tri(PRM_IMU_FORWARD_SIGN) < 0) ? -1 : 1, odo.a_fwd_g);

#if HUMP_TILT_DETECT
        /* gravity vector with the forward acceleration removed, as for pitch */
        float    gv[3] = { ax, ay, az };
        uint32_t fi = (prm_int(PRM_IMU_FORWARD_AXIS) > 1) ? 1u : 0u;

        gv[fi] -= ((prm_tri(PRM_IMU_FORWARD_SIGN) < 0) ? -1.0f : 1.0f) * odo.a_fwd_g;
#endif
        /* mounting offset: re-learnt while idle and still (and the level
           gravity vector for the tilt detector) */
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
#if HUMP_TILT_DETECT
        if (!b_gf)
        {
            gf[0] = gv[0];
            gf[1] = gv[1];
            gf[2] = gv[2];
            b_gf  = true;
        }
        gf[0] = ema(gf[0], gv[0], HUMP_TILT_ALPHA);
        gf[1] = ema(gf[1], gv[1], HUMP_TILT_ALPHA);
        gf[2] = ema(gf[2], gv[2], HUMP_TILT_ALPHA);
#endif

#if HUMP_TILT_DETECT
        /* Region A: tilt hump detector, only while driving. While
           turning, only the pitch counts: the pull of a curve reads as tilt. */
        if ((st == SYS_LINE_FOLLOWING) || (st == SYS_OBSTACLE_AVOIDANCE) || (st == SYS_MANEUVER))
        {
            bool  b_turning = (odo.turn_dps > HUMP_TILT_MAX_TURN_DPS) || (odo.turn_dps < -HUMP_TILT_MAX_TURN_DPS);
            float t;

            /* Tilt is measured against a BASELINE that follows the robot's own
               driving posture (caster trailing, motor torque: a few degrees that
               a rest-level reference reads as a permanent hump, 3 Nov runs).
               The baseline is frozen during a hump, so the hump stands out. */
            if (drive_ms < HUMP_TILT_BLANK_MS)
            {
                drive_ms += 20u;
                gb[0] = gf[0];                       /* start jolt: no detection; settle the baseline */
                gb[1] = gf[1];
                gb[2] = gf[2];
                t     = 0.0f;
            }
            else
            {
                if (!tilt_hump_on(&th))
                {
                    gb[0] = ema(gb[0], gf[0], HUMP_TILT_BASE_ALPHA);
                    gb[1] = ema(gb[1], gf[1], HUMP_TILT_BASE_ALPHA);
                    gb[2] = ema(gb[2], gf[2], HUMP_TILT_BASE_ALPHA);
                }
                t = b_turning ? ((pitch < 0.0f) ? -pitch : pitch) : imu_tilt_deg(gf, gb);
            }
            ho = tilt_hump_step(&th, &tp, pitch, t, HUMP_TILT_LEVER_MM, ds);
            if (ho.b_start)
            {
                flag_set(EVF_HUMP_START);
                hump_event(EVC_HUMP_START, true, 0.0f);
            }
            if (ho.b_peak)
            {
                flag_set(EVF_HUMP_PEAK);
                hump_event(EVC_HUMP_PEAK, true, th.peak_mm);
            }
            if (ho.b_end)
            {
                if (ho.b_valid)
                {
                    st_hump_done(ho.height_mm);
                    hump_event(EVC_HUMP_END, true, ho.height_mm);
                }
                else
                {
                    hump_event(EVC_HUMP_END, false, 0.0f);
                }
                flag_set(EVF_HUMP_END);
            }
        }
        else if (tilt_hump_on(&th))
        {
            tilt_hump_reset(&th);
            flag_set(EVF_HUMP_END);
            drive_ms = 0u;
        }
        else
        {
            drive_ms = 0u;                          /* flat and not driving */
        }
#else
        /* Region A: humps, only while driving */
        if ((st == SYS_LINE_FOLLOWING) || (st == SYS_OBSTACLE_AVOIDANCE) || (st == SYS_MANEUVER))
        {
            ho = hump_step(&hump, &hp, pitch, ds);
            if (ho.b_start)
            {
                flag_set(EVF_HUMP_START);
                hump_event(EVC_HUMP_START, true, 0.0f);
            }
            if (ho.b_peak)
            {
                flag_set(EVF_HUMP_PEAK);
                hump_event(EVC_HUMP_PEAK, true, hump.peak_mm);
            }
            if (ho.b_end)
            {
                if (ho.b_valid)
                {
                    st_hump_done(ho.height_mm);   /* humps + 1, run max (FR22), last peak */
                    hump_event(EVC_HUMP_END, true, ho.height_mm);
                }
                else
                {
                    hump_event(EVC_HUMP_END, false, 0.0f);   /* false slope: no height */
                }
                flag_set(EVF_HUMP_END);
            }
        }
        else if (hump_state(&hump) != HUMP_FLAT)
        {
            hump_reset(&hump);
            flag_set(EVF_HUMP_END);
        }
        else
        {
            /* flat and not driving */
        }

#endif

        /* Region B: motion events (turn rate from the encoders, DR A10 fallback) */
#if HUMP_TILT_DETECT
        st_set_motion_event(mev_step(&mev, b_impact, tilt_hump_on(&th) ? HUMP_CREST : HUMP_FLAT, odo.turn_dps,
                                     odo.a_fwd_g, odo.v_mm_s, 20u));
        st_set_imu(pitch, th.lift_mm, odo.turn_dps);   /* telemetry hump height = current lift estimate */
        if (tilt_hump_on(&th))
        {
            st_set_hump_peak(th.peak_mm);
        }
#else
        st_set_motion_event(mev_step(&mev, b_impact, hump_state(&hump), odo.turn_dps,
                                     odo.a_fwd_g, odo.v_mm_s, 20u));
        st_set_imu(pitch, hump.h_mm, odo.turn_dps);
        if (hump_state(&hump) != HUMP_FLAT)
        {
            st_set_hump_peak(hump.peak_mm);
        }
#endif
    }
}
