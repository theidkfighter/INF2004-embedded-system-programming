/**
 * @file    tsk_imu.c
 * @brief   TSK_TERRAIN_IMU (DR Section 3 / 6.3, 20 ms): pitch, hump
 *          detection and height, lift / overturn stop, motion events.
 *
 * Every 20 ms, under MTX_I2C_BUS: read the accelerometer, pitch =
 * atan2(a_fwd_axis - a_fwd, sqrt(other^2 + z^2)) - theta0, EMA filtered.
 * Region A (hump profiler, imu_logic.c) sets EVF_HUMP_START / PEAK / END and
 * updates the run maximum (FR21, FR22). Region B classifies motion events
 * (FR24). Lift / overturn (16 Nov requirement change, replaces the IMU
 * collision detection): while driving, lift_step() (imu_logic.c).
 * Lifted (rising at 150 mm/s or more, or tipped 30 deg or more): lift_stop_put(true) -> TSK_MOTION_CTRL holds the
 * motors off at once and TSK_NAV_PLANNER ends the run normally
 * (MISSION_COMPLETE). Overturned (tilted 60 deg or more): EVF_OVERTURNED ->
 * TSK_SAFETY_MON E-stop "overturned (IMU)". Lifting ends the demo run.
 * I2C fault: bus recovery (9 SCL clocks + re-init). After 3 failed recoveries
 * (or if the IMU did not answer at start-up) the robot keeps driving without
 * hump / lift detection (FAULT_IMU_DEAD in telemetry, NeoPixel blinks
 * blue/yellow in IDLE) and the IMU is re-tried every 2 s. 21 Oct change:
 * the DR sends this case to ERROR; degraded operation was chosen so a loose
 * IMU wire does not stop the whole run.
 *
 * theta0 (mounting offset) is averaged over 50 samples whenever the robot
 * is in IDLE and standing still, so it is fresh at every start.
 */
#include <tk/tkernel.h>

#include "app_tasks.h"

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
#define TENTHS_PER_MM    (10.0f)
#define HUMP_LEN_MAX_D_MM (15000u)    /* 1500.0 mm in 0.1 mm: the U14 field's ceiling */
#define HUMP_LEN_MAX_MM  (1500.0f)
#define UM_PER_MM        (1000.0f)
#define IMU_RETRY_LOOPS  (100u)       /* every 2 s at the 20 ms loop */
#define STILL_MM_S       (5.0f)       /* |v| below this counts as standing still */
#define ROUND_HALF       (0.5f)

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
    float c = (mm * TENTHS_PER_MM) + ROUND_HALF;

    return bp_is_nan(mm) ? 0u : ((c <= 0.0f) ? 0u : ((c >= (float)HUMP_LEN_MAX_D_MM) ? HUMP_LEN_MAX_D_MM : (uint32_t)c));
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

void tsk_imu_task(INT stacd, void *exinf)
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
    uint32_t      fails = 0u;
    uint32_t      recoveries = 0u;
    uint32_t      retry_n = 0u;
#if LIFT_DETECT
    lift_params_t lp;
    lift_ctx_t    lift;
#endif
#if HUMP_TILT_DETECT
    /* tilt hump detector (robot_config.h HUMP_TILT_DETECT, imu_logic.h) */
    hump_params_t tp;
    tilt_hump_t   th;
    float         gb[3] = { 0.0f, 0.0f, 1.0f };    /* baseline: slow-moving driving posture */
    float         gf[3] = { 0.0f, 0.0f, 0.0f };    /* low-passed gravity vector */
    bool          b_gf = false;
    uint32_t      drive_ms = 0u;                   /* time since driving started */
    float         turn_f = 0.0f;                   /* low-passed turn rate, deg/s (hump gate) */
    float         straight_mm = 0.0f;              /* straight line following so far (hump gate) */
    sys_state_t   st_prev = SYS_IDLE;
#endif

    (void)stacd;
    (void)exinf;
    hp.start_deg     = HUMP_START_DEG;
    hp.level_deg     = HUMP_LEVEL_DEG;
    hp.min_height_mm = HUMP_MIN_HEIGHT_MM;
    hp.start_samples = HUMP_START_SAMPLES;
    hp.end_samples   = HUMP_END_SAMPLES;
    hp.max_len_mm    = HUMP_LEN_MAX_MM;
    hump_reset(&hump);
    mev_reset(&mev);
#if LIFT_DETECT
    lp.v_lift_mm_s   = LIFT_V_MM_S;
    lp.v_leak        = LIFT_V_LEAK;
    lp.up_clip_g     = LIFT_UP_CLIP_G;
    lp.tilt_deg      = LIFT_TILT_DEG;
    lp.over_deg      = LIFT_OVER_DEG;
    lp.alpha         = LIFT_ALPHA;
    lp.tilt_samples  = (uint8_t)LIFT_TILT_SAMPLES;
    lp.over_samples  = (uint8_t)LIFT_OVER_SAMPLES;
    lp.blank_samples = (uint8_t)LIFT_BLANK_SAMPLES;
    lift_reset(&lift);
    lift_stop_put(false);
#endif
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
        bool        b_lifted = false;
        sys_state_t st;
        hump_out_t  ho;

        PERF_CYCLE_END(PERF_TSK_IMU, PERF_LIMIT_20MS_US);      /* end of the previous cycle (also after "continue") */
        (void)tk_slp_tsk(TMO_FEVR);          /* released every 20 ms by the cyclic tick */
        g_hb[HB_IMU]++;
        st = sys_state();
        odometry_get(&odo);
        ds       = (float)(odo.odo_um - odo_prev) / UM_PER_MM;
        odo_prev = odo.odo_um;

        if (fault_has(FAULT_IMU_DEAD))       /* the one record of the IMU state (CR1) */
        {
            retry_n++;
            if (retry_n >= IMU_RETRY_LOOPS)
            {
                retry_n = 0u;
                if (imu_recover())
                {
                    recoveries = 0u;
                    fault_clear(FAULT_IMU_DEAD | FAULT_IMU_I2C);
                    {
                        evt_b_t b;                    /* chip E2, accel address U7 */

                        evt_begin(&b, EVC_IMU_OK);
                        evt_u(&b, (uint32_t)hal_imu_chip(), 2u);
                        evt_u(&b, hal_imu_addr(), 7u);
                        (void)evt_post(&b);
                    }
                    log_line("[imu] GY-511 answering: hump and lift detection on");
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
                        fault_set(FAULT_IMU_DEAD);
                        (void)evt_post0(EVC_FAULT_IMU);       /* "3 failed recoveries: driving without the IMU" */
                        log_line("[imu] GY-511 lost: driving without hump/lift detection");
                    }
                }
            }
            continue;
        }
        fails = 0u;

#if LIFT_DETECT
        /* lift / overturn: only while driving (motors are off otherwise) */
        if ((st == SYS_LINE_FOLLOWING) || (st == SYS_OBSTACLE_AVOIDANCE) || (st == SYS_MANEUVER))
        {
            lift_cause_t cause = lift_step(&lift, &lp, ax, ay, az);

            if (cause != LIFT_NONE)
            {
                lift_stop_put(true);                      /* motion: motors off now; nav: end the run */
                (void)evt_post1(EVC_LIFTED, (cause == LIFT_OVER) ? 1u : 0u, 1u);   /* overturned F1 */
                if (cause == LIFT_OVER)
                {
                    flag_set(EVF_OVERTURNED);             /* overturned: E-stop */
                    (void)tk_wup_tsk(g_tsk_safety);
                    log_line("[imu] overturned: E-stop");
                }
                else
                {
                    log_line("[imu] lifted: motors off, run ended");
                }
                hump_reset(&hump);
#if HUMP_TILT_DETECT
                tilt_hump_reset(&th);
#endif
            }
            b_lifted = lift_latched(&lift);
        }
        else
        {
            lift_reset(&lift);                            /* new reference at the next start */
            lift_stop_put(false);
        }
#endif

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
        if ((st == SYS_IDLE) && ((odo.v_mm_s < STILL_MM_S) && (odo.v_mm_s > -STILL_MM_S)))
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
            /* 19 Nov: only straight line following may start a hump; turning or
               pivoting on the foam mat tilts the car like a hump does (17:25-17:31
               runs: 15 "humps" on a track with one) */
            {
                float a_turn;

                turn_f = ema(turn_f, odo.turn_dps, HUMP_TURN_ALPHA);
                a_turn = (turn_f < 0.0f) ? -turn_f : turn_f;
                if ((st == SYS_LINE_FOLLOWING) && (st_prev == SYS_IDLE))
                {
                    straight_mm = HUMP_STRAIGHT_RUNIN_MM;   /* a run starts straight on the line */
                    turn_f      = 0.0f;
                }
                else if ((st == SYS_LINE_FOLLOWING) && (a_turn <= HUMP_STRAIGHT_DPS) && (ds > 0.0f))
                {
                    straight_mm += ds;
                }
                else
                {
                    straight_mm = 0.0f;
                }
                if (tilt_hump_on(&th) && (st != SYS_LINE_FOLLOWING))
                {
                    tilt_hump_reset(&th);             /* not a hump: dropped, not counted */
                    flag_set(EVF_HUMP_END);           /* lifts the hump speed limit */
                    hump_event(EVC_HUMP_END, false, 0.0f);
                    straight_mm = 0.0f;
                }
                else if (!tilt_hump_on(&th) && (straight_mm < HUMP_STRAIGHT_RUNIN_MM))
                {
                    t = 0.0f;                         /* not armed: cannot start */
                }
                else
                {
                    /* armed, or a hump in progress while still straight enough */
                }
            }
            st_prev = st;
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
        if ((st != SYS_LINE_FOLLOWING) && (st != SYS_OBSTACLE_AVOIDANCE) && (st != SYS_MANEUVER))
        {
            st_prev = st;                           /* IDLE before a run: the hump gate starts armed */
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
        st_set_motion_event(mev_step(&mev, b_lifted, tilt_hump_on(&th) ? HUMP_CREST : HUMP_FLAT, odo.turn_dps,
                                     odo.a_fwd_g, odo.v_mm_s, 20u));
        st_set_imu(pitch, th.lift_mm, odo.turn_dps);   /* telemetry hump height = current lift estimate */
        if (tilt_hump_on(&th))
        {
            st_set_hump_peak(th.peak_mm);
        }
#else
        st_set_motion_event(mev_step(&mev, b_lifted, hump_state(&hump), odo.turn_dps,
                                     odo.a_fwd_g, odo.v_mm_s, 20u));
        st_set_imu(pitch, hump.h_mm, odo.turn_dps);
        if (hump_state(&hump) != HUMP_FLAT)
        {
            st_set_hump_peak(hump.peak_mm);
        }
#endif
    }
}
