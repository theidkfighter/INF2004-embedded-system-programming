/**
 * @file    app_status.c
 * @brief   The shared Status_Snapshot (packed, see status.h) and the state /
 *          event names.
 */
#include "app_types.h"
#include "status.h"
#include "bitpack.h"
#include "crit.h"

robot_status_t g_status;

static const char * const STATE_NAMES[SYS_STATE_COUNT] =
{
    "INITIALISING", "IDLE", "LINE_FOLLOWING", "MANEUVER", "OBSTACLE_AVOIDANCE",
    "MISSION_COMPLETE", "EMERGENCY_STOP", "ERROR"
};

static const char * const MEV_NAMES[MEV_COUNT] =
{
    "STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING", "TURNING",
    "CLIMBING_HUMP", "DESCENDING_HUMP", "SUDDEN_IMPACT"
};

const char *sys_state_name(sys_state_t state)
{
    return ((uint32_t)state < (uint32_t)SYS_STATE_COUNT) ? STATE_NAMES[state] : "UNKNOWN";
}

const char *motion_event_name(motion_event_t ev)
{
    return ((uint32_t)ev < (uint32_t)MEV_COUNT) ? MEV_NAMES[ev] : "UNKNOWN";
}

/* ------------------------------------------------------------------------ */
/* quantizer specs (scale, lo_code, hi_code, nan_code), status.h             */
/* ------------------------------------------------------------------------ */

static const bp_qspec_t Q_SPD   = { 10,    -10000, 10000, -10000 };   /* S15 0.1 mm/s */
static const bp_qspec_t Q_DUTY  = { 1000,  -1000,  1000,  -1000 };    /* S11 0.1 %, from -1..+1 */
static const bp_qspec_t Q_SCALE = { 100,   0,      100,   0 };        /* U7 0.01 */
static const bp_qspec_t Q_DIST  = { 10,    0,      1000000, 0 };      /* U20 0.1 mm */
static const bp_qspec_t Q_PITCH = { 10,    -900,   900,   -900 };     /* S11 0.1 deg */
static const bp_qspec_t Q_HUMPH = { 10,    -15000, 15000, -15000 };   /* S15 0.1 mm */
static const bp_qspec_t Q_HUMPU = { 10,    0,      15000, 0 };        /* U14 0.1 mm */
static const bp_qspec_t Q_TURN  = { 10,    -7200,  7200,  -7200 };    /* S14 0.1 deg/s */
static const bp_qspec_t Q_US    = { 10,    0,      4310,  (int32_t)ST_US_NONE };   /* U13 0.1 cm */

/* Quantizes x; counts a clamp or NaN in the Saturation_Counter (outside any
   Critical_Section, crit.h). */
static uint32_t qz(float x, const bp_qspec_t *p_q, uint32_t *p_sat)
{
    bool    b_sat;
    int32_t c = bp_quant(x, p_q, &b_sat);

    *p_sat += b_sat ? 1u : 0u;
    return (uint32_t)c;
}

static uint32_t q_us(float cm, uint32_t *p_sat)
{
    if ((cm != cm) || (cm < 0.0f))
    {
        return ST_US_NONE;                       /* no echo: the sentinel, not a saturation */
    }
    return qz(cm, &Q_US, p_sat);
}

/* One field of one unit, read-modify-write in a dispatch-disabled section. */
static void put1(volatile uint32_t *p_u, uint32_t sh, uint32_t w, uint32_t code)
{
    crit_dsp_enter();
    *p_u = BP_PUT(*p_u, sh, w, code);
    crit_dsp_exit();
}

static float deq_s(uint32_t unit, uint32_t sh, uint32_t w, int32_t scale)
{
    return bp_dequant(bp_sext(BP_GET(unit, sh, w), w), scale);
}

static float deq_u(uint32_t unit, uint32_t sh, uint32_t w, int32_t scale)
{
    return bp_dequant((int32_t)BP_GET(unit, sh, w), scale);
}

/* ------------------------------------------------------------------------ */
/* writers                                                                   */
/* ------------------------------------------------------------------------ */

void st_set_state(sys_state_t state)
{
    put1(&g_status.sys, ST_SH_STATE, ST_W_STATE, (uint32_t)state);
}

void st_fault_set(uint32_t bits)
{
    crit_dsp_enter();
    g_status.sys |= (bits & BP_MASK(ST_W_FAULTS)) << ST_SH_FAULTS;
    crit_dsp_exit();
}

void st_fault_clear(uint32_t bits)
{
    crit_dsp_enter();
    g_status.sys &= ~((bits & BP_MASK(ST_W_FAULTS)) << ST_SH_FAULTS);
    crit_dsp_exit();
}

void st_set_stop(uint32_t stop_reason)
{
    put1(&g_status.sys, ST_SH_STOP, ST_W_STOP, stop_reason);
}

void st_set_motion_event(motion_event_t ev)
{
    put1(&g_status.sys, ST_SH_MEV, ST_W_MEV, (uint32_t)ev);
}

static uint32_t enc24(int32_t c)
{
    c = (c > 8388607) ? 8388607 : ((c < -8388607) ? -8388607 : c);
    return bp_tc(c, 24u);
}

void st_set_motion(float spd_l, float spd_r, float duty_l, float duty_r,
                   int32_t enc_l, int32_t enc_r, float dist_mm, float speed_scale)
{
    uint32_t n = 0u;
    uint32_t a = BP_PUT(BP_PUT(0u, 0u, 15u, qz(spd_l, &Q_SPD, &n)), 15u, 15u, qz(spd_r, &Q_SPD, &n));
    uint32_t b = BP_PUT(BP_PUT(BP_PUT(0u, 0u, 11u, qz(duty_l, &Q_DUTY, &n)), 11u, 11u,
                               qz(duty_r, &Q_DUTY, &n)), 22u, 7u, qz(speed_scale, &Q_SCALE, &n));
    uint32_t e = qz(dist_mm, &Q_DIST, &n);

    /* motion units have one writer (TSK_MOTION_CTRL) and are whole-word
       stores, so each store is atomic; the section only keeps the five
       units of one cycle together for a reader */
    crit_dsp_enter();
    g_status.mot_a = a;
    g_status.mot_b = b;
    g_status.mot_c = enc24(enc_l);
    g_status.mot_d = enc24(enc_r);
    g_status.mot_e = e;
    crit_dsp_exit();
    sat_note(n);
}

void st_set_line(bool b_l, bool b_r, uint32_t ir_adc, uint32_t line_state)
{
    crit_dsp_enter();
    g_status.line_a = BP_PUT(BP_PUT(BP_PUT(BP_PUT(g_status.line_a, 0u, 1u, b_l ? 1u : 0u),
                                           1u, 1u, b_r ? 1u : 0u),
                                    2u, 12u, (ir_adc > 4095u) ? 4095u : ir_adc),
                             14u, 3u, line_state);
    crit_dsp_exit();
}

void st_set_bc_levels(uint32_t thr_black, uint32_t thr_white, bool b_enabled)
{
    thr_black = (thr_black > 4095u) ? 4095u : thr_black;
    thr_white = (thr_white > 4095u) ? 4095u : thr_white;
    crit_dsp_enter();
    g_status.line_b = BP_PUT(BP_PUT(0u, 0u, 12u, thr_black), 12u, 12u, thr_white);
    g_status.line_a = BP_PUT(g_status.line_a, 23u, 1u, b_enabled ? 1u : 0u);
    crit_dsp_exit();
}

void st_barcode_read(char code)
{
    uint32_t c6 = ((code > ' ') && (code <= '_')) ? (uint32_t)(code - ' ') : 0u;

    crit_dsp_enter();
    g_status.line_a = BP_PUT(BP_PUT(g_status.line_a, 17u, 6u, c6), 24u, 7u,
                             bp_sat_inc(BP_GET(g_status.line_a, 24u, 7u), 127u));
    crit_dsp_exit();
}

void st_set_imu(float pitch_deg, float hump_mm, float turn_dps)
{
    uint32_t n = 0u;
    uint32_t p = qz(pitch_deg, &Q_PITCH, &n);
    uint32_t h = qz(hump_mm, &Q_HUMPH, &n);
    uint32_t t = qz(turn_dps, &Q_TURN, &n);

    crit_dsp_enter();
    g_status.imu_a = BP_PUT(BP_PUT(0u, 0u, 11u, p), 11u, 15u, h);
    g_status.imu_c = BP_PUT(g_status.imu_c, 0u, 14u, t);
    crit_dsp_exit();
    sat_note(n);
}

void st_set_hump_peak(float peak_mm)
{
    uint32_t n = 0u;
    uint32_t c = qz(peak_mm, &Q_HUMPU, &n);

    put1(&g_status.imu_b, 0u, 14u, c);
    sat_note(n);
}

void st_hump_done(float height_mm)
{
    uint32_t n = 0u;
    uint32_t c = qz(height_mm, &Q_HUMPU, &n);

    crit_dsp_enter();
    {
        uint32_t mx = BP_GET(g_status.imu_b, 14u, 14u);

        g_status.imu_b = BP_PUT(BP_PUT(g_status.imu_b, 0u, 14u, c), 14u, 14u, (c > mx) ? c : mx);
        g_status.imu_c = BP_PUT(g_status.imu_c, 14u, 6u, bp_sat_inc(BP_GET(g_status.imu_c, 14u, 6u), 63u));
    }
    crit_dsp_exit();
    sat_note(n);
}

void st_set_obst_cm(float cm)
{
    uint32_t n = 0u;
    uint32_t c = q_us(cm, &n);

    put1(&g_status.obs_a, 0u, 13u, c);
    sat_note(n);
}

void st_set_scan(uint32_t i, float cm)
{
    uint32_t n = 0u;
    uint32_t c = q_us(cm, &n);

    switch (i)
    {
        case 0u: put1(&g_status.obs_a, 15u, 13u, c); break;
        case 1u: put1(&g_status.obs_b, 0u, 13u, c);  break;
        case 2u: put1(&g_status.obs_b, 13u, 13u, c); break;
        case 3u: put1(&g_status.obs_c, 0u, 13u, c);  break;
        case 4u: put1(&g_status.obs_c, 13u, 13u, c); break;
        default: break;
    }
    sat_note(n);
}

void st_set_scan_side(char side)
{
    put1(&g_status.obs_a, 13u, 2u, (side == 'L') ? 1u : ((side == 'R') ? 2u : 0u));
}

void st_obstacle_done(void)
{
    crit_dsp_enter();
    g_status.obs_c = BP_PUT(g_status.obs_c, 26u, 6u, bp_sat_inc(BP_GET(g_status.obs_c, 26u, 6u), 63u));
    crit_dsp_exit();
}

void st_set_vbat(uint32_t mv)
{
    put1(&g_status.misc, 0u, 13u, (mv > 8191u) ? 8191u : mv);
}

void st_run_reset(void)
{
    crit_dsp_enter();
    g_status.line_a = BP_PUT(BP_PUT(g_status.line_a, 17u, 6u, 0u), 24u, 7u, 0u);
    g_status.imu_b  = BP_PUT(g_status.imu_b, 14u, 14u, 0u);
    g_status.imu_c  = BP_PUT(g_status.imu_c, 14u, 6u, 0u);
    g_status.obs_c  = BP_PUT(g_status.obs_c, 26u, 6u, 0u);
    crit_dsp_exit();
}

/* ------------------------------------------------------------------------ */
/* readers: one 32-bit load per unit                                         */
/* ------------------------------------------------------------------------ */

sys_state_t    st_state(void)        { return (sys_state_t)BP_GET(g_status.sys, ST_SH_STATE, ST_W_STATE); }
uint32_t       st_faults(void)       { return BP_GET(g_status.sys, ST_SH_FAULTS, ST_W_FAULTS); }
uint32_t       st_stop(void)         { return BP_GET(g_status.sys, ST_SH_STOP, ST_W_STOP); }
motion_event_t st_motion_event(void) { return (motion_event_t)BP_GET(g_status.sys, ST_SH_MEV, ST_W_MEV); }
float          st_dist_mm(void)      { return deq_u(g_status.mot_e, 0u, 20u, 10); }
bool           st_line_l(void)       { return BP_GET(g_status.line_a, 0u, 1u) != 0u; }
bool           st_line_r(void)       { return BP_GET(g_status.line_a, 1u, 1u) != 0u; }
uint32_t       st_ir_adc(void)       { return BP_GET(g_status.line_a, 2u, 12u); }
uint32_t       st_bc_thr_black(void) { return BP_GET(g_status.line_b, 0u, 12u); }
uint32_t       st_bc_thr_white(void) { return BP_GET(g_status.line_b, 12u, 12u); }
bool           st_bc_enabled(void)   { return BP_GET(g_status.line_a, 23u, 1u) != 0u; }
uint32_t       st_barcodes_read(void){ return BP_GET(g_status.line_a, 24u, 7u); }
float          st_pitch_deg(void)    { return deq_s(g_status.imu_a, 0u, 11u, 10); }
float          st_hump_max_mm(void)  { return deq_u(g_status.imu_b, 14u, 14u, 10); }
uint32_t       st_humps(void)        { return BP_GET(g_status.imu_c, 14u, 6u); }
uint32_t       st_obstacles(void)    { return BP_GET(g_status.obs_c, 26u, 6u); }
uint32_t       st_vbat(void)         { return BP_GET(g_status.misc, 0u, 13u); }

char st_barcode(void)
{
    uint32_t c = BP_GET(g_status.line_a, 17u, 6u);

    return (c == 0u) ? '\0' : (char)(c + ' ');
}

static float us_value(uint32_t code)
{
    return (code == ST_US_NONE) ? -1.0f : bp_dequant((int32_t)code, 10);
}

float st_obst_cm(void)
{
    return us_value(BP_GET(g_status.obs_a, 0u, 13u));
}

float st_scan_cm(uint32_t i)
{
    switch (i)
    {
        case 0u: return us_value(BP_GET(g_status.obs_a, 15u, 13u));
        case 1u: return us_value(BP_GET(g_status.obs_b, 0u, 13u));
        case 2u: return us_value(BP_GET(g_status.obs_b, 13u, 13u));
        case 3u: return us_value(BP_GET(g_status.obs_c, 0u, 13u));
        case 4u: return us_value(BP_GET(g_status.obs_c, 13u, 13u));
        default: return -1.0f;
    }
}

char st_scan_side(void)
{
    uint32_t s = BP_GET(g_status.obs_a, 13u, 2u);

    return (s == 1u) ? 'L' : ((s == 2u) ? 'R' : '\0');
}
