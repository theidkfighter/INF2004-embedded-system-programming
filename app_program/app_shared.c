/**
 * @file    app_shared.c
 * @brief   Packed Task_Shared_Data outside the Status_Snapshot (see shared.h).
 *
 * The units are not static: the host tests watch them (Property 18). Tasks
 * reach them only through shared.h.
 */
#include "shared.h"
#include "bitpack.h"
#include "crit.h"

#define NF_ESTOP_PENDING  (1u << 0)
#define NF_ESTOP_ACK_DUE  (1u << 1)
#define NF_CONFIG_DUE     (1u << 2)

#define DONE_W_RESULT     (2u)
#define PWM_OFF_MAX       (65535u)

_Static_assert(BUZZ_CLIP <= BP_MASK(3u), "g_buzz.pattern too narrow");
_Static_assert((uint32_t)MOTION_TIMEOUT <= BP_MASK(DONE_W_RESULT), "g_done.done_result too narrow");

volatile uint8_t  g_buzz;
volatile uint8_t  g_net_flags;
volatile uint32_t g_estop_rx_us;
volatile uint16_t g_estop_pwm_off_us;
volatile uint32_t g_done;

/* ---- buzzer --------------------------------------------------------------- */

bool buzz_put(uint32_t pattern)
{
    if (pattern > BUZZ_CLIP)
    {
        return false;
    }
    crit_dsp_enter();
    g_buzz = (uint8_t)pattern;
    crit_dsp_exit();
    return true;
}

uint32_t buzzer_pattern_take(void)
{
    uint32_t p;

    crit_dsp_enter();
    p = g_buzz;
    g_buzz = (uint8_t)BUZZ_NONE;
    crit_dsp_exit();
    return p;
}

/* ---- E-stop and config flags ---------------------------------------------- */

void estop_rx_put(uint32_t rx_us)
{
    crit_dsp_enter();
    g_estop_rx_us = rx_us;
    g_net_flags   = (uint8_t)(g_net_flags | NF_ESTOP_PENDING | NF_ESTOP_ACK_DUE);
    crit_dsp_exit();
}

bool estop_rx_take(uint32_t *p_rx_us)
{
    bool b_pending;

    crit_dsp_enter();
    b_pending = (g_net_flags & NF_ESTOP_PENDING) != 0u;
    if (b_pending)
    {
        *p_rx_us    = g_estop_rx_us;
        g_net_flags = (uint8_t)(g_net_flags & ~NF_ESTOP_PENDING);
    }
    crit_dsp_exit();
    return b_pending;
}

void estop_ack_request(void)
{
    crit_dsp_enter();
    g_net_flags = (uint8_t)(g_net_flags | NF_ESTOP_ACK_DUE);
    crit_dsp_exit();
}

bool estop_ack_take(void)
{
    bool b_due;

    crit_dsp_enter();
    b_due = (g_net_flags & (NF_ESTOP_ACK_DUE | NF_ESTOP_PENDING)) == NF_ESTOP_ACK_DUE;
    if (b_due)
    {
        g_net_flags = (uint8_t)(g_net_flags & ~NF_ESTOP_ACK_DUE);
    }
    crit_dsp_exit();
    return b_due;
}

void estop_pwm_off_put(uint32_t us)
{
    g_estop_pwm_off_us = (uint16_t)((us > PWM_OFF_MAX) ? PWM_OFF_MAX : us);
}

uint16_t estop_pwm_off(void)
{
    return g_estop_pwm_off_us;
}

void config_due_set(void)
{
    crit_dsp_enter();
    g_net_flags = (uint8_t)(g_net_flags | NF_CONFIG_DUE);
    crit_dsp_exit();
}

void config_due_clear(void)
{
    crit_dsp_enter();
    g_net_flags = (uint8_t)(g_net_flags & ~NF_CONFIG_DUE);
    crit_dsp_exit();
}

bool config_due(void)
{
    return (g_net_flags & NF_CONFIG_DUE) != 0u;
}

/* ---- motion done word ------------------------------------------------------ */

void motion_done_put(uint16_t id, motion_result_t result)
{
    uint32_t w = BP_PUT((uint32_t)id, 16u, DONE_W_RESULT, (uint32_t)result);

    crit_dsp_enter();
    g_done = w;
    crit_dsp_exit();
}

uint32_t motion_done_get(void)
{
    return g_done;
}

/* ---- odometry -------------------------------------------------------------- */

/* Packed odometry record (shared.h): 5 words, fields at these bit positions. */
#define ODO_WORDS   (5u)
#define ODO_P_UM    (0u)      /* U32 */
#define ODO_P_DIST  (32u)     /* S28 @ 0.01 mm */
#define ODO_P_HEAD  (60u)     /* S25 @ 0.01 deg */
#define ODO_P_V     (85u)     /* S18 @ 0.01 mm/s */
#define ODO_P_A     (103u)    /* S17 @ 1e-4 g */
#define ODO_P_TURN  (120u)    /* S18 @ 0.01 deg/s */
_Static_assert((ODO_P_TURN + 18u) <= (ODO_WORDS * 32u), "odometry record does not fit 5 words");

static const bp_qspec_t Q_ODO_DIST = { 100,   -134217727, 134217727, 0 };
static const bp_qspec_t Q_ODO_HEAD = { 100,   -10000000,  10000000,  0 };
static const bp_qspec_t Q_ODO_V    = { 100,   -100000,    100000,    0 };
static const bp_qspec_t Q_ODO_A    = { 10000, -40000,     40000,     0 };
static const bp_qspec_t Q_ODO_TURN = { 100,   -72000,     72000,     0 };

uint32_t g_odo_rec[ODO_WORDS];        /* shared: copied whole inside one section */

void odometry_get(odometry_t *p_out)
{
    uint32_t w[ODO_WORDS];

    crit_dsp_enter();
    for (uint32_t i = 0u; i < ODO_WORDS; i++)
    {
        w[i] = g_odo_rec[i];
    }
    crit_dsp_exit();
    p_out->odo_um      = bp_wget(w, ODO_P_UM, 32u);
    p_out->dist_mm     = bp_dequant(bp_sext(bp_wget(w, ODO_P_DIST, 28u), 28u), 100);
    p_out->heading_deg = bp_dequant(bp_sext(bp_wget(w, ODO_P_HEAD, 25u), 25u), 100);
    p_out->v_mm_s      = bp_dequant(bp_sext(bp_wget(w, ODO_P_V, 18u), 18u), 100);
    p_out->a_fwd_g     = bp_dequant(bp_sext(bp_wget(w, ODO_P_A, 17u), 17u), 10000);
    p_out->turn_dps    = bp_dequant(bp_sext(bp_wget(w, ODO_P_TURN, 18u), 18u), 100);
}

void odometry_put(const odometry_t *p_in)
{
    uint32_t w[ODO_WORDS] = { 0u, 0u, 0u, 0u, 0u };
    bool     b_s;
    uint32_t n = 0u;

    /* quantize and pack outside the section: only the copy is inside */
    bp_wput(w, ODO_P_UM, 32u, p_in->odo_um);
    bp_wput(w, ODO_P_DIST, 28u, bp_tc(bp_quant(p_in->dist_mm, &Q_ODO_DIST, &b_s), 28u));
    n += b_s ? 1u : 0u;
    bp_wput(w, ODO_P_HEAD, 25u, bp_tc(bp_quant(p_in->heading_deg, &Q_ODO_HEAD, &b_s), 25u));
    n += b_s ? 1u : 0u;
    bp_wput(w, ODO_P_V, 18u, bp_tc(bp_quant(p_in->v_mm_s, &Q_ODO_V, &b_s), 18u));
    n += b_s ? 1u : 0u;
    bp_wput(w, ODO_P_A, 17u, bp_tc(bp_quant(p_in->a_fwd_g, &Q_ODO_A, &b_s), 17u));
    n += b_s ? 1u : 0u;
    bp_wput(w, ODO_P_TURN, 18u, bp_tc(bp_quant(p_in->turn_dps, &Q_ODO_TURN, &b_s), 18u));
    n += b_s ? 1u : 0u;
    crit_dsp_enter();
    for (uint32_t i = 0u; i < ODO_WORDS; i++)
    {
        g_odo_rec[i] = w[i];
    }
    crit_dsp_exit();
    sat_note(n);
}
