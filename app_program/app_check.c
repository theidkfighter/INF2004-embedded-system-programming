/**
 * @file    app_check.c
 * @brief   Start-up / recovery check of the calibration values (DR Section 5:
 *          INITIALISING self-test, "automated sensor re-check").
 */
#include "app_check.h"
#include "app_params.h"
#include "jfmt.h"
#include "robot_config.h"

static void need(jbuf_t *p_jb, bool b_ok, const char *p_name, uint32_t *p_missing)
{
    if (!b_ok)
    {
        if (*p_missing > 0u)
        {
            jb_raw(p_jb, ", ");
        }
        jb_raw(p_jb, p_name);
        (*p_missing)++;
    }
}

/* The 13 calibration checks, in bit order (the RECOVERY_REFUSED and
   telemetry bit numbers). One list for both functions below (Barr 6.1.i). */
#define CHECK_COUNT  (13u)

static const char * const CHECK_NAMES[CHECK_COUNT] =
{
    "MOTOR_MAPPING_CONFIRMED", "ENC_DIRECTION_CONFIRMED", "ENC_A_EDGES_PER_REV",
    "WHEEL_DIAMETER_MM", "TRACK_WIDTH_MM", "SPEED_FF_DEADBAND_PCT",
    "SPEED_FF_MM_S_PER_PCT", "LINE_DO_BLACK_HIGH", "IR_WHITE_ADC", "IR_BLACK_ADC",
    "SERVO_CALIBRATED", "IMU_FORWARD_AXIS", "IMU_FORWARD_SIGN"
};

/* Returns true when calibration item k (0 .. CHECK_COUNT - 1) is set. */
static bool check_ok(uint32_t k)
{
    bool b_ok = false;

    switch (k)
    {
        case 0u:
            b_ok = (prm_tri(PRM_MOTOR_MAPPING_CONFIRMED) > 0);
        break;

        case 1u:
            b_ok = (prm_tri(PRM_ENC_DIRECTION_CONFIRMED) > 0);
        break;

        case 2u:
            b_ok = (g_params.enc_a_edges_per_rev > 0.0f);
        break;

        case 3u:
            b_ok = (g_params.wheel_diameter_mm > 0.0f);
        break;

        case 4u:
            b_ok = (g_params.track_width_mm > 0.0f);
        break;

        case 5u:
            b_ok = (g_params.ff_deadband_pct >= 0.0f);
        break;

        case 6u:
            b_ok = (g_params.ff_mm_s_per_pct > 0.0f);
        break;

        case 7u:
            b_ok = (prm_tri(PRM_LINE_DO_BLACK_HIGH) >= 0);
        break;

        case 8u:
            b_ok = (g_params.ir_white_adc >= 0.0f);
        break;

        case 9u:
            b_ok = (g_params.ir_black_adc >= 0.0f);
        break;

        case 10u:
            b_ok = (prm_tri(PRM_SERVO_CALIBRATED) > 0);
        break;

        case 11u:
            b_ok = (prm_int(PRM_IMU_FORWARD_AXIS) > 0);
        break;

        case 12u:
            b_ok = (prm_tri(PRM_IMU_FORWARD_SIGN) != 0);
        break;

        default:
            b_ok = false;
        break;
    }
    return b_ok;
}

uint32_t config_missing_bits(void)
{
    uint32_t bits = 0u;

    for (uint32_t k = 0u; k < CHECK_COUNT; k++)
    {
        bits |= check_ok(k) ? 0u : (1u << k);
    }
    return bits;
}

bool config_check(char *p_text, uint32_t size)
{
    jbuf_t   jb;
    uint32_t missing = 0u;

    jb_init(&jb, p_text, size);
    for (uint32_t k = 0u; k < CHECK_COUNT; k++)
    {
        need(&jb, check_ok(k), CHECK_NAMES[k], &missing);
    }
    return missing == 0u;
}
