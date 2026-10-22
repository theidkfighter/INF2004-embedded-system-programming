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

bool config_check(char *p_text, uint32_t size)
{
    jbuf_t   jb;
    uint32_t missing = 0u;

    jb_init(&jb, p_text, size);
    need(&jb, g_params.motor_mapping_confirmed > 0.5f, "MOTOR_MAPPING_CONFIRMED", &missing);
    need(&jb, g_params.enc_direction_confirmed > 0.5f, "ENC_DIRECTION_CONFIRMED", &missing);
    need(&jb, g_params.enc_a_edges_per_rev > 0.0f, "ENC_A_EDGES_PER_REV", &missing);
    need(&jb, g_params.wheel_diameter_mm > 0.0f, "WHEEL_DIAMETER_MM", &missing);
    need(&jb, g_params.track_width_mm > 0.0f, "TRACK_WIDTH_MM", &missing);
    need(&jb, g_params.ff_deadband_pct >= 0.0f, "SPEED_FF_DEADBAND_PCT", &missing);
    need(&jb, g_params.ff_mm_s_per_pct > 0.0f, "SPEED_FF_MM_S_PER_PCT", &missing);
    need(&jb, g_params.line_do_black_high >= 0.0f, "LINE_DO_BLACK_HIGH", &missing);
    need(&jb, g_params.ir_white_adc >= 0.0f, "IR_WHITE_ADC", &missing);
    need(&jb, g_params.ir_black_adc >= 0.0f, "IR_BLACK_ADC", &missing);
    need(&jb, g_params.servo_calibrated > 0.5f, "SERVO_CALIBRATED", &missing);
    need(&jb, (g_params.imu_forward_axis > 0.5f), "IMU_FORWARD_AXIS", &missing);
    need(&jb, (g_params.imu_forward_sign > 0.5f) || (g_params.imu_forward_sign < -0.5f), "IMU_FORWARD_SIGN", &missing);
    return missing == 0u;
}
