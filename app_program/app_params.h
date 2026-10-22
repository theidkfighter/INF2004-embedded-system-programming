/**
 * @file    app_params.h
 * @brief   Run-time parameters (NFR14: calibration without recompiling,
 *          NFR15: PID gains through a configuration interface).
 *
 * Start-up values come from robot_config.h. Every value can be changed over
 * MQTT without recompiling:
 *   SET:LINE_STEER=0.35      -> ack {"cmd":"SET:...","result":"ok", ...}
 *   GET:PARAMS               -> ack with every parameter
 *   is02/config (retained)   -> "NAME=VALUE" lines, applied at every connect,
 *                               so the laptop's broker keeps the robot's
 *                               calibration (laptop_tools/robot_params.py)
 * Nothing is written to the Pico's flash: flash writes are outside the
 * template's qualified boundary.
 */
#ifndef APP_PARAMS_H
#define APP_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* confirmations and directions from the part tests (1 = yes / reversed) */
    float motor_mapping_confirmed;
    float enc_r_reversed;
    float enc_l_reversed;
    float enc_direction_confirmed;
    float line_do_black_high;
    float servo_min_us;
    float servo_max_us;
    float servo_calibrated;
    float servo_low_angle_is_right;
    float imu_forward_axis;
    float imu_forward_sign;
    float robot_half_width_mm;
    /* calibration (CAL_UNSET = -1 until measured) */
    float enc_a_edges_per_rev;
    float wheel_diameter_mm;
    float track_width_mm;
    float ff_deadband_pct;
    float ff_mm_s_per_pct;
    float ir_white_adc;
    float ir_black_adc;
    /* speed control */
    float speed_kp;
    float speed_ki;
    /* line following */
    float line_base_mm_s;
    float line_steer;
    float line_steer_ramp;
    float line_hold_steer;
    float line_hold_mm;
    float line_lost_mm;
    float barcode_at_junction;
    /* motion primitives / obstacles */
    float turn_mm_s;
    float bypass_mm_s;
    float obstacle_stop_cm;
    float obst_margin_mm;
    float obst_depth_mm;
    float ir_to_axle_mm;
} app_params_t;

extern app_params_t g_params;

void  params_init(void);

/* "NAME=VALUE" (a leading "SET:" is accepted). Returns 0 = ok, -1 = unknown
   name, -2 = bad number, -3 = out of range. */
int32_t params_set(const char *p_text);

/* Several "NAME=VALUE" separated by new lines, ';' or ','. Counts the good
   and bad ones; the names of bad ones are listed in p_bad. */
void    params_set_many(const char *p_text, uint32_t *p_ok, uint32_t *p_bad,
                        char *p_bad_names, uint32_t size);

/* Writes {"NAME":value,...} for parameters first..first+count-1 into p_buf.
   Returns the length. params_count() = how many there are. */
uint32_t params_to_json(char *p_buf, uint32_t size, uint32_t first, uint32_t count);
uint32_t params_count(void);

/* Derived values (0 if the calibration is missing) */
float params_mm_per_count(void);     /* 4x quadrature count */

#endif /* APP_PARAMS_H */
