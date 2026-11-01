/**
 * @file    app_check.h
 * @brief   Calibration check: false + the list of missing values if any
 *          robot_config.h / run-time calibration value is still unset.
 */
#ifndef APP_CHECK_H
#define APP_CHECK_H

#include <stdbool.h>
#include <stdint.h>

bool config_check(char *p_text, uint32_t size);

/* The same check as a bit mask (RECOVERY_REFUSED, START_REFUSED): bit i set =
   item i missing, in this order: MOTOR_MAPPING_CONFIRMED, ENC_DIRECTION_CONFIRMED,
   ENC_A_EDGES_PER_REV, WHEEL_DIAMETER_MM, TRACK_WIDTH_MM, SPEED_FF_DEADBAND_PCT,
   SPEED_FF_MM_S_PER_PCT, LINE_DO_BLACK_HIGH, IR_WHITE_ADC, IR_BLACK_ADC,
   SERVO_CALIBRATED, IMU_FORWARD_AXIS, IMU_FORWARD_SIGN (13 bits). */
uint32_t config_missing_bits(void);

#endif /* APP_CHECK_H */
