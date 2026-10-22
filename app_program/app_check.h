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

#endif /* APP_CHECK_H */
