/**
 * @file    hal_internal.h
 * @brief   Shared between the hal_*.c drivers only.
 */
#ifndef HAL_INTERNAL_H
#define HAL_INTERNAL_H

void hal_us_arm(void);        /* hal_isr.c: forget the previous echo before a new ping */

#endif /* HAL_INTERNAL_H */
