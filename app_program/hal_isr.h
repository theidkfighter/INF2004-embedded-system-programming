/**
 * @file    hal_isr.h
 * @brief   Interrupt service routines of hal_isr.c (Barr 6.5.c).
 *
 * Included only by hal_isr.c and by os_glue.c, which registers them with
 * tk_def_int(): no task can call an ISR by mistake.
 */
#ifndef HAL_ISR_H
#define HAL_ISR_H

#include <stdint.h>

void hal_gpio_isr(uint32_t intno);       /* encoders, ultrasonic echo, IMU INT1 (ignored) */
void hal_timer_isr(uint32_t intno);      /* 1 kHz: barcode ADC sampling, battery */

#endif /* HAL_ISR_H */

/*** end of file ***/
