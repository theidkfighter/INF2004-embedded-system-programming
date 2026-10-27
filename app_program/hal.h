/**
 * @file    hal.h
 * @brief   IS-02 hardware abstraction (drivers), built on the Pico SDK.
 *
 * The drivers (hal_*.c) include Pico SDK headers; the micro T-Kernel task
 * code cannot (the two define the same type names), so this interface uses
 * plain C types only. Interrupt handlers are defined here and registered
 * with tk_def_int() by os_glue.c (NFR13: drivers separated from tasks).
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* RP2040 interrupt numbers used by the robot */
#define HAL_IRQ_TIMER3   (3u)      /* TIMER alarm 3: 1 kHz barcode / VBAT sampling */
#define HAL_IRQ_GPIO     (13u)     /* IO_BANK0: encoders, IMU INT1, ultrasonic Echo */

/* ---- set-up ---- */
void     hal_init(void);                         /* pins, PWM, ADC, I2C, PIO; motors braked */
void     hal_irq_enable_sources(void);           /* per-pin edge IRQs + the 1 kHz alarm */
void     hal_gpio_isr(uint32_t intno);           /* registered with tk_def_int() */
void     hal_timer_isr(uint32_t intno);          /* registered with tk_def_int() */
void     hal_set_int1_callback(void (*p_cb)(void));   /* called from the GPIO ISR */

uint64_t hal_time_us(void);
uint32_t hal_time_ms(void);

/* ---- motors (Robo Pico driver): motor 1 = right wheel, motor 2 = left ---- */
#define HAL_MOTOR_RIGHT  (1u)
#define HAL_MOTOR_LEFT   (2u)
void     hal_motor_set(uint32_t motor, float duty);  /* -1..+1, 0 = brake (Low/Low) */
void     hal_motor_kill(void);                   /* both motors Low/Low now (E-stop) */
bool     hal_motor_killed(void);                 /* latched until hal_motor_unkill() */
void     hal_motor_unkill(void);

/* ---- encoders: wheel 0 = right, 1 = left; forward counts up ---- */
#define HAL_WHEEL_RIGHT  (0u)
#define HAL_WHEEL_LEFT   (1u)
int32_t  hal_enc_count(uint32_t wheel);          /* 4x quadrature, direction flags applied */
void     hal_enc_set_reversed(bool b_right, bool b_left);
uint32_t hal_odometer(void);                     /* |transitions| of both wheels (monotonic) */
uint32_t hal_enc_errors(void);                   /* impossible A/B transitions seen */

/* ---- IR line sensors (digital) and barcode sensor (analog) ---- */
bool     hal_ir_left_level(void);                /* raw GPIO level */
bool     hal_ir_right_level(void);
uint32_t hal_ir_adc(void);                       /* last 1 kHz sample */
void     hal_ir_adc_range(uint32_t *p_min, uint32_t *p_max);   /* min/max since the last call (diagnostics) */
void     hal_barcode_config(uint32_t thr_black, uint32_t thr_white, bool b_black_high,
                            bool b_enable);
typedef struct
{
    uint32_t odo;                                /* hal_odometer() at the edge */
    uint32_t t_ms;
    bool     b_black;
} hal_bc_edge_t;
bool     hal_barcode_pop(hal_bc_edge_t *p_edge); /* oldest edge, false if none */
uint32_t hal_barcode_overruns(void);
/* BARCODE_DETECT = 1: the interrupt only records every 1 kHz sample with the
   odometer, and bc_detect.c (TSK_LINE_BARCODE) finds the bars. Turning raw
   mode on switches the threshold edges above off, and the reverse. */
void     hal_barcode_raw(bool b_enable);
bool     hal_barcode_pop_sample(int32_t *p_enc_r, int32_t *p_enc_l, uint32_t *p_adc);   /* oldest sample: wheel counts (direction flags applied) + ADC; false if none */
uint32_t hal_vbat_mv(void);                      /* 0 when VBAT sensing is off */

/* ---- HC-SR04 (Echo timed by the GPIO interrupt) ---- */
void     hal_us_trigger(void);                   /* 10 us pulse; >= 60 ms between pings */
float    hal_us_read_cm(void);                   /* after >= 30 ms: cm, or -1 = no echo */

/* ---- servo, buzzer, NeoPixels, buttons ---- */
void     hal_servo_deg(float physical_deg);      /* 0..180 physical, 90 = straight ahead */
void     hal_servo_range(uint32_t min_us, uint32_t max_us);   /* pulse at 0 and 180 deg */
void     hal_buzzer(uint32_t freq_hz);           /* 0 = off */
void     hal_neopixel(uint8_t r, uint8_t g, uint8_t b);   /* both on-board pixels */
bool     hal_btn1_down(void);
bool     hal_btn2_down(void);

/* ---- IMU (LSM303DLHC accelerometer on I2C1) ---- */
bool     hal_imu_init(void);                     /* +-4 g, 100 Hz, INT1 = 2 g high-pass, latched */
bool     hal_imu_accel_g(float *p_x, float *p_y, float *p_z);  /* false = I2C timeout (2 ms) */
bool     hal_imu_int1_src(uint8_t *p_src);       /* reads INT1_SRC_A, clears the latch */
bool     hal_imu_int1_level(void);
void     hal_i2c_recover(void);                  /* 9 SCL clocks, then re-initialise I2C1 */

#endif /* HAL_H */
