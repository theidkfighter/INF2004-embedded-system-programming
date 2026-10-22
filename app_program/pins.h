/**
 * @file    pins.h
 * @brief   IS-02 Robo-Wheeler pin map (RoboWheeler_Connections_Spec.md, 21 Oct).
 *
 * Plain numbers only, so both the Pico SDK side (hal_*.c) and the
 * micro T-Kernel side (tsk_*.c) can include it. Same pins as
 * part_tests/include/pins.h and full_tests/include/pins.h.
 */
#ifndef PINS_H
#define PINS_H

/* ---- Motors: Robo Pico on-board driver (A/B per motor) ---- */
#define PIN_M1A              (8u)     /* motor 1 = RIGHT wheel, PWM slice 4A */
#define PIN_M1B              (9u)     /* PWM slice 4B */
#define PIN_M2A              (10u)    /* motor 2 = LEFT wheel, PWM slice 5A */
#define PIN_M2B              (11u)    /* PWM slice 5B */
#define MOTOR_PWM_FREQ_HZ    (20000u)

/* ---- Wheel encoders (quadrature, 4x decoded) ---- */
#define PIN_ENC_R_A          (0u)     /* Grove 1 */
#define PIN_ENC_R_B          (1u)
#define PIN_ENC_L_A          (16u)    /* Grove 4 */
#define PIN_ENC_L_B          (17u)

/* ---- IMU: GY-511 / LSM303DLHC on I2C1 ---- */
#define PIN_I2C_SDA          (2u)     /* Grove 2 */
#define PIN_I2C_SCL          (3u)
#define IMU_I2C_HZ           (400000u)
#define PIN_IMU_INT1         (6u)     /* Grove 5 white */
#define IMU_ACCEL_ADDR       (0x19u)
#define IMU_MAG_ADDR         (0x1Eu)

/* ---- IR sensors (TCRT5000 modules) ---- */
#define PIN_IR_LEFT_DO       (4u)     /* Grove 3 white  */
#define PIN_IR_RIGHT_DO      (5u)     /* Grove 3 yellow */
#define PIN_IR_AOUT          (26u)    /* barcode sensor AO, Grove 6 white */
#define ADC_CH_IR            (0u)     /* GP26 = ADC0 */

/* ---- HC-SR04 ultrasonic ---- */
#define PIN_US_TRIG          (7u)     /* Grove 7 white */
#define PIN_US_ECHO          (27u)    /* Grove 6 yellow, direct */

/* ---- SG90 pan servo ---- */
#define PIN_SERVO            (12u)    /* servo column 12, PWM slice 6A, 50 Hz */

/* ---- Robo Pico on-board parts ---- */
#define PIN_NEOPIXEL         (18u)
#define PIN_BTN1             (20u)    /* start / end run / acknowledge */
#define PIN_BTN2             (21u)    /* local stop (latched like an E-stop) */
#define PIN_BUZZER           (22u)    /* PWM slice 3A */
#define PIN_VBAT_SENSE       (28u)    /* only if the VBAT jumper is soldered */
#define ADC_CH_VBAT          (2u)     /* GP28 = ADC2, reads VBAT / 2 */

/* GP19 is unused on this robot: the template's liveness LED pin is moved
   there (BOARD_LED_PIN in robot.mk) so it cannot drive GP16. */

#endif /* PINS_H */
