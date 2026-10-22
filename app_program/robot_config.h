/**
 * @file    robot_config.h
 * @brief   IS-02 calibration values and design constants.
 *
 * Calibration values come from the part tests and full tests. Copy them from
 * full_tests/include/robot_config.h once you have filled that in: the names
 * are the same. Every value still CAL_UNSET is listed on the USB console at
 * start-up and the robot stays in ERROR until it is set.
 *
 * Values marked [P] are also run-time parameters: they can be changed over
 * MQTT without recompiling ("SET:NAME=VALUE", see README_ROBOT.md, NFR14/15).
 * The value here is the start-up default.
 *
 * 21 Oct: values marked PLACEHOLDER are reasonable guesses so the robot can
 * run before calibration. Replace them with the measured values (or send the
 * measured values with laptop_tools/robot_params.py, which overrides these).
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define CAL_UNSET                 (-1)

/* ---- From part_tests: test_motors / test_encoders ---- */
#define MOTOR_MAPPING_CONFIRMED   (1)   /* PLACEHOLDER: assumed (spec 3.2 wires swapped) */   /* 1 once M1-M4 pass (right motor wires swapped, spec 3.2) */
#define ENC_R_REVERSED            (0)
#define ENC_L_REVERSED            (1)           /* 21 Oct: left counted DOWN rolling forward */
#define ENC_DIRECTION_CONFIRMED   (1)   /* PLACEHOLDER: assumed */   /* 1 once E1 and E4 pass on both wheels */
#define ENC_A_EDGES_PER_REV       (1260)   /* PLACEHOLDER: 21 Oct E1 hand turn (right); run E2 */   /* test_encoders E2 (A edges per wheel turn) */
#define WHEEL_DIAMETER_MM         (65)     /* PLACEHOLDER: measure (E5) */   /* test_encoders E5 (ruler) */
#define TRACK_WIDTH_MM            (130)    /* PLACEHOLDER: measure with a ruler */   /* ruler: tyre centre to tyre centre */

/* ---- From full_tests: ft_speed_pid P1 ---- */
#define SPEED_FF_DEADBAND_PCT     (20)     /* PLACEHOLDER: ft_speed_pid P1 */
#define SPEED_FF_MM_S_PER_PCT_X10 (40)     /* PLACEHOLDER: ft_speed_pid P1 */
#define SPEED_KP                  (0.0020f)     /* [P] duty per mm/s of error */
#define SPEED_KI                  (0.0100f)     /* [P] duty per mm of error   */

/* ---- From part_tests: test_ir ---- */
#define LINE_DO_BLACK_HIGH        (0)           /* spec 4: IR DO reads LOW over black */
#define IR_WHITE_ADC              (200)    /* PLACEHOLDER: test_ir I3 */   /* test_ir I3, barcode sensor over white */
#define IR_BLACK_ADC              (3000)   /* PLACEHOLDER: test_ir I3 */   /* test_ir I3, barcode sensor over black */

/* ---- From part_tests: test_servo ---- */
#define SERVO_MIN_US              (500)
#define SERVO_MAX_US              (2500)
#define SERVO_CALIBRATED          (1)      /* PLACEHOLDER: assumed */   /* 1 once V2 passes */
#define SERVO_LOW_ANGLE_IS_RIGHT  (1)

/* ---- From part_tests: test_imu U5 ---- */
#define IMU_FORWARD_AXIS          (1)      /* PLACEHOLDER: spec 5 says X forward; test_imu U5 */   /* 1 = X forward (spec 5 says X), 2 = Y */
#define IMU_FORWARD_SIGN          (1)      /* PLACEHOLDER: test_imu U5 */           /* +1 or -1, 0 = not set */

/* ---- Geometry (ruler on the assembled car) ---- */
#define IR_TO_AXLE_MM             (60)          /* [P] IR sensor row ahead of the wheel axle */
#define ROBOT_HALF_WIDTH_MM       (80)          /* half the widest part of the car */

/* ---- Line following (DR 6.1) ---- */
#define LINE_BASE_MM_S            (120)         /* [P] forward speed while following */
#define LINE_STEER                (0.30f)       /* [P] steering when one sensor sees black */
#define LINE_STEER_RAMP           (1.50f)       /* [P] extra steering per second it stays black */
#define LINE_HOLD_STEER           (0.12f)       /* [P] steering kept after the sensor clears */
#define LINE_HOLD_MM              (40)          /* [P] ... for this distance */
#define LINE_CURVE_MM             (25)          /* black for longer than this = a curve */
#define LINE_LOST_MM              (250)         /* [P] no black this far after a curve = line lost */
#define LINE_SEARCH_DEG           (45)          /* first sweep of the search, doubled each pass */
#define LINE_SEARCH_MAX_DEG       (90)          /* sweeps 45, 90 deg each way, then EVF_LINE_LOST */
#define LINE_SEARCH_MM_S          (80)          /* wheel speed while searching */

/* ---- Barcodes (DR 6.1, FR5-FR8) ---- */
#define BARCODE_AT_JUNCTION       (0)           /* [P] 0 = act as soon as decoded (chosen), 1 = at next junction */
#define BARCODE_TURN_DEG          (90)
#define BARCODE_MAX_LEN_MM        (400)         /* abandon a window longer than this */
#define BARCODE_MIN_ELEMENTS      (9)           /* fewer = a stray mark (e.g. a crossing line) */

/* ---- Motion primitives ---- */
#define TURN_MM_S                 (90)          /* [P] wheel speed for pivot turns */
#define BYPASS_MM_S               (100)         /* [P] speed for bypass legs */
#define SPEED_MAX_MM_S            (250)         /* NFR18 speed cap */
#define HUMP_SPEED_PCT            (60)          /* DR A2: speed limit on a hump */
#define ENC_LOSS_DUTY             (0.30f)       /* DR 5: duty above this ... */
#define ENC_LOSS_MS               (200)         /* ... with no ticks for this long = encoder loss */

/* ---- Obstacles (DR 6.2) ---- */
#define OBSTACLE_STOP_CM          (20)          /* [P] FR9 */
#define OBSTACLE_CLEAR_CM         (25)          /* coarse sweep: all above this = clear */
#define PROXIMITY_STOP_CM         (8)           /* NFR21 */
#define OBST_MARGIN_MM            (60)          /* [P] extra side clearance in the bypass */
#define OBST_DEPTH_MM             (150)         /* [P] assumed depth (ultrasonic cannot see the back) */
#define REACQ_MAX_LATERAL_MM      (300)         /* DR: abort if the line is not found */
#define FINE_STEP_DEG             (2)
#define SERVO_SETTLE_MS           (50)          /* DR A6 */

/* ---- Terrain / IMU (DR 6.3) ---- */
#define HUMP_START_DEG            (5.0f)
#define HUMP_LEVEL_DEG            (3.0f)
#define HUMP_MIN_HEIGHT_MM        (15.0f)
#define HUMP_START_SAMPLES        (3)
#define HUMP_END_SAMPLES          (5)
#define IMU_EMA_ALPHA             (0.3f)        /* DR 6.3: alpha = 0.3 */
#define IMPACT_G                  (2.0f)        /* FR23 software backup threshold */
#define TURNING_DPS               (30.0f)
#define ACCEL_G                   (0.05f)
#define STATIONARY_MM_S           (10.0f)

/* ---- Battery (NFR20): only if the VBAT jumper is soldered (spec 3.7) ---- */
#define VBAT_ENABLED              (0)
#define VBAT_LOW_MV               (4600)
#define VBAT_CRITICAL_MV          (4300)

/* ---- Safety (DR 5) ---- */
#define HEARTBEAT_TIMEOUT_MS      (300)         /* control task silent this long = watchdog E-stop */

#endif /* ROBOT_CONFIG_H */
