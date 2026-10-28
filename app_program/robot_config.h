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
#define SPEED_FF_DEADBAND_PCT     (6)      /* PLACEHOLDER until ft_speed_pid P1. Was 20; the 26 Oct telemetry shows
                                              the wheels hold 120 mm/s at only 13-18 % duty and ~35-40 mm/s at
                                              9-10 %, i.e. start-up ~6 %, ~11 mm/s per % */
#define SPEED_FF_MM_S_PER_PCT_X10 (110)    /* PLACEHOLDER until ft_speed_pid P1 (was 40 = 4.0 mm/s per %, which
                                              asked 50 % duty for 120 mm/s and ran slow inner wheels far too fast) */
#define SPEED_KP                  (0.0020f)     /* [P] duty per mm/s of error */
#define SPEED_KI                  (0.0100f)     /* [P] duty per mm of error   */

/* ---- From part_tests: test_ir ---- */
#define LINE_DO_BLACK_HIGH        (0)           /* spec 4: IR DO reads LOW over black */
#define IR_WHITE_ADC              (200)    /* PLACEHOLDER until test_barcode B2 (26 Oct track telemetry: white 160-240) */   /* test_ir I3, barcode sensor over white */
#define IR_BLACK_ADC              (3900)   /* PLACEHOLDER until test_barcode B2 (was 3000; 26 Oct track telemetry:
                                               black 3800-4095, so the switch level moves from 1600 to ~2050) */   /* test_ir I3, barcode sensor over black */

/* ---- From part_tests: test_servo ---- */
#define SERVO_MIN_US              (500)
#define SERVO_MAX_US              (2500)
#define SERVO_CALIBRATED          (1)      /* PLACEHOLDER: assumed */   /* 1 once V2 passes */
#define SERVO_LOW_ANGLE_IS_RIGHT  (1)
#define SERVO_TRIM_DEG            (20.5f)       /* [P] added to every pan angle, + = turn the sensor RIGHT. 28 Oct: at
                                                   the servo's centre (90) the sensor pointed 15-17 deg to the car's
                                                   left; 16 still left it 2-3 deg left, 18.5 still 2 deg left, so
                                                   20.5: straight ahead is 90 - 20.5 = 69.5, and the left / right
                                                   limits (-60 / +60 from straight ahead) are 129.5 / 9.5. Tune with
                                                   a flat board square ahead: shortest reading at 0 */

/* ---- From part_tests: test_imu U5 ---- */
#define IMU_FORWARD_AXIS          (1)      /* PLACEHOLDER: spec 5 says X forward; test_imu U5 */   /* 1 = X forward (spec 5 says X), 2 = Y */
#define IMU_FORWARD_SIGN          (1)      /* PLACEHOLDER: test_imu U5 */           /* +1 or -1, 0 = not set */

/* ---- Geometry (ruler on the assembled car) ---- */
#define IR_TO_AXLE_MM             (60)          /* [P] IR sensor row ahead of the wheel axle */
#define ROBOT_HALF_WIDTH_MM       (80)          /* half the widest part of the car */

/* ---- Line following (DR 6.1) ---- */
#define LINE_BASE_MM_S            (80)          /* [P] forward speed while following (was 120: too fast for the
                                                   ~115-145 mm loops with uncalibrated wheel speed control) */
#define LINE_STEER                (0.40f)       /* [P] steering when one sensor sees black (was 0.30) */
#define LINE_STEER_RAMP           (3.00f)       /* [P] extra steering per second it stays black (was 1.5) */
#define LINE_HOLD_STEER           (0.30f)       /* [P] steering kept after the sensor clears (was 0.12) */
#define LINE_HOLD_MM              (60)          /* [P] ... for this distance (was 40) */
#define LINE_CURVE_MM             (25)          /* black for longer than this = a curve */
#define LINE_CURVE_SLOW           (0.60f)       /* [P] base speed x (1 - this x |steer|): slow down in tight curves (0 = off; was 0.4) */
#define LINE_MIN_WHEEL_MM_S       (20)          /* [P] in a curve the inner wheel is braked (0) instead of being asked
                                                   for less than this: a slow target still gets the start-up duty and
                                                   creeps too fast, so the car runs wide (0 = off) */
#define LINE_BIAS_KI              (0.0f)        /* [P] curve memory, OFF by default: try 1.5-3 only after the speed
                                                   PI is calibrated (with a sluggish wheel loop it makes the car weave) */
#define LINE_BIAS_DECAY_MM        (400)         /* [P] curve memory fades over this much white */
#define LINE_LOST_MM              (250)         /* [P] no black this far after a curve = line lost */
#define LINE_SEARCH_DEG           (45)          /* first sweep of the search, doubled each pass */
#define LINE_SEARCH_MAX_DEG       (90)          /* sweeps 45, 90 deg each way, then EVF_LINE_LOST */
#define LINE_SEARCH_MM_S          (80)          /* wheel speed while searching */

/* ---- Barcodes (DR 6.1, FR5-FR8) ---- */
#define BARCODE_AT_JUNCTION       (1)           /* [P] 1 = store, act at the next cross-junction (demo brief), 0 = act at once */
#define BARCODE_TURN_DEG          (90)
#define BARCODE_MAX_LEN_MM        (400)         /* abandon a window longer than this */
#define BARCODE_MIN_ELEMENTS      (9)           /* fewer = a stray mark (e.g. a crossing line) */
#define BARCODE_HYS               (0.05f)       /* [P] switch levels = mid +/- this x (black - white).
                                                   Was 0.10: small prints (75 %, 50 %) blur the narrow
                                                   bars so they no longer reach mid + 10 %. Raise it only
                                                   if a plain white/black area gives false edges. */
#define BARCODE_DETECT            (1)           /* [P] 1 = bars found on 0.25 mm distance bins with edges at the
                                                   mid level (bc_detect.c: copes with a blurred sensor spot),
                                                   0 = the original 1 kHz two-level interrupt (BARCODE_HYS) */
#define BARCODE_LEVELS            (6)           /* [P] BARCODE_DETECT = 1 only: read every barcode this many ways at once
                                                   (1 = mid level only; 3 = 50/35/65 %; 5 = also 25/75 %; 6 = also the
                                                   local-contrast detector for pale / streaky bars next to black ones),
                                                   so faded / grey prints still decode (bc_multi.c). Applies at START */
#define BARCODE_ADAPT_SWING       (0.15f)       /* [P] BARCODE_LEVELS = 6: the local-contrast read makes an edge only where
                                                   the signal within +-4 mm changes by more than this x (black - white).
                                                   Lower = paler bars, but more false edges from ADC noise. Applies at START */
#define BARCODE_SENSOR_Y_MM       (35)          /* [P] barcode sensor's distance to the LEFT of the car's centre line
                                                   (negative = right; 0 = use the wheels' mean distance as before).
                                                   BARCODE_DETECT = 1 only: bar widths follow the sensor's own path,
                                                   so steering over a barcode no longer stretches bars. Measure it */
#define BARCODE_DET_HYS           (0.02f)       /* [P] BARCODE_DETECT = 1 only: band = mid +/- this x (black - white).
                                                   The averaging allows a narrow band; raise it (0.03-0.05) only if
                                                   plain white paper gives BC_DBG "stray mark" windows */
#define BARCODE_DEBUG             (1)           /* [P] 1 = publish every barcode window's bar/space widths (BC_DBG events) */

/* ---- Motion primitives ---- */
#define TURN_MM_S                 (90)          /* [P] wheel speed for pivot turns */
#define TURN_SNAP                 (0)           /* [P] 1 = finish junction turns on the line sensors (test first) */
#define BYPASS_MM_S               (60)          /* [P] speed for bypass legs (28 Oct: was 100; the leg watch sweeps the
                                                   sensor, one 0 / 30 / 0 / 60 deg cycle takes ~0.65 s = 4 cm at 60 mm/s) */
#define SPEED_MAX_MM_S            (250)         /* NFR18 speed cap */
#define HUMP_SPEED_PCT            (60)          /* DR A2: speed limit on a hump */
#define ENC_LOSS_DUTY             (0.30f)       /* DR 5: duty above this ... */
#define ENC_LOSS_MS               (200)         /* ... with no ticks for this long = encoder loss */

/* ---- Obstacles (DR 6.2) ---- */
#define OBSTACLE_STOP_CM          (20)          /* [P] FR9 */
#define OBST_MAX_TURN_DPS         (15.0f)       /* [P] turning faster than this: the sensor looks off the track,
                                                   so readings above PROXIMITY_STOP_CM do not trigger (0 = off).
                                                   Was 60: at 80 mm/s the loops turn at only 20-30 deg/s */
#define OBSTACLE_CLEAR_CM         (25)          /* coarse sweep: all above this = clear */
#define PROXIMITY_STOP_CM         (8)           /* NFR21 */
#define OBST_MARGIN_MM            (60)          /* [P] extra side clearance in the bypass */
#define OBST_DEPTH_MM             (150)         /* [P] assumed depth (ultrasonic cannot see the back) */
#define OBST_LEG_STOP_CM          (15)          /* [P] during bypass legs and the return the sensor sweeps 0 / 30 / 0 / 60 deg
                                                   towards the obstacle side: anything in the car's path (car width
                                                   + OBST_PATH_SIDE_MM) closer than this ahead, seen twice at the same
                                                   angle = this side is blocked: stop, back out along the legs, then
                                                   the other side or a rescan. Keep it above PROXIMITY_STOP_CM (8,
                                                   the E-stop); 0 = off */
#define OBST_PATH_SIDE_MM         (20)          /* leg watch: corridor = car half width + this on each side */
#define OBST_BACKOFF_MAX_MM       (250)         /* longest reverse to get the 45 deg standoff (see obstacle_standoff_mm) */
#define OBST_RESCAN_REVERSE_MM    (100)         /* reverse before a rescan */
#define OBST_RETURN_PAST_MM       (80)          /* return: drive this far (sideways) past the original path before
                                                   deciding the line was missed */
#define OBST_US_AHEAD_MM          (80)          /* PLACEHOLDER: ultrasonic sensor ahead of the wheel axle (measure) */
#define OBST_SIDE_MAX_MM          (1000)        /* parallel leg follows the obstacle's side at most this far */
#define OBST_SIDE_STEP_MM         (100)         /* look-before-turning: drive on this far while the side is still there */
#define OBST_TURN_LOOK_STEPS      (8)           /* ... at most this many times */
#define OBST_LOOK_PIVOT_DEG       (30)          /* look-before-turning: pivot this far towards the obstacle; with the
                                                   servo at 60 the sensor looks square at its side (90 deg) */
#define OBST_TIE_PIVOT_DEG        (30)          /* tie: pivot this far + servo 60 deg = a 90 deg look each way */
#define OBST_RETRY_WIDER_MM       (100)         /* a side tried again after it was blocked: this much more side margin */
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
