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

/* Parameter_Store (Change Requirement 1, design D15). The 43 continuous
   parameters stay float members. The 17 discrete ones live in `bits`:
   one 32-bit word written only by TSK_TELEMETRY as one composed store, read
   by any task with one load through prm_flag() / prm_tri() / prm_int().
   Descriptor PRM_x = shift | width << 5 | encoding << 8. */
#define PE_FLAG     (1u)    /* F1 0..1 */
#define PE_TRI      (2u)    /* S2 -1..1 (CAL_UNSET = -1) */
#define PE_AXIS     (3u)    /* B2+(-1): -1..2 */
#define PE_LEVELS   (4u)    /* B3+1: 1..6 */
#define PRM_DESC(sh, w, e)  ((uint32_t)(sh) | ((uint32_t)(w) << 5) | ((uint32_t)(e) << 8))
#define PRM_ENC_R_REVERSED             PRM_DESC( 0u, 1u, PE_FLAG)
#define PRM_ENC_L_REVERSED             PRM_DESC( 1u, 1u, PE_FLAG)
#define PRM_SERVO_LOW_ANGLE_IS_RIGHT   PRM_DESC( 2u, 1u, PE_FLAG)
#define PRM_TURN_SNAP                  PRM_DESC( 3u, 1u, PE_FLAG)
#define PRM_BARCODE_AT_JUNCTION        PRM_DESC( 4u, 1u, PE_FLAG)
#define PRM_BARCODE_DETECT             PRM_DESC( 5u, 1u, PE_FLAG)
#define PRM_BARCODE_DEBUG              PRM_DESC( 6u, 1u, PE_FLAG)
#define PRM_OBST_LEGACY                PRM_DESC( 7u, 1u, PE_FLAG)
#define PRM_OBST_TRUST_MISS            PRM_DESC( 8u, 1u, PE_FLAG)
#define PRM_MOTOR_MAPPING_CONFIRMED    PRM_DESC( 9u, 2u, PE_TRI)
#define PRM_ENC_DIRECTION_CONFIRMED    PRM_DESC(11u, 2u, PE_TRI)
#define PRM_LINE_DO_BLACK_HIGH         PRM_DESC(13u, 2u, PE_TRI)
#define PRM_SERVO_CALIBRATED           PRM_DESC(15u, 2u, PE_TRI)
#define PRM_IMU_FORWARD_SIGN           PRM_DESC(17u, 2u, PE_TRI)
#define PRM_IMU_FORWARD_AXIS           PRM_DESC(19u, 2u, PE_AXIS)
#define PRM_BARCODE_LEVELS             PRM_DESC(21u, 3u, PE_LEVELS)
#define PRM_JUNCTION_BLACK_HIGH        PRM_DESC(24u, 1u, PE_FLAG)    /* patch/junction_check */

typedef struct
{
    uint32_t bits;          /* the 17 discrete parameters, D15 + JUNCTION_BLACK_HIGH; bits [25..31] are 0 */
    /* confirmations and directions from the part tests (1 = yes / reversed) */
    float servo_min_us;
    float servo_max_us;
    float servo_trim_deg;
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
    float line_curve_slow;
    float line_min_wheel_mm_s;
    float line_bias_ki;
    float line_bias_decay_mm;
    float obst_max_turn_dps;
    float barcode_hys;
    float barcode_det_hys;
    float barcode_sensor_y_mm;
    float barcode_adapt_swing;
    /* motion primitives / obstacles */
    float turn_mm_s;
    float bypass_mm_s;
    float obstacle_stop_cm;
    float obst_margin_mm;
    float obst_depth_mm;
    float obst_leg_stop_cm;
    float obst_reverse_max_mm;
    float obst_min_width_mm;
    float obst_edge_pad_mm;
    float robot_left_half_mm;
    float ir_to_axle_mm;
    /* junction check (patch/junction_check) */
    float junction_min_mm;
    float junction_max_turn_deg;
    float turn_pause_ms;
    float junction_max_mm;
} app_params_t;

extern app_params_t g_params;

static inline uint32_t prm_raw(uint32_t d)
{
    return (g_params.bits >> (d & 31u)) & ((1u << ((d >> 5) & 7u)) - 1u);
}

static inline bool prm_flag(uint32_t d)          /* F1: value 1 */
{
    return prm_raw(d) != 0u;
}

static inline int32_t prm_tri(uint32_t d)        /* S2: -1, 0, 1 */
{
    uint32_t c = prm_raw(d);

    return (c >= 2u) ? ((int32_t)c - 4) : (int32_t)c;
}

static inline int32_t prm_int(uint32_t d)        /* any encoding as its whole value */
{
    uint32_t e = (d >> 8) & 7u;

    return (e == PE_TRI) ? prm_tri(d) : ((e == PE_AXIS) ? ((int32_t)prm_raw(d) - 1) :
                                         ((e == PE_LEVELS) ? ((int32_t)prm_raw(d) + 1) : (int32_t)prm_raw(d)));
}

/* Bits of Parameter_ID id in its Parameter_Store representation (32 = F32),
   and that code (GET:PARAMS acks). */
uint32_t params_store_width(uint32_t id);
float    params_min(uint32_t id);               /* PARAMS[] limits (0 for an unknown ID) */
float    params_max(uint32_t id);
uint32_t params_store_code(uint32_t id);

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

/* Binary access by Parameter_ID (the table position, 0..params_count()-1),
   used by the Wire_Codec frames SET, GET:PARAMS and Config_Frame.
   params_set_id: 0 ok, -1 unknown ID, -2 not a finite number, -3 out of range
   (or not a whole number for a packed parameter, D15). */
int32_t     params_set_id(uint32_t id, float value);
bool        params_get_id(uint32_t id, float *p_value);
const char *params_name(uint32_t id);            /* NULL for an unknown ID */
uint16_t    params_table_crc(void);              /* D15 table version: CRC-16/CCITT-FALSE of the NAMEs (heartbeat) */

/* Derived values (0 if the calibration is missing) */
float params_mm_per_count(void);     /* 4x quadrature count */
float params_servo_phys(float robot_deg);   /* pan angle (+ right, 0 = straight ahead) -> servo degrees, with SERVO_TRIM_DEG */

#endif /* APP_PARAMS_H */
