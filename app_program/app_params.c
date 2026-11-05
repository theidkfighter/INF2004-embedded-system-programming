/**
 * @file    app_params.c
 * @brief   Run-time parameter table (see app_params.h).
 */
#include "app_params.h"
#include "jfmt.h"
#include "robot_config.h"

app_params_t g_params;

typedef struct
{
    const char *p_name;
    float      *p_value;      /* NULL for a packed parameter */
    float       min;
    float       max;
    uint32_t    desc;         /* PRM_x for a packed parameter, 0 for a float */
} param_def_t;

static const param_def_t PARAMS[] =
{
    { "MOTOR_MAPPING_CONFIRMED", (float *)0, -1.0f, 1.0f, PRM_MOTOR_MAPPING_CONFIRMED },
    { "ENC_R_REVERSED", (float *)0, 0.0f, 1.0f, PRM_ENC_R_REVERSED },
    { "ENC_L_REVERSED", (float *)0, 0.0f, 1.0f, PRM_ENC_L_REVERSED },
    { "ENC_DIRECTION_CONFIRMED", (float *)0, -1.0f, 1.0f, PRM_ENC_DIRECTION_CONFIRMED },
    { "LINE_DO_BLACK_HIGH", (float *)0, -1.0f, 1.0f, PRM_LINE_DO_BLACK_HIGH },
    { "SERVO_MIN_US",          &g_params.servo_min_us,        300.0f, 1500.0f, 0u },
    { "SERVO_MAX_US",          &g_params.servo_max_us,        1500.0f, 2800.0f, 0u },
    { "SERVO_CALIBRATED", (float *)0, -1.0f, 1.0f, PRM_SERVO_CALIBRATED },
    { "SERVO_LOW_ANGLE_IS_RIGHT", (float *)0, 0.0f, 1.0f, PRM_SERVO_LOW_ANGLE_IS_RIGHT },
    { "SERVO_TRIM_DEG",        &g_params.servo_trim_deg,      -45.0f, 45.0f, 0u },
    { "IMU_FORWARD_AXIS", (float *)0, -1.0f, 2.0f, PRM_IMU_FORWARD_AXIS },
    { "IMU_FORWARD_SIGN", (float *)0, -1.0f, 1.0f, PRM_IMU_FORWARD_SIGN },
    { "ROBOT_HALF_WIDTH_MM",   &g_params.robot_half_width_mm, 40.0f,  200.0f, 0u },
    { "ENC_A_EDGES_PER_REV",   &g_params.enc_a_edges_per_rev, 10.0f,  20000.0f, 0u },
    { "WHEEL_DIAMETER_MM",     &g_params.wheel_diameter_mm,   20.0f,  150.0f, 0u },
    { "TRACK_WIDTH_MM",        &g_params.track_width_mm,      50.0f,  300.0f, 0u },
    { "SPEED_FF_DEADBAND_PCT", &g_params.ff_deadband_pct,     0.0f,   90.0f, 0u },
    { "SPEED_FF_MM_S_PER_PCT", &g_params.ff_mm_s_per_pct,     0.1f,   100.0f, 0u },
    { "IR_WHITE_ADC",          &g_params.ir_white_adc,        0.0f,   4095.0f, 0u },
    { "IR_BLACK_ADC",          &g_params.ir_black_adc,        0.0f,   4095.0f, 0u },
    { "SPEED_KP",              &g_params.speed_kp,            0.0f,   0.1f, 0u },
    { "SPEED_KI",              &g_params.speed_ki,            0.0f,   1.0f, 0u },
    { "LINE_BASE_MM_S",        &g_params.line_base_mm_s,      (float)LINE_BASE_SAFE_MIN_MM_S, (float)LINE_BASE_SAFE_MAX_MM_S, 0u },
    { "LINE_STEER",            &g_params.line_steer,          0.0f,   1.0f, 0u },
    { "LINE_STEER_RAMP",       &g_params.line_steer_ramp,     0.0f,   20.0f, 0u },
    { "LINE_HOLD_STEER",       &g_params.line_hold_steer,     0.0f,   1.0f, 0u },
    { "LINE_HOLD_MM",          &g_params.line_hold_mm,        0.0f,   300.0f, 0u },
    { "LINE_LOST_MM",          &g_params.line_lost_mm,        50.0f,  3000.0f, 0u },
    { "LINE_CURVE_SLOW",       &g_params.line_curve_slow,     0.0f,   0.8f, 0u },
    { "LINE_MIN_WHEEL_MM_S",   &g_params.line_min_wheel_mm_s, 0.0f,   80.0f, 0u },
    { "LINE_BIAS_KI",          &g_params.line_bias_ki,        0.0f,   10.0f, 0u },
    { "LINE_BIAS_DECAY_MM",    &g_params.line_bias_decay_mm,  0.0f,   3000.0f, 0u },
    { "OBST_MAX_TURN_DPS",     &g_params.obst_max_turn_dps,   0.0f,   720.0f, 0u },
    { "TURN_SNAP", (float *)0, 0.0f, 1.0f, PRM_TURN_SNAP },
    { "BARCODE_AT_JUNCTION", (float *)0, 0.0f, 1.0f, PRM_BARCODE_AT_JUNCTION },
    { "BARCODE_HYS",           &g_params.barcode_hys,         0.01f,  0.30f, 0u },
    { "BARCODE_DETECT", (float *)0, 0.0f, 1.0f, PRM_BARCODE_DETECT },
    { "BARCODE_DET_HYS",       &g_params.barcode_det_hys,     0.005f, 0.30f, 0u },
    { "BARCODE_SENSOR_Y_MM",   &g_params.barcode_sensor_y_mm, -150.0f, 150.0f, 0u },
    { "BARCODE_LEVELS", (float *)0, 1.0f, 6.0f, PRM_BARCODE_LEVELS },
    { "BARCODE_ADAPT_SWING",   &g_params.barcode_adapt_swing, 0.05f,  0.60f, 0u },
    { "BARCODE_DEBUG", (float *)0, 0.0f, 1.0f, PRM_BARCODE_DEBUG },
    { "TURN_MM_S",             &g_params.turn_mm_s,           30.0f,  (float)SPEED_MAX_MM_S, 0u },
    { "BYPASS_MM_S",           &g_params.bypass_mm_s,         30.0f,  (float)SPEED_MAX_MM_S, 0u },
    { "OBSTACLE_STOP_CM",      &g_params.obstacle_stop_cm,    10.0f,  60.0f, 0u },
    { "OBST_MARGIN_MM",        &g_params.obst_margin_mm,      0.0f,   300.0f, 0u },
    { "OBST_DEPTH_MM",         &g_params.obst_depth_mm,       20.0f,  600.0f, 0u },
    { "OBST_LEG_STOP_CM",      &g_params.obst_leg_stop_cm,    0.0f,   40.0f, 0u },
    { "OBST_REVERSE_MAX_MM",   &g_params.obst_reverse_max_mm, 0.0f,   1000.0f, 0u },
    { "OBST_LEGACY", (float *)0, 0.0f, 1.0f, PRM_OBST_LEGACY },
    { "OBST_MIN_WIDTH_MM",     &g_params.obst_min_width_mm,   0.0f,   600.0f, 0u },
    { "OBST_TRUST_MISS", (float *)0, 0.0f, 1.0f, PRM_OBST_TRUST_MISS },
    { "OBST_EDGE_PAD_MM",      &g_params.obst_edge_pad_mm,    0.0f,   100.0f, 0u },
    { "ROBOT_LEFT_HALF_MM",    &g_params.robot_left_half_mm,  0.0f,   250.0f, 0u },
    { "IR_TO_AXLE_MM",         &g_params.ir_to_axle_mm,       0.0f,   200.0f, 0u },
    { "JUNCTION_MIN_MM",       &g_params.junction_min_mm,     0.0f,   30.0f, 0u },
    { "JUNCTION_BLANK_MM",     &g_params.junction_blank_mm,   0.0f,   1000.0f, 0u },
    { "JUNCTION_MAX_TURN_DEG", &g_params.junction_max_turn_deg, 0.0f, 90.0f, 0u },
    { "JUNCTION_BLACK_HIGH", (float *)0, 0.0f, 1.0f, PRM_JUNCTION_BLACK_HIGH },
    { "TURN_PAUSE_MS",         &g_params.turn_pause_ms,       0.0f,   1000.0f, 0u },
    { "JUNCTION_MAX_MM",       &g_params.junction_max_mm,     0.0f,   100.0f, 0u },
    { "JUNCTION_SIDE_CHECK", (float *)0, 0.0f, 1.0f, PRM_JUNCTION_SIDE_CHECK },
};

#define PARAM_COUNT (sizeof(PARAMS) / sizeof(PARAMS[0]))

static uint16_t g_table_crc;   /* params_table_crc(), set in params_init() */

/* Packed parameter: value -> code, false if not whole or not representable. */
static bool prm_code(uint32_t d, float v, uint32_t *p_code)
{
    int32_t  i = (int32_t)v;
    uint32_t e = (d >> 8) & 7u;
    uint32_t w = (d >> 5) & 7u;

    if ((float)i != v)
    {
        return false;                         /* D15: packed parameters hold whole numbers */
    }
    if (e == PE_TRI)
    {
        i = (i < 0) ? (i + 4) : i;            /* S2 two's complement */
    }
    else if (e == PE_AXIS)
    {
        i = i + 1;
    }
    else if (e == PE_LEVELS)
    {
        i = i - 1;
    }
    else
    {
        /* PE_FLAG */
    }
    if ((i < 0) || ((uint32_t)i >= (1u << w)))
    {
        return false;
    }
    *p_code = (uint32_t)i;
    return true;
}

/* One composed 32-bit store (single writer: TSK_TELEMETRY, or params_init). */
static bool prm_put(uint32_t d, float v)
{
    uint32_t code;
    uint32_t mask = ((1u << ((d >> 5) & 7u)) - 1u) << (d & 31u);

    if (!prm_code(d, v, &code))
    {
        return false;
    }
    g_params.bits = (g_params.bits & ~mask) | (code << (d & 31u));
    return true;
}

static float param_value(uint32_t i)
{
    return (PARAMS[i].p_value != (float *)0) ? *PARAMS[i].p_value : (float)prm_int(PARAMS[i].desc);
}

/* min / max already checked; -3 when a packed value is not whole */
static int32_t param_store(uint32_t i, float v)
{
    if (PARAMS[i].p_value != (float *)0)
    {
        *PARAMS[i].p_value = v;
        return 0;
    }
    return prm_put(PARAMS[i].desc, v) ? 0 : -3;
}

float params_min(uint32_t id)
{
    return (id < PARAM_COUNT) ? PARAMS[id].min : 0.0f;
}

float params_max(uint32_t id)
{
    return (id < PARAM_COUNT) ? PARAMS[id].max : 0.0f;
}

uint32_t params_store_width(uint32_t id)
{
    return ((id < PARAM_COUNT) && (PARAMS[id].p_value == (float *)0)) ? ((PARAMS[id].desc >> 5) & 7u) : 32u;
}

uint32_t params_store_code(uint32_t id)
{
    union { float f; uint32_t u; } v;

    if (id >= PARAM_COUNT)
    {
        return 0u;
    }
    if (PARAMS[id].p_value == (float *)0)
    {
        return prm_raw(PARAMS[id].desc);
    }
    v.f = *PARAMS[id].p_value;
    return v.u;
}
static uint16_t table_crc(void);

void params_init(void)
{
    g_table_crc = table_crc();
    g_params.bits = 0u;
    (void)prm_put(PRM_MOTOR_MAPPING_CONFIRMED, (float)MOTOR_MAPPING_CONFIRMED);
    (void)prm_put(PRM_ENC_R_REVERSED, (float)ENC_R_REVERSED);
    (void)prm_put(PRM_ENC_L_REVERSED, (float)ENC_L_REVERSED);
    (void)prm_put(PRM_ENC_DIRECTION_CONFIRMED, (float)ENC_DIRECTION_CONFIRMED);
    (void)prm_put(PRM_LINE_DO_BLACK_HIGH, (float)LINE_DO_BLACK_HIGH);
    g_params.servo_min_us             = (float)SERVO_MIN_US;
    g_params.servo_max_us             = (float)SERVO_MAX_US;
    (void)prm_put(PRM_SERVO_CALIBRATED, (float)SERVO_CALIBRATED);
    (void)prm_put(PRM_SERVO_LOW_ANGLE_IS_RIGHT, (float)SERVO_LOW_ANGLE_IS_RIGHT);
    g_params.servo_trim_deg           = (float)SERVO_TRIM_DEG;
    (void)prm_put(PRM_IMU_FORWARD_AXIS, (float)IMU_FORWARD_AXIS);
    (void)prm_put(PRM_IMU_FORWARD_SIGN, (float)IMU_FORWARD_SIGN);
    g_params.robot_half_width_mm      = (float)ROBOT_HALF_WIDTH_MM;
    g_params.enc_a_edges_per_rev = (float)ENC_A_EDGES_PER_REV;
    g_params.wheel_diameter_mm   = (float)WHEEL_DIAMETER_MM;
    g_params.track_width_mm      = (float)TRACK_WIDTH_MM;
    g_params.ff_deadband_pct     = (float)SPEED_FF_DEADBAND_PCT;
    g_params.ff_mm_s_per_pct     = (SPEED_FF_MM_S_PER_PCT_X10 == CAL_UNSET) ? (float)CAL_UNSET
                                   : ((float)SPEED_FF_MM_S_PER_PCT_X10 / 10.0f);
    g_params.ir_white_adc        = (float)IR_WHITE_ADC;
    g_params.ir_black_adc        = (float)IR_BLACK_ADC;
    g_params.speed_kp            = SPEED_KP;
    g_params.speed_ki            = SPEED_KI;
    g_params.line_base_mm_s      = (float)LINE_BASE_MM_S;
    g_params.line_steer          = LINE_STEER;
    g_params.line_steer_ramp     = LINE_STEER_RAMP;
    g_params.line_hold_steer     = LINE_HOLD_STEER;
    g_params.line_hold_mm        = (float)LINE_HOLD_MM;
    g_params.line_lost_mm        = (float)LINE_LOST_MM;
    g_params.line_curve_slow     = LINE_CURVE_SLOW;
    g_params.line_min_wheel_mm_s = (float)LINE_MIN_WHEEL_MM_S;
    g_params.line_bias_ki        = LINE_BIAS_KI;
    g_params.line_bias_decay_mm  = (float)LINE_BIAS_DECAY_MM;
    g_params.obst_max_turn_dps   = OBST_MAX_TURN_DPS;
    (void)prm_put(PRM_TURN_SNAP, (float)TURN_SNAP);
    (void)prm_put(PRM_BARCODE_AT_JUNCTION, (float)BARCODE_AT_JUNCTION);
    g_params.barcode_hys         = BARCODE_HYS;
    (void)prm_put(PRM_BARCODE_DETECT, (float)BARCODE_DETECT);
    g_params.barcode_det_hys     = BARCODE_DET_HYS;
    g_params.barcode_sensor_y_mm = (float)BARCODE_SENSOR_Y_MM;
    (void)prm_put(PRM_BARCODE_LEVELS, (float)BARCODE_LEVELS);
    g_params.barcode_adapt_swing = BARCODE_ADAPT_SWING;
    (void)prm_put(PRM_BARCODE_DEBUG, (float)BARCODE_DEBUG);
    g_params.turn_mm_s           = (float)TURN_MM_S;
    g_params.bypass_mm_s         = (float)BYPASS_MM_S;
    g_params.obstacle_stop_cm    = (float)OBSTACLE_STOP_CM;
    g_params.obst_margin_mm      = (float)OBST_MARGIN_MM;
    g_params.obst_depth_mm       = (float)OBST_DEPTH_MM;
    g_params.obst_leg_stop_cm    = (float)OBST_LEG_STOP_CM;
    g_params.obst_reverse_max_mm = (float)OBST_REVERSE_MAX_MM;
    (void)prm_put(PRM_OBST_LEGACY, (float)OBST_LEGACY);
    g_params.obst_min_width_mm   = (float)OBST_MIN_WIDTH_MM;
    (void)prm_put(PRM_OBST_TRUST_MISS, (float)OBST_TRUST_MISS);
    g_params.obst_edge_pad_mm    = (float)OBST_EDGE_PAD_MM;
    g_params.robot_left_half_mm  = (float)ROBOT_LEFT_HALF_MM;
    g_params.ir_to_axle_mm       = (float)IR_TO_AXLE_MM;
    g_params.junction_min_mm     = (float)JUNCTION_MIN_MM;
    g_params.junction_blank_mm   = (float)JUNCTION_BLANK_MM;
    g_params.junction_max_turn_deg = (float)JUNCTION_MAX_TURN_DEG;
    (void)prm_put(PRM_JUNCTION_BLACK_HIGH, (float)JUNCTION_BLACK_HIGH);
    g_params.turn_pause_ms       = (float)TURN_PAUSE_MS;
    g_params.junction_max_mm     = (float)JUNCTION_MAX_MM;
    (void)prm_put(PRM_JUNCTION_SIDE_CHECK, (float)JUNCTION_SIDE_CHECK);
}

int32_t params_set(const char *p_text)
{
    while ((*p_text == ' ') || (*p_text == '\t'))
    {
        p_text++;
    }
    if (jf_starts_with(p_text, "SET:"))
    {
        p_text = &p_text[4];
    }
    for (uint32_t i = 0u; i < PARAM_COUNT; i++)
    {
        const char *p_name = PARAMS[i].p_name;
        uint32_t    n = jf_strlen(p_name);

        if (jf_starts_with(p_text, p_name) && (p_text[n] == '='))
        {
            float value;

            if (!jf_parse_float(&p_text[n + 1u], &value))
            {
                return -2;
            }
            if ((value < PARAMS[i].min) || (value > PARAMS[i].max))
            {
                return -3;
            }
            return param_store(i, value);
        }
    }
    return -1;
}

void params_set_many(const char *p_text, uint32_t *p_ok, uint32_t *p_bad,
                     char *p_bad_names, uint32_t size)
{
    jbuf_t jb;
    char   item[64];

    *p_ok  = 0u;
    *p_bad = 0u;
    jb_init(&jb, p_bad_names, size);
    while (*p_text != '\0')
    {
        uint32_t n = 0u;

        while ((*p_text != '\0') && (*p_text != '\n') && (*p_text != '\r') &&
               (*p_text != ';') && (*p_text != ','))
        {
            if ((n + 1u) < sizeof(item))
            {
                item[n] = *p_text;
                n++;
            }
            p_text++;
        }
        item[n] = '\0';
        if (*p_text != '\0')
        {
            p_text++;
        }
        /* skip blank lines and # comments */
        {
            const char *p = item;

            while ((*p == ' ') || (*p == '\t'))
            {
                p++;
            }
            if ((*p == '\0') || (*p == '#'))
            {
                continue;
            }
            if (params_set(p) == 0)
            {
                (*p_ok)++;
            }
            else
            {
                if (*p_bad > 0u)
                {
                    jb_char(&jb, ' ');
                }
                jb_raw(&jb, p);
                (*p_bad)++;
            }
        }
    }
}

int32_t params_set_id(uint32_t id, float value)
{
    if (id >= PARAM_COUNT)
    {
        return -1;
    }
    if ((value != value) || (value > 3.0e38f) || (value < -3.0e38f))
    {
        return -2;
    }
    if ((value < PARAMS[id].min) || (value > PARAMS[id].max))
    {
        return -3;
    }
    return param_store(id, value);
}

bool params_get_id(uint32_t id, float *p_value)
{
    if (id >= PARAM_COUNT)
    {
        return false;
    }
    *p_value = param_value(id);
    return true;
}

const char *params_name(uint32_t id)
{
    return (id < PARAM_COUNT) ? PARAMS[id].p_name : (const char *)0;
}

/* Parameter table version (design D15): CRC-16/CCITT-FALSE (poly 0x1021,
   init 0xFFFF, no reflection, no final XOR) over every NAME in ID order, each
   followed by '\n'. The table is fixed at build time, so it is computed once. */
static uint32_t crc_byte(uint32_t crc, uint32_t byte)
{
    crc ^= (byte & 0xFFu) << 8;
    for (uint32_t b = 0u; b < 8u; b++)
    {
        crc = ((crc & 0x8000u) != 0u) ? (((crc << 1) ^ 0x1021u) & 0xFFFFu) : ((crc << 1) & 0xFFFFu);
    }
    return crc;
}

static uint16_t table_crc(void)
{
    uint32_t crc = 0xFFFFu;

    for (uint32_t i = 0u; i < PARAM_COUNT; i++)
    {
        for (const char *p = PARAMS[i].p_name; *p != '\0'; p++)
        {
            crc = crc_byte(crc, (uint32_t)(uint8_t)*p);
        }
        crc = crc_byte(crc, (uint32_t)'\n');
    }
    return (uint16_t)crc;
}

uint16_t params_table_crc(void)
{
    return g_table_crc;
}

uint32_t params_count(void)
{
    return (uint32_t)PARAM_COUNT;
}

uint32_t params_to_json(char *p_buf, uint32_t size, uint32_t first, uint32_t count)
{
    jbuf_t jb;

    jb_init(&jb, p_buf, size);
    jb_char(&jb, '{');
    for (uint32_t i = first; (i < PARAM_COUNT) && (i < (first + count)); i++)
    {
        if (i > first)
        {
            jb_char(&jb, ',');
        }
        jb_key(&jb, PARAMS[i].p_name);
        jb_fixed(&jb, param_value(i), 4u);
    }
    jb_char(&jb, '}');
    return jb_len(&jb);
}

float params_mm_per_count(void)
{
    if ((g_params.enc_a_edges_per_rev <= 0.0f) || (g_params.wheel_diameter_mm <= 0.0f))
    {
        return 0.0f;
    }
    /* 4x decoding: A edges + B edges = 2 x A edges per revolution */
    return (3.14159265f * g_params.wheel_diameter_mm) / (2.0f * g_params.enc_a_edges_per_rev);
}

float params_servo_phys(float robot_deg)
{
    float a = robot_deg + g_params.servo_trim_deg;     /* + = right, trimmed */

    return prm_flag(PRM_SERVO_LOW_ANGLE_IS_RIGHT) ? (90.0f - a) : (90.0f + a);
}
