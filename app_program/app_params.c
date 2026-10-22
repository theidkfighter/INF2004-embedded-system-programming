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
    float      *p_value;
    float       min;
    float       max;
} param_def_t;

static const param_def_t PARAMS[] =
{
    { "MOTOR_MAPPING_CONFIRMED", &g_params.motor_mapping_confirmed, -1.0f, 1.0f },
    { "ENC_R_REVERSED",        &g_params.enc_r_reversed,      0.0f,   1.0f     },
    { "ENC_L_REVERSED",        &g_params.enc_l_reversed,      0.0f,   1.0f     },
    { "ENC_DIRECTION_CONFIRMED", &g_params.enc_direction_confirmed, -1.0f, 1.0f },
    { "LINE_DO_BLACK_HIGH",    &g_params.line_do_black_high,  -1.0f,  1.0f     },
    { "SERVO_MIN_US",          &g_params.servo_min_us,        300.0f, 1500.0f  },
    { "SERVO_MAX_US",          &g_params.servo_max_us,        1500.0f, 2800.0f },
    { "SERVO_CALIBRATED",      &g_params.servo_calibrated,    -1.0f,  1.0f     },
    { "SERVO_LOW_ANGLE_IS_RIGHT", &g_params.servo_low_angle_is_right, 0.0f, 1.0f },
    { "IMU_FORWARD_AXIS",      &g_params.imu_forward_axis,    -1.0f,  2.0f     },
    { "IMU_FORWARD_SIGN",      &g_params.imu_forward_sign,    -1.0f,  1.0f     },
    { "ROBOT_HALF_WIDTH_MM",   &g_params.robot_half_width_mm, 40.0f,  200.0f   },
    { "ENC_A_EDGES_PER_REV",   &g_params.enc_a_edges_per_rev, 10.0f,  20000.0f },
    { "WHEEL_DIAMETER_MM",     &g_params.wheel_diameter_mm,   20.0f,  150.0f   },
    { "TRACK_WIDTH_MM",        &g_params.track_width_mm,      50.0f,  300.0f   },
    { "SPEED_FF_DEADBAND_PCT", &g_params.ff_deadband_pct,     0.0f,   90.0f    },
    { "SPEED_FF_MM_S_PER_PCT", &g_params.ff_mm_s_per_pct,     0.1f,   100.0f   },
    { "IR_WHITE_ADC",          &g_params.ir_white_adc,        0.0f,   4095.0f  },
    { "IR_BLACK_ADC",          &g_params.ir_black_adc,        0.0f,   4095.0f  },
    { "SPEED_KP",              &g_params.speed_kp,            0.0f,   0.1f     },
    { "SPEED_KI",              &g_params.speed_ki,            0.0f,   1.0f     },
    { "LINE_BASE_MM_S",        &g_params.line_base_mm_s,      30.0f,  (float)SPEED_MAX_MM_S },
    { "LINE_STEER",            &g_params.line_steer,          0.0f,   1.0f     },
    { "LINE_STEER_RAMP",       &g_params.line_steer_ramp,     0.0f,   20.0f    },
    { "LINE_HOLD_STEER",       &g_params.line_hold_steer,     0.0f,   1.0f     },
    { "LINE_HOLD_MM",          &g_params.line_hold_mm,        0.0f,   300.0f   },
    { "LINE_LOST_MM",          &g_params.line_lost_mm,        50.0f,  3000.0f  },
    { "BARCODE_AT_JUNCTION",   &g_params.barcode_at_junction, 0.0f,   1.0f     },
    { "TURN_MM_S",             &g_params.turn_mm_s,           30.0f,  (float)SPEED_MAX_MM_S },
    { "BYPASS_MM_S",           &g_params.bypass_mm_s,         30.0f,  (float)SPEED_MAX_MM_S },
    { "OBSTACLE_STOP_CM",      &g_params.obstacle_stop_cm,    10.0f,  60.0f    },
    { "OBST_MARGIN_MM",        &g_params.obst_margin_mm,      0.0f,   300.0f   },
    { "OBST_DEPTH_MM",         &g_params.obst_depth_mm,       20.0f,  600.0f   },
    { "IR_TO_AXLE_MM",         &g_params.ir_to_axle_mm,       0.0f,   200.0f   },
};

#define PARAM_COUNT (sizeof(PARAMS) / sizeof(PARAMS[0]))

void params_init(void)
{
    g_params.motor_mapping_confirmed  = (float)MOTOR_MAPPING_CONFIRMED;
    g_params.enc_r_reversed           = (float)ENC_R_REVERSED;
    g_params.enc_l_reversed           = (float)ENC_L_REVERSED;
    g_params.enc_direction_confirmed  = (float)ENC_DIRECTION_CONFIRMED;
    g_params.line_do_black_high       = (float)LINE_DO_BLACK_HIGH;
    g_params.servo_min_us             = (float)SERVO_MIN_US;
    g_params.servo_max_us             = (float)SERVO_MAX_US;
    g_params.servo_calibrated         = (float)SERVO_CALIBRATED;
    g_params.servo_low_angle_is_right = (float)SERVO_LOW_ANGLE_IS_RIGHT;
    g_params.imu_forward_axis         = (float)IMU_FORWARD_AXIS;
    g_params.imu_forward_sign         = (float)IMU_FORWARD_SIGN;
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
    g_params.barcode_at_junction = (float)BARCODE_AT_JUNCTION;
    g_params.turn_mm_s           = (float)TURN_MM_S;
    g_params.bypass_mm_s         = (float)BYPASS_MM_S;
    g_params.obstacle_stop_cm    = (float)OBSTACLE_STOP_CM;
    g_params.obst_margin_mm      = (float)OBST_MARGIN_MM;
    g_params.obst_depth_mm       = (float)OBST_DEPTH_MM;
    g_params.ir_to_axle_mm       = (float)IR_TO_AXLE_MM;
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
            *PARAMS[i].p_value = value;
            return 0;
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
        jb_fixed(&jb, *PARAMS[i].p_value, 4u);
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
