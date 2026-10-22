/**
 * @file    app_types.h
 * @brief   Message and status types shared by the IS-02 tasks (DR Section 4).
 *
 * Plain C types only: this header is included by micro T-Kernel task code
 * and by Pico SDK driver code, which cannot share each other's headers.
 */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* ---- System states (DR Section 5) ---- */
typedef enum
{
    SYS_INIT = 0,
    SYS_IDLE,
    SYS_LINE_FOLLOWING,
    SYS_MANEUVER,              /* barcode turn in progress (part of LINE FOLLOWING in the DR) */
    SYS_OBSTACLE_AVOIDANCE,
    SYS_MISSION_COMPLETE,
    SYS_ESTOP,
    SYS_ERROR,
    SYS_STATE_COUNT
} sys_state_t;

const char *sys_state_name(sys_state_t state);

/* ---- MBX_MOTION_CMD: MotionCmd_t ---- */
typedef enum
{
    MC_LINE = 1,               /* wheel speeds from line following / line search */
    MC_TURN,                   /* encoder-counted pivot turn */
    MC_BYPASS,                 /* encoder-counted straight leg (forward or back) */
    MC_SPEED_LIMIT,            /* scale all LINE speeds (hump: 60 %) */
    MC_STOP                    /* brake and cancel the active primitive */
} motion_cmd_type_t;

typedef enum
{
    SRC_LINE = 1,
    SRC_OBSTACLE,
    SRC_NAV,
    SRC_SAFETY
} cmd_src_t;

typedef struct
{
    uint8_t  type;             /* motion_cmd_type_t */
    uint8_t  src;              /* cmd_src_t */
    uint16_t id;               /* echoed in the done report */
    float    left_mm_s;        /* MC_LINE */
    float    right_mm_s;       /* MC_LINE */
    float    turn_deg;         /* MC_TURN: + = clockwise (to the right) */
    float    dist_mm;          /* MC_BYPASS: + forward, - backward */
    float    speed_mm_s;       /* MC_TURN / MC_BYPASS wheel speed */
    float    scale;            /* MC_SPEED_LIMIT: 0..1 */
    uint32_t timeout_ms;       /* MC_TURN / MC_BYPASS: encoder fault if exceeded */
} motion_cmd_t;

/* Result of the last TURN/BYPASS, read after EVF_MOTION_DONE */
typedef enum
{
    MOTION_OK = 0,
    MOTION_STOPPED,            /* cancelled by MC_STOP */
    MOTION_TIMEOUT             /* encoder fault assumed */
} motion_result_t;

/* ---- MBX_NAV_EVENT: NavEvent_t ---- */
typedef enum
{
    NAV_BARCODE = 1,           /* decoded on the track: code = 'A'..'D' */
    NAV_REMOTE_BARCODE,        /* FR19 override from MQTT */
    NAV_BARCODE_FAULT,         /* 3 ambiguous reads in a row */
    NAV_JUNCTION,              /* both line sensors black */
    NAV_BTN1,
    NAV_START,                 /* remote START (same as BTN1 in IDLE) */
    NAV_STOP                   /* remote STOP (end the run) */
} nav_event_type_t;

typedef struct
{
    uint8_t type;              /* nav_event_type_t */
    char    code;              /* barcode letter */
} nav_event_t;

/* ---- MBX_OBSTACLE: ObstacleProfile_t (DR 6.2 STAGE3) ---- */
typedef struct
{
    bool    b_valid;
    float   r_min_cm;          /* closest point */
    float   theta_l_deg;       /* left edge angle  (- = left of straight ahead) */
    float   theta_r_deg;       /* right edge angle (+ = right) */
    float   x_l_cm;            /* left edge, lateral position (- = left) */
    float   x_r_cm;            /* right edge, lateral position */
    float   width_cm;
    float   offset_cm;         /* obstacle centre, lateral */
    float   clear_left_cm;     /* free space to the left of the obstacle */
    float   clear_right_cm;
    char    side;              /* chosen bypass side 'L' / 'R', 0 = none */
    float   scan_cm[5];        /* coarse sweep -60..+60 deg, < 0 = no echo */
} obstacle_profile_t;

/* ---- MBX_TELEMETRY: events published on is02/event ---- */
#define TLM_EVENT_NAME_LEN   (20u)
#define TLM_EVENT_DETAIL_LEN (72u)
typedef struct
{
    char name[TLM_EVENT_NAME_LEN];
    char detail[TLM_EVENT_DETAIL_LEN];
} tlm_event_t;

/* One mailbox message body (the kernel header is added in os_glue.h) */
typedef union
{
    motion_cmd_t       motion;
    nav_event_t        nav;
    obstacle_profile_t obstacle;
    tlm_event_t        event;
} app_msg_body_t;

/* ---- Motion events (FR24, DR 6.3 region B) ---- */
typedef enum
{
    MEV_STATIONARY = 0,
    MEV_CRUISING,
    MEV_ACCELERATING,
    MEV_DECELERATING,
    MEV_TURNING,
    MEV_CLIMBING,
    MEV_DESCENDING,
    MEV_IMPACT,
    MEV_COUNT
} motion_event_t;

const char *motion_event_name(motion_event_t ev);

/* ---- Fault bits (NFR19), reported in telemetry ---- */
#define FAULT_IMU_I2C        (1u << 0)
#define FAULT_IMU_DEAD       (1u << 1)
#define FAULT_ENCODER_LOSS   (1u << 2)
#define FAULT_MOTION_TIMEOUT (1u << 3)
#define FAULT_ULTRASONIC     (1u << 4)
#define FAULT_LINE_LOST      (1u << 5)
#define FAULT_BYPASS_ABORT   (1u << 6)
#define FAULT_WATCHDOG       (1u << 7)
#define FAULT_BATTERY_LOW    (1u << 8)
#define FAULT_BATTERY_CRIT   (1u << 9)
#define FAULT_CONFIG         (1u << 10)
#define FAULT_BARCODE        (1u << 11)
#define FAULT_TELEMETRY      (1u << 12)

/* ---- Robot snapshot for telemetry ----
 * Each group is written by one task only (owner in brackets) and read by the
 * telemetry task. Single 32-bit fields are atomic on the RP2040; a frame may
 * mix values from two consecutive 20 ms cycles, which is fine for 2 Hz
 * telemetry. Control decisions never read this struct. */
typedef struct
{
    volatile uint32_t state;           /* sys_state_t      [nav / safety] */
    volatile uint32_t faults;          /* FAULT_* bits     [any, set only] */
    /* [motion] */
    volatile float    spd_l;           /* mm/s */
    volatile float    spd_r;
    volatile float    duty_l;          /* -1..+1 */
    volatile float    duty_r;
    volatile int32_t  enc_l;
    volatile int32_t  enc_r;
    volatile float    dist_mm;         /* distance travelled (mean of both wheels, absolute) */
    volatile float    speed_scale;
    /* [line] */
    volatile int32_t  line_l;          /* 1 black, 0 white */
    volatile int32_t  line_r;
    volatile uint32_t ir_adc;
    volatile uint32_t line_state;
    volatile char     barcode;         /* last decoded, 0 = none */
    volatile uint32_t barcodes_read;
    /* [imu] */
    volatile float    pitch_deg;
    volatile float    hump_mm;
    volatile float    hump_peak_mm;
    volatile float    hump_max_mm;     /* FR22: highest peak this run */
    volatile uint32_t humps;
    volatile uint32_t motion_event;    /* motion_event_t */
    volatile float    turn_rate_dps;
    /* [obstacle] */
    volatile float    obst_cm;         /* last forward reading, < 0 = no echo */
    volatile float    scan_cm[5];
    volatile char     scan_side;
    volatile uint32_t obstacles;
    /* [telemetry] */
    volatile uint32_t vbat_mv;
} robot_status_t;

extern robot_status_t g_status;

#endif /* APP_TYPES_H */
