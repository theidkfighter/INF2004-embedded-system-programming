/**
 * @file    os_glue.h
 * @brief   IS-02 kernel objects (DR Section 4) and helpers. micro T-Kernel side.
 *
 * Mailboxes (tk_cre_mbx) carry MotionCmd_t, NavEvent_t, ObstacleProfile_t
 * and telemetry events; their message blocks come from one fixed-size
 * memory pool (the receiver releases the block). One event-flag group holds
 * the DR's EVF_* signals. MTX_I2C_BUS is a priority-inheritance mutex.
 */
#ifndef OS_GLUE_H
#define OS_GLUE_H

#include <tk/tkernel.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"

/* ---- task priorities (smaller number = higher priority) ----
 * DR Section 3 order, offset to fit around the template's services:
 * USB console service = 3 (template), CYW43 radio owner = 9 (robot.mk). */
#define PRI_SAFETY      (2)
#define PRI_MOTION      (4)
#define PRI_LINE        (5)
#define PRI_OBSTACLE    (6)
#define PRI_IMU         (7)
#define PRI_NAV         (8)
#define PRI_TELEMETRY   (10)

/* ---- EVF_* bits in g_flg_sys ---- */
#define EVF_COLLISION        (1u << 0)    /* IMU INT1 ISR or software backup */
#define EVF_PROXIMITY        (1u << 1)    /* ultrasonic < 8 cm */
#define EVF_REMOTE_ESTOP     (1u << 2)    /* MQTT ESTOP */
#define EVF_BATTERY_CRITICAL (1u << 3)
#define EVF_BUZZER_REQ       (1u << 4)
#define EVF_ESTOP            (1u << 5)    /* latched by the safety monitor */
#define EVF_MOTION_DONE      (1u << 6)
#define EVF_LINE_FOUND       (1u << 7)
#define EVF_LINE_LOST        (1u << 8)
#define EVF_OBSTACLE_TRIGGER (1u << 9)
#define EVF_BYPASS_COMPLETE  (1u << 10)
#define EVF_HUMP_START       (1u << 11)
#define EVF_HUMP_PEAK        (1u << 12)
#define EVF_HUMP_END         (1u << 13)
#define EVF_IMU_FAULT        (1u << 14)
#define EVF_BYPASS_ABORT     (1u << 15)
#define EVF_ERROR_REQ        (1u << 16)   /* any task: enter ERROR (fault bits say why) */
#define EVF_LOCAL_STOP       (1u << 17)   /* BTN2 */

/* ---- buzzer patterns (EVF_BUZZER_REQ) ---- */
#define BUZZ_NONE        (0u)
#define BUZZ_START       (1u)
#define BUZZ_BARCODE     (2u)
#define BUZZ_COMPLETE    (3u)
#define BUZZ_ERROR       (4u)
#define BUZZ_LOW_BATTERY (5u)
#define BUZZ_WARN        (6u)

/* ---- heartbeats watched by the safety monitor ---- */
#define HB_MOTION   (0u)
#define HB_LINE     (1u)
#define HB_IMU      (2u)
#define HB_COUNT    (3u)
extern volatile uint32_t g_hb[HB_COUNT];

/* ---- kernel objects ---- */
extern ID g_flg_sys;
extern ID g_mbx_motion;     /* MBX_MOTION_CMD */
extern ID g_mbx_nav;        /* MBX_NAV_EVENT  */
extern ID g_mbx_obstacle;   /* MBX_OBSTACLE   */
extern ID g_mbx_tlm;        /* MBX_TELEMETRY  */
extern ID g_mtx_i2c;        /* MTX_I2C_BUS    */
extern ID g_tsk_safety, g_tsk_motion, g_tsk_line, g_tsk_obstacle, g_tsk_imu, g_tsk_nav, g_tsk_tlm;

typedef struct
{
    T_MSG          hdr;
    app_msg_body_t body;
} app_msg_t;

/* Creates every object above. Returns false (and prints which) on failure. */
bool       os_create_objects(void);
bool       os_start_tasks(void);
bool       os_start_interrupts(void);            /* tk_def_int + EnableInt + 10 ms cyclic tick */

app_msg_t *msg_alloc(void);                      /* from the pool, no wait; NULL if empty */
void       msg_free(app_msg_t *p_msg);
app_msg_t *msg_recv(ID mbxid, TMO tmo);          /* NULL on timeout */

bool       post_motion(const motion_cmd_t *p_cmd);
bool       post_nav(nav_event_type_t type, char code);
bool       post_obstacle(const obstacle_profile_t *p_prof);
bool       post_event(const char *p_name, const char *p_detail);   /* is02/event */

void       flag_set(uint32_t bits);
void       flag_clear(uint32_t bits);
uint32_t   flag_peek(void);
bool       flag_take(uint32_t bits, uint32_t *p_got);              /* read + clear these bits */

void       buzzer_request(uint32_t pattern);
uint32_t   buzzer_pattern_take(void);

/* System state (DR Section 5). Only the safety monitor may leave or enter
   EMERGENCY_STOP / ERROR; other callers are refused while either is active. */
bool       sys_set_state(sys_state_t state, bool b_safety);
sys_state_t sys_state(void);
void       fault_set(uint32_t fault_bits);       /* NFR19: reported in telemetry */
void       fault_clear(uint32_t fault_bits);
extern const char * volatile g_stop_reason;      /* why the last E-stop / ERROR happened */
extern volatile uint32_t g_reacq_seq;            /* bumped by the bypass: TSK_LINE re-arms its line watch */

/* Shared odometry (written by the motion task) */
typedef struct
{
    int32_t enc_r;
    int32_t enc_l;
    float   dist_r_mm;          /* signed, forward + */
    float   dist_l_mm;
    float   heading_deg;        /* from the encoders, + = clockwise */
    float   v_mm_s;             /* mean of both wheels */
    float   a_fwd_g;            /* forward acceleration from the encoder speed */
    float   turn_dps;
    uint32_t odo_um;            /* travelled distance, monotonic, micrometres */
} odometry_t;
void       odometry_get(odometry_t *p_out);
void       odometry_put(const odometry_t *p_in);

/* Pushes driver-held parameters (encoder directions, servo range) after a change */
void       params_apply(void);

/* Console line (one tm_printf per call, so lines from tasks never splice) */
void       log_line(const char *p_text);

#endif /* OS_GLUE_H */
