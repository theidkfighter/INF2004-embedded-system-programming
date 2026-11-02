/**
 * @file    status.h
 * @brief   Status_Snapshot: the shared robot status, packed (Change Requirement 1).
 *
 * The old robot_status_t held 31 separate 32-bit fields (floats and
 * uint32_t, 156 bytes). Each field now uses only the bits its valid range
 * needs, packed into 32-bit Storage_Units (robot_status_t in app_types.h):
 *
 *   unit   bits    field (resolution, range)                         writer
 *   sys    0..2    state E3 (sys_state_t)                            nav / safety
 *          3..18   faults U16 (Fault_Mask, fault_mask.h)             any task
 *          19..22  stop reason E4 (stop_reason_t)                    safety
 *          23..25  motion event E3 (motion_event_t)                  imu
 *   mot_a  0..14   spd_l S15 (0.1 mm/s, +-1000.0)                    motion
 *          15..29  spd_r S15
 *   mot_b  0..10   duty_l S11 (0.1 %, +-100.0)                       motion
 *          11..21  duty_r S11
 *          22..28  speed_scale U7 (0.01, 0..1.00)
 *   mot_c  0..23   enc_l S24 (counts)                                motion
 *   mot_d  0..23   enc_r S24
 *   mot_e  0..19   dist U20 (0.1 mm, 0..100 m)                       motion
 *   line_a 0       line_l U1, 1 line_r U1                            line
 *          2..13   ir U12 (ADC)
 *          14..16  line_state E3 (line_state_t)
 *          17..22  barcode U6 (ASCII - 32, 0 = none)                 line (nav resets)
 *          23      bc_enabled U1
 *          24..30  barcodes_read U7 (saturates at 127)               line (nav resets)
 *   line_b 0..11   bc_thr_black U12, 12..23 bc_thr_white U12         line
 *   imu_a  0..10   pitch S11 (0.1 deg, +-90.0)                       imu
 *          11..25  hump_h S15 (0.1 mm, +-1500.0)
 *   imu_b  0..13   hump_peak U14 (0.1 mm, 0..1500.0)                 imu
 *          14..27  hump_max U14                                      imu (nav resets)
 *   imu_c  0..13   turn S14 (0.1 deg/s, +-720.0)                     imu
 *          14..19  humps U6 (saturates at 63)                        imu (nav resets)
 *   obs_a  0..12   obst_cm U13 (0.1 cm, 0..431.0, 8191 = no echo)    obstacle
 *          13..14  scan_side U2 (0 none, 1 L, 2 R)
 *          15..27  scan[0] U13
 *   obs_b  0..12   scan[1], 13..25 scan[2]                           obstacle
 *   obs_c  0..12   scan[3], 13..25 scan[4]                           obstacle
 *          26..31  obstacles U6 (saturates at 63)                    obstacle (nav resets)
 *   misc   0..12   vbat U13 (mV)                                     telemetry
 *
 * Access rules (no race on a shared unit):
 *   - every write is a read-modify-write of one unit inside a
 *     dispatch-disabled Critical_Section (crit_dsp_enter/exit), because
 *     several units have fields written by two tasks (for example the run
 *     counters, which TSK_NAV_PLANNER resets). No interrupt handler writes
 *     the snapshot, so masking interrupts is not needed;
 *   - a reader loads a unit with one 32-bit load (atomic on the RP2040) and
 *     extracts the field, so it never sees half of an update;
 *   - continuous values are quantized on write (round half away from zero,
 *     clamped, NaN -> the sentinel or lower limit, sat_note() counts it).
 * Control decisions keep their own float values: the snapshot is for
 * telemetry and for the few cross-task reads listed below.
 */
#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "bitpack.h"
#include "line_logic.h"
#include "wire_ids.h"

/* ---- writers ---------------------------------------------------------------- */
void st_set_state(sys_state_t state);
void st_fault_set(uint32_t bits);
void st_fault_clear(uint32_t bits);
void st_set_stop(uint32_t stop_reason);
void st_set_motion_event(motion_event_t ev);

void st_set_motion(float spd_l, float spd_r, float duty_l, float duty_r,
                   int32_t enc_l, int32_t enc_r, float dist_mm, float speed_scale);

void st_set_line(bool b_l, bool b_r, uint32_t ir_adc, uint32_t line_state);
void st_set_bc_levels(uint32_t thr_black, uint32_t thr_white, bool b_enabled);
void st_barcode_read(char code);                 /* last code + count */

void st_set_imu(float pitch_deg, float hump_mm, float turn_dps);
void st_set_hump_peak(float peak_mm);
void st_hump_done(float height_mm);              /* humps + 1, run max, peak = height */

void st_set_obst_cm(float cm);                   /* < 0 = no echo */
void st_set_scan(uint32_t i, float cm);          /* i = 0..4 */
void st_set_scan_side(char side);                /* 'L', 'R' or 0 */
void st_obstacle_done(void);                     /* obstacles + 1 */

void st_set_vbat(uint32_t mv);
void st_run_reset(void);                         /* start of a run: counters, maxima, last code */

/* ---- readers ---------------------------------------------------------------- */
sys_state_t    st_state(void);
uint32_t       st_faults(void);
uint32_t       st_stop(void);
motion_event_t st_motion_event(void);
float          st_dist_mm(void);
bool           st_line_l(void);
bool           st_line_r(void);
uint32_t       st_ir_adc(void);
uint32_t       st_bc_thr_black(void);
uint32_t       st_bc_thr_white(void);
bool           st_bc_enabled(void);
char           st_barcode(void);
uint32_t       st_barcodes_read(void);
float          st_pitch_deg(void);
float          st_hump_max_mm(void);
uint32_t       st_humps(void);
float          st_obst_cm(void);                 /* -1 = no echo */
float          st_scan_cm(uint32_t i);           /* -1 = no echo */
char           st_scan_side(void);
uint32_t       st_obstacles(void);
uint32_t       st_vbat(void);

/* ---- field layout (also used by the telemetry frame fill) ---------------------- */
/* sys */
#define ST_SH_STATE      (0u)
#define ST_W_STATE       (3u)
#define ST_SH_FAULTS     (3u)
#define ST_W_FAULTS      (16u)     /* Fault_Mask, design D1 */
#define ST_SH_STOP       (19u)
#define ST_W_STOP        (4u)
#define ST_SH_MEV        (23u)
#define ST_W_MEV         (3u)
/* mot_a */
#define ST_SH_SPD_L      (0u)
#define ST_SH_SPD_R      (15u)
#define ST_W_SPD         (15u)
/* mot_b */
#define ST_SH_DUTY_L     (0u)
#define ST_SH_DUTY_R     (11u)
#define ST_W_DUTY        (11u)
#define ST_SH_SCALE      (22u)
#define ST_W_SCALE       (7u)
/* mot_c, mot_d */
#define ST_SH_ENC        (0u)
#define ST_W_ENC         (24u)
/* mot_e */
#define ST_SH_DIST       (0u)
#define ST_W_DIST        (20u)
/* line_a */
#define ST_SH_LINE_L     (0u)
#define ST_SH_LINE_R     (1u)
#define ST_W_LINE        (1u)
#define ST_SH_IR         (2u)
#define ST_W_IR          (12u)
#define ST_SH_LINE_STATE (14u)
#define ST_W_LINE_STATE  (3u)
#define ST_SH_BARCODE    (17u)
#define ST_W_BARCODE     (6u)
#define ST_SH_BC_ON      (23u)
#define ST_W_BC_ON       (1u)
#define ST_SH_BC_N       (24u)
#define ST_W_BC_N        (7u)
/* line_b */
#define ST_SH_THR_BLACK  (0u)
#define ST_SH_THR_WHITE  (12u)
#define ST_W_THR         (12u)
/* imu_a */
#define ST_SH_PITCH      (0u)
#define ST_W_PITCH       (11u)
#define ST_SH_HUMP_H     (11u)
#define ST_W_HUMP_H      (15u)
/* imu_b */
#define ST_SH_HUMP_PEAK  (0u)
#define ST_SH_HUMP_MAX   (14u)
#define ST_W_HUMP_U      (14u)
/* imu_c */
#define ST_SH_TURN       (0u)
#define ST_W_TURN        (14u)
#define ST_SH_HUMPS      (14u)
#define ST_W_HUMPS       (6u)
/* obs_a */
#define ST_SH_OBST       (0u)
#define ST_W_US          (13u)     /* every ultrasonic reading */
#define ST_SH_SCAN_SIDE  (13u)
#define ST_W_SCAN_SIDE   (2u)
#define ST_SH_SCAN0      (15u)
/* obs_b */
#define ST_SH_SCAN1      (0u)
#define ST_SH_SCAN2      (13u)
/* obs_c */
#define ST_SH_SCAN3      (0u)
#define ST_SH_SCAN4      (13u)
#define ST_SH_OBSTACLES  (26u)
#define ST_W_OBSTACLES   (6u)
/* misc */
#define ST_SH_VBAT       (0u)
#define ST_W_VBAT        (13u)

#define ST_US_NONE       (8191u)

/* ---- compile-time checks (Requirement 2.13); test_compile_fail.sh ------------- */
#define ST_UNIT_BITS     (32u)
#define ST_FITS(sh, w)   (((sh) + (w)) <= ST_UNIT_BITS)

BP_STATIC_ASSERT(sizeof(robot_status_t) == 60u, "status: robot_status_t must be 60 bytes");

BP_STATIC_ASSERT((uint32_t)SYS_STATE_COUNT <= (1u << ST_W_STATE), "status.sys.state too narrow");
BP_STATIC_ASSERT((uint32_t)STOP_REASON_COUNT <= (1u << ST_W_STOP), "status.sys.stop_reason too narrow");
BP_STATIC_ASSERT((uint32_t)MEV_COUNT <= (1u << ST_W_MEV), "status.sys.mev too narrow");
BP_STATIC_ASSERT((uint32_t)LINE_LOST < (1u << ST_W_LINE_STATE), "status.line_a.line_state too narrow");

BP_STATIC_ASSERT(ST_FITS(ST_SH_STATE, ST_W_STATE), "status.sys.state does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_FAULTS, ST_W_FAULTS), "status.sys.faults does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_STOP, ST_W_STOP), "status.sys.stop_reason does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_MEV, ST_W_MEV), "status.sys.mev does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SPD_L, ST_W_SPD), "status.mot_a.spd_l does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SPD_R, ST_W_SPD), "status.mot_a.spd_r does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_DUTY_L, ST_W_DUTY), "status.mot_b.duty_l does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_DUTY_R, ST_W_DUTY), "status.mot_b.duty_r does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCALE, ST_W_SCALE), "status.mot_b.speed_scale does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_ENC, ST_W_ENC), "status.mot_c.enc_l, mot_d.enc_r do not fit their units");
BP_STATIC_ASSERT(ST_FITS(ST_SH_DIST, ST_W_DIST), "status.mot_e.dist does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_LINE_L, ST_W_LINE), "status.line_a.line_l does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_LINE_R, ST_W_LINE), "status.line_a.line_r does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_IR, ST_W_IR), "status.line_a.ir does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_LINE_STATE, ST_W_LINE_STATE), "status.line_a.line_state does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_BARCODE, ST_W_BARCODE), "status.line_a.barcode does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_BC_ON, ST_W_BC_ON), "status.line_a.bc_enabled does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_BC_N, ST_W_BC_N), "status.line_a.barcodes_read does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_THR_BLACK, ST_W_THR), "status.line_b.bc_thr_black does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_THR_WHITE, ST_W_THR), "status.line_b.bc_thr_white does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_PITCH, ST_W_PITCH), "status.imu_a.pitch does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_HUMP_H, ST_W_HUMP_H), "status.imu_a.hump_h does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_HUMP_PEAK, ST_W_HUMP_U), "status.imu_b.hump_peak does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_HUMP_MAX, ST_W_HUMP_U), "status.imu_b.hump_max does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_TURN, ST_W_TURN), "status.imu_c.turn does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_HUMPS, ST_W_HUMPS), "status.imu_c.humps does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_OBST, ST_W_US), "status.obs_a.obst_cm does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN_SIDE, ST_W_SCAN_SIDE), "status.obs_a.scan_side does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN0, ST_W_US), "status.obs_a.scan0 does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN1, ST_W_US), "status.obs_b.scan1 does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN2, ST_W_US), "status.obs_b.scan2 does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN3, ST_W_US), "status.obs_c.scan3 does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_SCAN4, ST_W_US), "status.obs_c.scan4 does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_OBSTACLES, ST_W_OBSTACLES), "status.obs_c.obstacles does not fit its unit");
BP_STATIC_ASSERT(ST_FITS(ST_SH_VBAT, ST_W_VBAT), "status.misc.vbat does not fit its unit");

#endif /* STATUS_H */
