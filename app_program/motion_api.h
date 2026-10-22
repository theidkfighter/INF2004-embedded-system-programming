/**
 * @file    motion_api.h
 * @brief   Motion-control API for the other tasks (Buddy 2 deliverable:
 *          moveForward / moveBackward / turnLeft / turnRight / stop).
 *
 * Every call posts a MotionCmd_t to MBX_MOTION_CMD; TSK_MOTION_CTRL executes
 * it with encoder feedback and sets EVF_MOTION_DONE when a TURN or BYPASS
 * primitive finishes (DR Section 4). The blocking calls wait for that flag.
 *
 *   brief name            here
 *   moveForward(d)        motion_move_forward(d)
 *   moveBackward(d)       motion_move_backward(d)
 *   turnLeft(a)           motion_turn_left(a)
 *   turnRight(a)          motion_turn_right(a)
 *   stop()                motion_stop()
 */
#ifndef MOTION_API_H
#define MOTION_API_H

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"

/* blocking (return when the primitive is done, stopped or timed out) */
motion_result_t motion_move_forward(float dist_mm);
motion_result_t motion_move_backward(float dist_mm);
motion_result_t motion_turn_left(float deg);
motion_result_t motion_turn_right(float deg);
motion_result_t motion_turn(float deg_cw, float speed_mm_s);
motion_result_t motion_move(float dist_mm, float speed_mm_s);

/* non-blocking */
void     motion_stop(cmd_src_t src);
void     motion_speed_limit(float scale);                 /* 1.0 = full speed */
bool     motion_line(float left_mm_s, float right_mm_s);  /* line following / search */
uint16_t motion_start_move(float dist_mm, float speed_mm_s, cmd_src_t src);
uint16_t motion_start_turn(float deg_cw, float speed_mm_s, cmd_src_t src);

/* Waits for the primitive 'id' to finish, or for any of other_bits
   (EVF_*). Returns true when the primitive finished (*p_result valid);
   false if other_bits fired first (*p_other) or on timeout. */
bool     motion_wait(uint16_t id, uint32_t other_bits, uint32_t timeout_ms,
                     motion_result_t *p_result, uint32_t *p_other);

#endif /* MOTION_API_H */
