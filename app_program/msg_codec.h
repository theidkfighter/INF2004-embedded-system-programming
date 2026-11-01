/**
 * @file    msg_codec.h
 * @brief   Inter-task messages packed to their minimum bits (Change Requirement 1, D6).
 *
 * The tasks keep building and reading the unpacked forms (motion_cmd_t,
 * nav_event_t, obstacle_profile_t in app_types.h). post_motion(), post_nav()
 * and post_obstacle() pack them into these mailbox bodies; the receiver
 * unpacks. A message block is owned by one task at a time (sender until
 * tk_snd_mbx, receiver after tk_rcv_mbx), so no Critical_Section is needed.
 *
 * motion_msg_t (3 words, was motion_cmd_t 32 bytes), bit positions from
 * bit 0 of w[0]:
 *   type E3 @0, src E2 @3, id U16 @5, then from bit 21 by type:
 *   MC_LINE         left S14, right S14            (0.1 mm/s, +-819.1: line_wheels() can
 *                   ask up to 2 x LINE_BASE_MM_S before the hump scale and the
 *                   SPEED_MAX_MM_S clamp in TSK_MOTION_CTRL, so it is not clipped here)
 *   MC_TURN         turn_deg S13 (0.1 deg, +-360.0), speed S13, timeout U19 (ms)
 *   MC_BYPASS       dist S16 (0.1 mm, +-3000.0), speed S13, timeout U19
 *   MC_SPEED_LIMIT  scale U7 (0.01, 0..1.00)
 *   MC_STOP         nothing
 * nav_msg_t (1 byte, was 2): type E3 @0, letter U5 @3 (0 = 'A'; 'A'..'Z' only)
 * obst_msg_t (6 words, was obstacle_profile_t 64 bytes):
 *   valid U1, side E2 (0 none, 1 L, 2 R), r_min U13 (0.1 cm, 8191 = none),
 *   theta_l S11, theta_r S11 (0.1 deg), x_l S13, x_r S13, width U13,
 *   offset S13, clear_l S13, clear_r S13 (0.1 cm, +-400.0), scan U13 x 5
 *   = 181 bits.
 * Values outside a range are clamped (bp_quant); the receiver never sees a
 * wrapped value.
 */
#ifndef MSG_CODEC_H
#define MSG_CODEC_H

#include <stdint.h>

#include "app_types.h"

void msg_motion_pack(const motion_cmd_t *p_c, motion_msg_t *p_m);
void msg_motion_unpack(const motion_msg_t *p_m, motion_cmd_t *p_c);
void msg_nav_pack(const nav_event_t *p_e, nav_msg_t *p_m);
void msg_nav_unpack(const nav_msg_t *p_m, nav_event_t *p_e);
void msg_obst_pack(const obstacle_profile_t *p_o, obst_msg_t *p_m);
void msg_obst_unpack(const obst_msg_t *p_m, obstacle_profile_t *p_o);

#endif /* MSG_CODEC_H */
