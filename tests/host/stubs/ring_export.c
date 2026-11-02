/**
 * @file    ring_export.c
 * @brief   Out-of-line wrappers of the app_program/isr_ring.h inline helpers,
 *          for ctypes (tasks 12.1, 12.2).
 */
#include "isr_ring.h"

uint32_t rx_ring_used(uint32_t head, uint32_t tail, uint32_t idx_mask) { return ring_used(head, tail, idx_mask); }
uint32_t rx_bc_edge_pack(uint32_t odo, bool b_black)                  { return bc_edge_pack(odo, b_black); }
bool     rx_bc_edge_black(uint32_t slot)                              { return bc_edge_black(slot); }
uint32_t rx_bc_edge_odo(uint32_t slot, uint32_t now)                  { return bc_edge_odo(slot, now); }
uint32_t rx_raw_slot_pack(uint32_t adc, uint32_t er, uint32_t el, bool b_gap) { return raw_slot_pack(adc, er, el, b_gap); }
int32_t  rx_raw_delta(uint32_t low, uint32_t prev)                    { return raw_delta(low, prev); }
