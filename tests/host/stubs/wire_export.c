/**
 * @file    wire_export.c
 * @brief   Host_Test_Suite only: struct sizes and offsets of the Wire_Codec
 *          types (app_program/wire.h), so py/wire_ctypes.py can check that its
 *          ctypes Structures match the C layout before using them.
 */
#include <stddef.h>

#include "wire.h"

/* kind: 0 wire_tlm_t, 1 wire_hb_t, 2 wire_evt_t, 3 wire_ack_t, 4 wire_pval_t,
   5 bw_t, 6 br_t. Returns 0 for any other kind. */
uint32_t wirex_sizeof(uint32_t kind)
{
    switch (kind)
    {
        case 0u: return (uint32_t)sizeof(wire_tlm_t);
        case 1u: return (uint32_t)sizeof(wire_hb_t);
        case 2u: return (uint32_t)sizeof(wire_evt_t);
        case 3u: return (uint32_t)sizeof(wire_ack_t);
        case 4u: return (uint32_t)sizeof(wire_pval_t);
        case 5u: return (uint32_t)sizeof(bw_t);
        case 6u: return (uint32_t)sizeof(br_t);
        default: return 0u;
    }
}

/* Offsets of a few members spread over each struct (spot check of the
   ctypes field order): 0 tlm.buf, 1 tlm.scan, 2 hb.ip, 3 ack.a,
   4 ack.a.getp.v, 5 ack.a.cfg.rej, 6 tlm.turn, 7 hb.uptime_s. */
uint32_t wirex_offsetof(uint32_t which)
{
    switch (which)
    {
        case 0u: return (uint32_t)offsetof(wire_tlm_t, buf);
        case 1u: return (uint32_t)offsetof(wire_tlm_t, scan);
        case 2u: return (uint32_t)offsetof(wire_hb_t, ip);
        case 3u: return (uint32_t)offsetof(wire_ack_t, a);
        case 4u: return (uint32_t)offsetof(wire_ack_t, a.getp.v);
        case 5u: return (uint32_t)offsetof(wire_ack_t, a.cfg.rej);
        case 6u: return (uint32_t)offsetof(wire_tlm_t, turn);
        case 7u: return (uint32_t)offsetof(wire_hb_t, uptime_s);
        default: return 0xFFFFFFFFu;
    }
}
