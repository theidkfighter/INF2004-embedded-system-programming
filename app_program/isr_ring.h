/**
 * @file    isr_ring.h
 * @brief   Index arithmetic and entry layouts of the two rings the 1 kHz timer
 *          ISR fills (design D4; Requirements 5.4, 5.5, 10.4, 10.8).
 *
 * Plain C header (only C standard headers and bitpack.h), so hal_isr.c and the
 * host tests use the same code.
 *
 * A ring of n = 2^k entries keeps head and tail modulo 2n, in k + 1 bits, so a
 * full ring (difference n) differs from an empty one (difference 0). The slot
 * of an index is its low k bits. The stored indices are narrower than a C
 * int, so every difference goes through ring_used(), which masks the result.
 */
#ifndef ISR_RING_H
#define ISR_RING_H

#include <stdbool.h>
#include <stdint.h>

#include "bitpack.h"

/* Entries held in a ring, from its head and tail indices. */
static inline uint32_t ring_used(uint32_t head, uint32_t tail, uint32_t idx_mask)
{
    return (head - tail) & idx_mask;
}

/* ---- barcode edge ring: 64 entries, M7 indices in 8-bit units ------------- */
#define BC_RING          (64u)          /* power of 2 */
#define BC_IDX_MASK      ((2u * BC_RING) - 1u)
#define BC_SLOT_MASK     (BC_RING - 1u)

/* Edge slot (one word, one store by the ISR): bit 31 black, bits 30..0 the
   odometer's low 31 bits. */
#define BC_EDGE_BLACK    (1u << 31)
#define BC_ODO_MASK      (0x7FFFFFFFu)

static inline uint32_t bc_edge_pack(uint32_t odo, bool b_black)
{
    return (odo & BC_ODO_MASK) | (b_black ? BC_EDGE_BLACK : 0u);
}

static inline bool bc_edge_black(uint32_t slot)
{
    return (slot & BC_EDGE_BLACK) != 0u;
}

/* The full odometer value of an edge, with bit 31 restored from the live
   odometer: exact while the edge is less than 2^31 counts old. */
static inline uint32_t bc_edge_odo(uint32_t slot, uint32_t now)
{
    return now - ((now - slot) & BC_ODO_MASK);
}

/* ---- raw sample ring: 256 entries, M9 indices in 16-bit units ------------- */
#define RAW_RING         (256u)         /* power of 2: 256 ms of samples */
#define RAW_IDX_MASK     ((2u * RAW_RING) - 1u)
#define RAW_SLOT_MASK    (RAW_RING - 1u)

/* Raw sample slot (one word, one store by the ISR, was 10 bytes in three
   arrays): ADC U12 @0, right count low 9 bits @12, left count low 9 bits @21
   (direction flags applied), gap U1 @30 (samples were dropped just before this
   one because the ring was full), bit 31 unused. The reader rebuilds the full
   counts from the modulo-512 difference to the previous sample: a wheel moves
   a few counts per 1 ms sample, far inside +-255. After a gap that difference
   is not trustworthy, so the reader counts no movement for that one sample
   (the pass is already spoiled by the lost samples, and
   hal_barcode_overruns() reports it). */
#define RAW_ADC_MASK     (0xFFFu)
#define RAW_ER_SHIFT     (12u)
#define RAW_EL_SHIFT     (21u)
#define RAW_CNT_BITS     (9u)
#define RAW_CNT_MASK     ((1u << RAW_CNT_BITS) - 1u)
#define RAW_GAP          (1u << 30)

static inline uint32_t raw_slot_pack(uint32_t adc, uint32_t er, uint32_t el, bool b_gap)
{
    return (adc & RAW_ADC_MASK)
         | ((er & RAW_CNT_MASK) << RAW_ER_SHIFT)
         | ((el & RAW_CNT_MASK) << RAW_EL_SHIFT)
         | (b_gap ? RAW_GAP : 0u);
}

/* Signed modulo-512 step from the previous low part to a stored low part. */
static inline int32_t raw_delta(uint32_t low, uint32_t prev)
{
    uint32_t d = (low - prev) & RAW_CNT_MASK;

    return ((d >> (RAW_CNT_BITS - 1u)) != 0u) ? ((int32_t)d - (int32_t)(1u << RAW_CNT_BITS)) : (int32_t)d;
}

BP_STATIC_ASSERT((BC_RING & (BC_RING - 1u)) == 0u, "BC_RING: power of 2");
BP_STATIC_ASSERT((RAW_RING & (RAW_RING - 1u)) == 0u, "RAW_RING: power of 2");
BP_STATIC_ASSERT(BC_IDX_MASK <= 0xFFu, "g_bc_head, g_bc_tail: M7 fits in 8 bits");
BP_STATIC_ASSERT(RAW_IDX_MASK <= 0xFFFFu, "g_raw_head, g_raw_tail: M9 fits in 16 bits");

#endif /* ISR_RING_H */
