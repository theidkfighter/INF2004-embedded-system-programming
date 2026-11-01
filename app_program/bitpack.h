/**
 * @file    bitpack.h
 * @brief   Packed_Field helpers: mask/shift get and set, range-checked
 *          enum/flag writes, saturating quantizers and saturating counters
 *          (design C1; Requirements 1.1, 1.3, 1.4, 1.8, 1.9, 2.12).
 *
 * Header-only and plain (Requirement 9.5): it includes only C standard
 * headers, so the host test suite and every firmware module can use it.
 *
 * Field layout: a Packed_Field of width w (1..32) at bit shift sh inside a
 * 32-bit Storage_Unit u occupies bits sh .. sh + w - 1. Callers keep
 * sh + w <= 32.
 *
 * Continuous fields: code = value * scale, rounded to the nearest integer
 * with exact halves away from zero, then clamped to [lo_code, hi_code].
 * The product is formed in double: a float has a 24-bit mantissa and every
 * scale is a small integer (1, 10, 100, 10000), so (double)x * scale is
 * exact and the rounding sees the true value. Codes reach 2^28 (odometry
 * distance), which float could not hold exactly. bp_quant is called from
 * task context only, never from an interrupt handler.
 *
 * All float-to-double conversions are explicit casts and all integer
 * literals carry the u suffix, so -Wdouble-promotion stays quiet.
 */
#ifndef BITPACK_H
#define BITPACK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- mask / shift -------------------------------------------------------- */

/* Mask of the low w bits (w = 0 .. 32). */
#define BP_MASK(w)              (((w) >= 32u) ? 0xFFFFFFFFu : ((1u << (w)) - 1u))
/* Field of width w at shift sh in unit u. */
#define BP_GET(u, sh, w)        (((uint32_t)(u) >> (sh)) & BP_MASK(w))
/* Unit u with the field of width w at shift sh replaced by the low w bits of v;
   every other bit of u is unchanged. */
#define BP_PUT(u, sh, w, v)     (((uint32_t)(u) & ~(BP_MASK(w) << (sh))) | \
                                 (((uint32_t)(v) & BP_MASK(w)) << (sh)))
/* Compile-time check; m names the affected field or type (Requirement 2.13, 1.7). */
#define BP_STATIC_ASSERT(c, m)  _Static_assert((c), m)

/* ---- two's complement ---------------------------------------------------- */

/* Sign-extends the w-bit two's complement code (w = 1 .. 32) to int32_t.
   Bits of code above w are ignored. w = 0 returns 0. */
static inline int32_t bp_sext(uint32_t code, uint32_t w)
{
    uint32_t m;
    uint32_t c;

    if (w == 0u)
    {
        return 0;
    }
    m = BP_MASK(w);
    c = code & m;
    if ((c >> (w - 1u)) != 0u)              /* sign bit set: value = -(~c & m) - 1 */
    {
        return -(int32_t)((~c) & m) - 1;
    }
    return (int32_t)c;
}

/* The w-bit two's complement code of v (w = 0 .. 32). The caller keeps v in
   [-2^(w-1), 2^(w-1) - 1]; other values wrap modulo 2^w. */
static inline uint32_t bp_tc(int32_t v, uint32_t w)
{
    return (uint32_t)v & BP_MASK(w);
}

/* ---- range-checked enum / flag write (Requirement 2.12) ------------------ */

/* Writes v into the field (sh, w) of *p_unit and returns true when
   lo <= v <= hi and v fits in w bits. Otherwise returns false and leaves
   *p_unit unchanged. Every other field of the unit is kept. The
   read-modify-write is not atomic: a unit shared between contexts is
   written inside a Critical_Section by the caller. */
static inline bool bp_set_checked(volatile uint32_t *p_unit, uint32_t sh, uint32_t w,
                                  uint32_t v, uint32_t lo, uint32_t hi)
{
    if ((v < lo) || (v > hi) || (v > BP_MASK(w)))
    {
        return false;
    }
    *p_unit = BP_PUT(*p_unit, sh, w, v);
    return true;
}

/* ---- continuous quantizer (Requirements 1.4, 1.8, 1.9) ------------------- */

/* scale:    codes per unit of the value (Resolution = 1 / scale), >= 1
   lo_code:  code of the lower non-sentinel limit of the Valid_Range
   hi_code:  code of the upper non-sentinel limit (hi_code >= lo_code)
   nan_code: code stored for NaN: the sentinel when the field has one,
             otherwise lo_code */
typedef struct
{
    int32_t scale;
    int32_t lo_code;
    int32_t hi_code;
    int32_t nan_code;
} bp_qspec_t;

/* Returns round_half_away_from_zero((double)x * scale) clamped to
   [lo_code, hi_code], or nan_code when x is NaN. ±infinity and values beyond
   the int32 range clamp to the matching limit. When p_sat is not NULL,
   *p_sat is set to true when the result was clamped or x was NaN, and to
   false otherwise. */
static inline int32_t bp_quant(float x, const bp_qspec_t *p_q, bool *p_sat)
{
    double  d;
    double  a;
    int64_t code;
    bool    b_sat = false;
    int32_t out;

    if (x != x)                             /* NaN */
    {
        b_sat = true;
        out   = p_q->nan_code;
    }
    else
    {
        d = (double)x * (double)p_q->scale;
        if (d > 2147483647.0)               /* above every int32 code (also +inf) */
        {
            code = (int64_t)p_q->hi_code + 1;
        }
        else if (d < -2147483648.0)         /* below every int32 code (also -inf) */
        {
            code = (int64_t)p_q->lo_code - 1;
        }
        else
        {
            /* |d| <= 2^31: d + 0.5 is exact and fits int64, and the
               conversion truncates, which is floor for a + 0.5 >= 0. */
            a    = (d < 0.0) ? -d : d;
            code = (int64_t)(a + 0.5);
            if (d < 0.0)
            {
                code = -code;
            }
        }
        if (code > (int64_t)p_q->hi_code)
        {
            b_sat = true;
            out   = p_q->hi_code;
        }
        else if (code < (int64_t)p_q->lo_code)
        {
            b_sat = true;
            out   = p_q->lo_code;
        }
        else
        {
            out = (int32_t)code;
        }
    }
    if (p_sat != NULL)
    {
        *p_sat = b_sat;
    }
    return out;
}

/* The value of a continuous code: code / scale (scale >= 1). */
static inline float bp_dequant(int32_t code, int32_t scale)
{
    return (float)((double)code / (double)scale);
}

/* ---- saturating counter ------------------------------------------------- */

/* v + 1, holding at max instead of wrapping (v >= max returns max). */
static inline uint32_t bp_sat_inc(uint32_t v, uint32_t max)
{
    return (v >= max) ? max : (v + 1u);
}

#endif /* BITPACK_H */
