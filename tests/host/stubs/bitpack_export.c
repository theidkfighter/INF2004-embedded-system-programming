/**
 * @file    bitpack_export.c
 * @brief   Host_Test_Suite only: out-of-line wrappers around the static
 *          inline helpers of app_program/bitpack.h, so the Python tests can
 *          call them through ctypes (py/conftest.py).
 *
 * Each wrapper calls the bitpack.h helper or macro unchanged; the tests
 * check the firmware code, not a copy of it.
 */
#include "bitpack.h"
#include "host_stubs.h"

uint32_t bpx_mask(uint32_t w)
{
    return BP_MASK(w);
}

uint32_t bpx_get(uint32_t u, uint32_t sh, uint32_t w)
{
    return BP_GET(u, sh, w);
}

uint32_t bpx_put(uint32_t u, uint32_t sh, uint32_t w, uint32_t v)
{
    return BP_PUT(u, sh, w, v);
}

int32_t bpx_sext(uint32_t code, uint32_t w)
{
    return bp_sext(code, w);
}

uint32_t bpx_tc(int32_t v, uint32_t w)
{
    return bp_tc(v, w);
}

bool bpx_set_checked(uint32_t *p_unit, uint32_t sh, uint32_t w,
                     uint32_t v, uint32_t lo, uint32_t hi)
{
    volatile uint32_t unit = *p_unit;
    bool b_ok = bp_set_checked(&unit, sh, w, v, lo, hi);

    *p_unit = unit;
    return b_ok;
}

int32_t bpx_quant(float x, int32_t scale, int32_t lo_code, int32_t hi_code,
                  int32_t nan_code, bool *p_sat)
{
    const bp_qspec_t q = { scale, lo_code, hi_code, nan_code };

    return bp_quant(x, &q, p_sat);
}

float bpx_dequant(int32_t code, int32_t scale)
{
    return bp_dequant(code, scale);
}

uint32_t bpx_sat_inc(uint32_t v, uint32_t max)
{
    return bp_sat_inc(v, max);
}
