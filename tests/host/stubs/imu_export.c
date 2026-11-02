/**
 * @file    imu_export.c
 * @brief   Host_Test_Suite only: out-of-line wrappers around the static
 *          inline setters of app_program/imu_logic.h, so the Python tests can
 *          call them through ctypes (py/test_prop15_hump.py).
 */
#include <stddef.h>

#include "imu_logic.h"
#include "host_stubs.h"

uint32_t imx_hump_ctx_size(void)
{
    return (uint32_t)sizeof(hump_ctx_t);
}

uint32_t imx_mev_ctx_size(void)
{
    return (uint32_t)sizeof(mev_ctx_t);
}

uint32_t imx_hump_n_max(void)
{
    return HUMP_N_MAX;
}

bool imx_hump_set_state(hump_ctx_t *p_h, uint32_t v)
{
    return hump_set_state(p_h, (hump_state_t)v);
}

bool imx_hump_set_n_up(hump_ctx_t *p_h, uint32_t v)
{
    return hump_set_n_up(p_h, v);
}

bool imx_hump_set_n_down(hump_ctx_t *p_h, uint32_t v)
{
    return hump_set_n_down(p_h, v);
}

bool imx_hump_set_n_level(hump_ctx_t *p_h, uint32_t v)
{
    return hump_set_n_level(p_h, v);
}

bool imx_mev_set_current(mev_ctx_t *p_m, uint32_t v)
{
    return mev_set_current(p_m, (motion_event_t)v);
}

bool imx_mev_set_candidate(mev_ctx_t *p_m, uint32_t v)
{
    return mev_set_candidate(p_m, (motion_event_t)v);
}

bool imx_mev_set_n_candidate(mev_ctx_t *p_m, uint32_t v)
{
    return mev_set_n_candidate(p_m, v);
}
