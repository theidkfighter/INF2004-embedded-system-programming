/**
 * @file    base_params.c (Baseline_Build harness, host only)
 * @brief   Baseline_Build float parameter table for the differential
 *          Property 22 (Requirements 15.5, 15.6).
 *
 * The Baseline_Build app_params.c is plain C, but its descriptor table
 * PARAMS[] is static. This file #includes the extracted app_params.c
 * (build/base/src/, found through -I) so the table can be read by index.
 * params_init(), params_set(), params_set_many(), params_to_json(),
 * params_count() and params_mm_per_count() are the Baseline_Build functions
 * themselves (base_params_set() etc. in libbase.so).
 *
 * Exported (prefixed base_ in libbase.so):
 *   bh_param_name(i)   NAME of entry i, NULL past the end
 *   bh_param_min(i)    min of entry i
 *   bh_param_max(i)    max of entry i
 *   bh_param_get(i)    current float value of entry i (0 past the end)
 *   bh_param_find(n)   index of NAME n, -1 if unknown
 */
#include <string.h>

#include "app_params.c"

const char *bh_param_name(uint32_t i)
{
    return (i < PARAM_COUNT) ? PARAMS[i].p_name : NULL;
}

float bh_param_min(uint32_t i)
{
    return (i < PARAM_COUNT) ? PARAMS[i].min : 0.0f;
}

float bh_param_max(uint32_t i)
{
    return (i < PARAM_COUNT) ? PARAMS[i].max : 0.0f;
}

float bh_param_get(uint32_t i)
{
    return (i < PARAM_COUNT) ? *PARAMS[i].p_value : 0.0f;
}

int32_t bh_param_find(const char *p_name)
{
    for (uint32_t i = 0u; i < PARAM_COUNT; i++)
    {
        if (strcmp(PARAMS[i].p_name, p_name) == 0)
        {
            return (int32_t)i;
        }
    }
    return -1;
}
