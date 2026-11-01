/**
 * @file    sat_stub.c
 * @brief   Host stand-in for the Saturation_Counter (sat_note / sat_count).
 *
 * Same rule as the firmware in os_glue.c (design C2, Requirement 1.10):
 * 0 at start, sat_note(n) adds n inside one dispatch-disabled section and
 * holds at 65,535 instead of wrapping. The section goes through crit_stub.c,
 * so a sat_note() call made inside another dispatch-disabled section shows
 * up as a nesting error, as it would break tk_dis_dsp() on the target.
 */
#include "host_stubs.h"

#define SAT_MAX (65535u)

static volatile uint16_t s_sat_count;
static uint32_t s_calls;

void sat_note(uint32_t n)
{
    uint32_t v;

    crit_dsp_enter();
    v = (uint32_t)s_sat_count;
    v = (n > (SAT_MAX - v)) ? SAT_MAX : (v + n);
    s_sat_count = (uint16_t)v;
    crit_dsp_exit();
    s_calls++;
}

uint16_t sat_count(void)
{
    return s_sat_count;
}

void sat_stub_reset(void)
{
    s_sat_count = 0u;
    s_calls     = 0u;
}

void sat_stub_set(uint16_t value)
{
    s_sat_count = value;
}

uint32_t sat_stub_calls(void)
{
    return s_calls;
}
