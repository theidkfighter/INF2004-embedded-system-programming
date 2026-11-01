/**
 * @file    evt_stub.c
 * @brief   Host_Test_Suite only: the two kernel-side hooks of evt.c.
 *          post_evt() keeps the last event so tests can decode it.
 */
#include <string.h>

#include "evt.h"

static evt_msg_t s_last;
static uint32_t  s_posted;
static uint32_t  s_now_ms;

bool post_evt(const evt_msg_t *p_m)
{
    s_last = *p_m;
    s_posted++;
    return true;
}

uint32_t evt_now_ms(void)
{
    return s_now_ms;
}

/* test helpers */
void evt_stub_reset(uint32_t now_ms)
{
    (void)memset(&s_last, 0, sizeof(s_last));
    s_posted = 0u;
    s_now_ms = now_ms;
}

uint32_t evt_stub_posted(void)
{
    return s_posted;
}

const evt_msg_t *evt_stub_last(void)
{
    return &s_last;
}
