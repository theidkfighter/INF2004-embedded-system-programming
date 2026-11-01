/**
 * @file    evt_stub.c
 * @brief   Host_Test_Suite only: evt_now_ms() for evt.c and test helpers.
 *          post_evt() itself is os_stub.c's (captured on the telemetry
 *          mailbox, task 3.16); these helpers read that capture.
 */
#include <string.h>

#include "evt.h"
#include "host_stubs.h"

static evt_msg_t s_last;
static uint32_t  s_now_ms;

uint32_t evt_now_ms(void)
{
    return s_now_ms;
}

/* test helpers */
void evt_stub_reset(uint32_t now_ms)
{
    os_stub_reset();
    (void)memset(&s_last, 0, sizeof(s_last));
    s_now_ms = now_ms;
}

uint32_t evt_stub_posted(void)
{
    return os_stub_count_mbx(OS_STUB_MBX_TLM);
}

/* The last event posted (zeros when none). */
const evt_msg_t *evt_stub_last(void)
{
    (void)memset(&s_last, 0, sizeof(s_last));
    for (uint32_t i = os_stub_count(); i > 0u; i--)
    {
        uint32_t mbx = 0u;
        evt_msg_t m;

        (void)os_stub_get(i - 1u, &mbx, &m, (uint32_t)sizeof(m));
        if (mbx == OS_STUB_MBX_TLM)
        {
            s_last = m;
            break;
        }
    }
    return &s_last;
}
