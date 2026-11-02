/**
 * @file    os_stub.c
 * @brief   Host stand-in for the micro T-Kernel side of os_glue.c.
 *
 * - Message capture: every post goes through os_stub_post(), which models the
 *   40-block Message_Pool (TMO_POL: no free block -> false, nothing sent) and
 *   records the body bytes in posting order for the tests to decode.
 * - post_evt() (evt.h, task 3.16) is captured on the telemetry mailbox once
 *   that header exists.
 * - Event flags: flag_set/clear/peek/take on a plain 32-bit pattern, with the
 *   os_glue.h signatures and the TWF_ORW | TWF_BITCLR rule of flag_take().
 */
#include <string.h>
#include "host_stubs.h"

#if defined(__has_include)
#  if __has_include("evt.h")
#    include "evt.h"
#    define HOST_HAVE_EVT_H 1
#  endif
#endif

#define CAPTURE_MAX (1024u)

typedef struct
{
    uint32_t mbx;
    uint32_t len;
    uint8_t  body[OS_STUB_MAX_BODY];
} capture_t;

static capture_t s_cap[CAPTURE_MAX];
static uint32_t  s_n_cap;
static uint32_t  s_n_mbx[OS_STUB_MBX_COUNT];
static uint32_t  s_rejected;
static uint32_t  s_blocks_used;
static bool      s_b_pool_empty;

static uint32_t  s_flags;
static uint32_t  s_flag_set_calls;

void os_stub_reset(void)
{
    s_n_cap         = 0u;
    s_rejected      = 0u;
    s_blocks_used   = 0u;
    s_b_pool_empty  = false;
    s_flags         = 0u;
    s_flag_set_calls = 0u;
    memset(s_n_mbx, 0, sizeof s_n_mbx);
}

void os_stub_set_pool_empty(bool b_empty)
{
    s_b_pool_empty = b_empty;
}

bool os_stub_post(uint32_t mbx, const void *p_body, uint32_t len)
{
    capture_t *p_c;

    if (s_b_pool_empty || (s_blocks_used >= OS_STUB_POOL_BLOCKS) ||
        (mbx >= OS_STUB_MBX_COUNT) || (len > OS_STUB_MAX_BODY) ||
        (s_n_cap >= CAPTURE_MAX) || ((p_body == NULL) && (len > 0u)))
    {
        s_rejected++;
        return false;
    }
    p_c = &s_cap[s_n_cap];
    p_c->mbx = mbx;
    p_c->len = len;
    memset(p_c->body, 0, sizeof p_c->body);
    if (len > 0u)
    {
        memcpy(p_c->body, p_body, len);
    }
    s_n_cap++;
    s_n_mbx[mbx]++;
    s_blocks_used++;
    return true;
}

uint32_t os_stub_count(void)
{
    return s_n_cap;
}

uint32_t os_stub_count_mbx(uint32_t mbx)
{
    return (mbx < OS_STUB_MBX_COUNT) ? s_n_mbx[mbx] : 0u;
}

uint32_t os_stub_rejected(void)
{
    return s_rejected;
}

uint32_t os_stub_get(uint32_t idx, uint32_t *p_mbx, void *p_out, uint32_t cap)
{
    const capture_t *p_c;

    if (idx >= s_n_cap)
    {
        return 0u;
    }
    p_c = &s_cap[idx];
    if (p_mbx != NULL)
    {
        *p_mbx = p_c->mbx;
    }
    if (p_out != NULL)
    {
        memcpy(p_out, p_c->body, (p_c->len < cap) ? p_c->len : cap);
    }
    return p_c->len;
}

void os_stub_free(uint32_t n)
{
    s_blocks_used = (n >= s_blocks_used) ? 0u : (s_blocks_used - n);
}

#ifdef HOST_HAVE_EVT_H
bool post_evt(const evt_msg_t *p)
{
    return os_stub_post(OS_STUB_MBX_TLM, p, (uint32_t)sizeof *p);
}
#endif

/* ---- event flags ---------------------------------------------------------- */

void flag_set(uint32_t bits)
{
    s_flags |= bits;
    s_flag_set_calls++;
}

void flag_clear(uint32_t bits)
{
    s_flags &= ~bits;
}

uint32_t flag_peek(void)
{
    return s_flags;
}

bool flag_take(uint32_t bits, uint32_t *p_got)
{
    uint32_t got = s_flags & bits;

    if (got == 0u)
    {
        *p_got = 0u;
        return false;
    }
    s_flags &= ~got;
    *p_got = got;
    return true;
}

void os_stub_flags_load(uint32_t bits)
{
    s_flags = bits;
}

uint32_t os_stub_flag_set_calls(void)
{
    return s_flag_set_calls;
}
