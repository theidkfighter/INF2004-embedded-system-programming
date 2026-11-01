/**
 * @file    evt.c
 * @brief   Event_Frame builder (see evt.h). The posting side is in os_glue.c
 *          (post_evt), so this file stays plain C for the Host_Test_Suite.
 */
#include <string.h>

#include "evt.h"
#include "code39.h"

bool post_evt(const evt_msg_t *p_m);          /* os_glue.c (host: stubs) */
uint32_t evt_now_ms(void);                    /* os_glue.c (host: stubs) */

void evt_begin(evt_b_t *p_b, evt_code_t code)
{
    (void)memset(&p_b->m, 0, sizeof(p_b->m));
    p_b->m.t_ms   = evt_now_ms();
    p_b->m.e.code = (uint8_t)code;
    bw_init(&p_b->w, p_b->m.e.args, WIRE_EVT_ARG_BYTES);
}

void evt_u(evt_b_t *p_b, uint32_t v, uint32_t width)
{
    uint32_t max = (width >= 32u) ? 0xFFFFFFFFu : ((1u << width) - 1u);

    bw_put(&p_b->w, (v > max) ? max : v, width);
}

void evt_s(evt_b_t *p_b, int32_t v, uint32_t width)
{
    int32_t hi = (int32_t)((1u << (width - 1u)) - 1u);
    int32_t lo = -hi - 1;

    v = (v > hi) ? hi : ((v < lo) ? lo : v);
    bw_put(&p_b->w, (uint32_t)v & ((1u << width) - 1u), width);
}

/* Rounds to the nearest whole unit, halves away from zero. */
static int32_t round_f(float x)
{
    return (x < 0.0f) ? -(int32_t)(0.5f - x) : (int32_t)(x + 0.5f);
}

void evt_cm13(evt_b_t *p_b, float cm)
{
    int32_t c = round_f(cm * 10.0f);

    if ((cm != cm) || (cm < 0.0f))
    {
        evt_u(p_b, EVT_CM13_NONE, 13u);
        return;
    }
    evt_u(p_b, (c > 4310) ? 4310u : (uint32_t)c, 13u);
}

void evt_cm9(evt_b_t *p_b, float cm)
{
    int32_t c = round_f(cm);

    if ((cm != cm) || (cm < 0.0f))
    {
        evt_u(p_b, EVT_CM9_NONE, 9u);
        return;
    }
    evt_u(p_b, (c > 431) ? 431u : (uint32_t)c, 9u);
}

bool evt_post(evt_b_t *p_b)
{
    uint32_t need;

    if (p_b->w.b_overflow ||
        (wire_evt_arg_bits(p_b->m.e.code, p_b->m.e.args, p_b->w.pos, &need) != WIRE_OK) ||
        (need != p_b->w.pos))
    {
        return false;                             /* a layout bug: never sent */
    }
    p_b->m.e.nbits = (uint8_t)p_b->w.pos;
    return post_evt(&p_b->m);
}

bool evt_post0(evt_code_t code)
{
    evt_b_t b;

    evt_begin(&b, code);
    return evt_post(&b);
}

bool evt_post1(evt_code_t code, uint32_t v, uint32_t width)
{
    evt_b_t b;

    evt_begin(&b, code);
    evt_u(&b, v, width);
    return evt_post(&b);
}

void evt_map26(evt_b_t *p_b, const uint8_t p_act[26])
{
    for (uint32_t i = 0u; i < 26u; i++)
    {
        evt_u(p_b, p_act[i], 3u);
    }
}

uint32_t evt_char6(char c)
{
    return c39_to_index(c);                  /* C6 (design D10): Code 39 index 0..42, 63 none */
}
