/**
 * @file    wire.c
 * @brief   Wire_Codec (see wire.h).
 */
#include <string.h>

#include "wire.h"

/* ------------------------------------------------------------------------ */
/* bit writer / reader                                                       */
/* ------------------------------------------------------------------------ */

void bw_init(bw_t *p_w, uint8_t *p_buf, uint32_t cap_bytes)
{
    p_w->p          = p_buf;
    p_w->cap_bits   = cap_bytes * 8u;
    p_w->pos        = 0u;
    p_w->b_overflow = false;
    if (cap_bytes > 0u)
    {
        (void)memset(p_buf, 0, cap_bytes);
    }
}

void bw_put(bw_t *p_w, uint32_t value, uint32_t width)
{
    if ((width == 0u) || (width > 32u))
    {
        return;
    }
    if ((p_w->pos + width) > p_w->cap_bits)
    {
        p_w->b_overflow = true;
        return;
    }
    for (uint32_t i = width; i > 0u; i--)
    {
        if (((value >> (i - 1u)) & 1u) != 0u)
        {
            p_w->p[p_w->pos >> 3] |= (uint8_t)(0x80u >> (p_w->pos & 7u));
        }
        p_w->pos++;
    }
}

void bw_copy(bw_t *p_w, const uint8_t *p_src, uint32_t n_bits)
{
    for (uint32_t i = 0u; i < n_bits; i++)
    {
        bw_put(p_w, (uint32_t)(p_src[i >> 3] >> (7u - (i & 7u))) & 1u, 1u);
    }
}

uint32_t bw_bytes(const bw_t *p_w)
{
    return (p_w->pos + 7u) / 8u;
}

void br_init(br_t *p_r, const uint8_t *p_buf, uint32_t len_bytes)
{
    p_r->p        = p_buf;
    p_r->len_bits = len_bytes * 8u;
    p_r->pos      = 0u;
    p_r->b_short  = false;
}

uint32_t br_get(br_t *p_r, uint32_t width)
{
    uint32_t v = 0u;

    if ((width == 0u) || (width > 32u))
    {
        return 0u;
    }
    if ((p_r->pos + width) > p_r->len_bits)
    {
        p_r->b_short = true;
        return 0u;
    }
    for (uint32_t i = 0u; i < width; i++)
    {
        v = (v << 1) | ((uint32_t)(p_r->p[p_r->pos >> 3] >> (7u - (p_r->pos & 7u))) & 1u);
        p_r->pos++;
    }
    return v;
}

int32_t br_get_s(br_t *p_r, uint32_t width)
{
    uint32_t v = br_get(p_r, width);

    if ((width > 0u) && (width < 32u) && (((v >> (width - 1u)) & 1u) != 0u))
    {
        v |= ~((1u << width) - 1u);
    }
    return (int32_t)v;
}

/* Signed value as a width-bit two's complement code. */
static uint32_t tc(int32_t v, uint32_t width)
{
    return (width >= 32u) ? (uint32_t)v : ((uint32_t)v & ((1u << width) - 1u));
}

/* Unsigned value clamped to width bits (saturating). */
static uint32_t sat_u(uint32_t v, uint32_t width)
{
    uint32_t max = (width >= 32u) ? 0xFFFFFFFFu : ((1u << width) - 1u);

    return (v > max) ? max : v;
}

/* All bits from the reader position to the end are 0. */
static bool rest_zero(const br_t *p_r)
{
    br_t r = *p_r;

    while (r.pos < r.len_bits)
    {
        if (br_get(&r, 1u) != 0u)
        {
            return false;
        }
    }
    return true;
}

static void put_header(bw_t *p_w)
{
    bw_put(p_w, WIRE_FORMAT_VERSION, WIRE_VER_BITS);
}

/* WIRE_OK if the payload starts with the current Format_Version. */
static int check_version(const uint8_t *p, uint32_t len)
{
    if (len == 0u)
    {
        return WIRE_BAD_LENGTH;
    }
    if ((uint32_t)(p[0] >> 4) != WIRE_FORMAT_VERSION)
    {
        return WIRE_BAD_VERSION;
    }
    return WIRE_OK;
}

/* ------------------------------------------------------------------------ */
/* Telemetry_Frame                                                           */
/* ------------------------------------------------------------------------ */

uint32_t wire_enc_telemetry(const wire_tlm_t *p_t, uint8_t *p_out)
{
    bw_t w;
    bool b_thr = (p_t->thr_valid != 0u);

    bw_init(&w, p_out, WIRE_TLM_LEN);
    put_header(&w);
    bw_put(&w, p_t->seq, 15u);
    bw_put(&w, p_t->t_ms, 24u);
    bw_put(&w, p_t->state, 3u);
    bw_put(&w, tc(p_t->spd[0], 15u), 15u);
    bw_put(&w, tc(p_t->spd[1], 15u), 15u);
    bw_put(&w, tc(p_t->duty[0], 11u), 11u);
    bw_put(&w, tc(p_t->duty[1], 11u), 11u);
    bw_put(&w, tc(p_t->enc[0], 24u), 24u);
    bw_put(&w, tc(p_t->enc[1], 24u), 24u);
    bw_put(&w, sat_u(p_t->dist, 20u), 20u);
    bw_put(&w, tc(p_t->line_l, 2u), 2u);
    bw_put(&w, tc(p_t->line_r, 2u), 2u);
    bw_put(&w, sat_u(p_t->ir, 12u), 12u);
    bw_put(&w, sat_u(p_t->ir_min, 12u), 12u);
    bw_put(&w, sat_u(p_t->ir_max, 12u), 12u);
    bw_put(&w, b_thr ? 1u : 0u, 1u);
    bw_put(&w, b_thr ? sat_u(p_t->thr_b, 12u) : 0u, 12u);
    bw_put(&w, b_thr ? sat_u(p_t->thr_w, 12u) : 0u, 12u);
    bw_put(&w, p_t->bc, 6u);
    bw_put(&w, tc(p_t->pitch, 11u), 11u);
    bw_put(&w, tc(p_t->hump_h, 15u), 15u);
    bw_put(&w, sat_u(p_t->hump_peak, 14u), 14u);
    bw_put(&w, sat_u(p_t->hump_max, 14u), 14u);
    bw_put(&w, sat_u(p_t->us, 13u), 13u);
    for (uint32_t i = 0u; i < 5u; i++)
    {
        bw_put(&w, sat_u(p_t->scan[i], 13u), 13u);
    }
    bw_put(&w, p_t->side, 2u);
    bw_put(&w, p_t->imu_ev, 3u);
    bw_put(&w, tc(p_t->turn, 14u), 14u);
    bw_put(&w, p_t->faults, 16u);
    bw_put(&w, sat_u(p_t->vbat, 13u), 13u);
    bw_put(&w, p_t->rssi, 8u);
    bw_put(&w, sat_u(p_t->buf, 7u), 7u);
    return (w.b_overflow || (w.pos != WIRE_TLM_BITS)) ? 0u : bw_bytes(&w);
}

int wire_dec_telemetry(const uint8_t *p, uint32_t len, wire_tlm_t *p_t)
{
    br_t r;
    int  st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    if (len != WIRE_TLM_LEN)
    {
        return WIRE_BAD_LENGTH;
    }
    (void)memset(p_t, 0, sizeof(*p_t));
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    p_t->seq       = (uint16_t)br_get(&r, 15u);
    p_t->t_ms      = br_get(&r, 24u);
    p_t->state     = (uint8_t)br_get(&r, 3u);
    p_t->spd[0]    = (int16_t)br_get_s(&r, 15u);
    p_t->spd[1]    = (int16_t)br_get_s(&r, 15u);
    p_t->duty[0]   = (int16_t)br_get_s(&r, 11u);
    p_t->duty[1]   = (int16_t)br_get_s(&r, 11u);
    p_t->enc[0]    = br_get_s(&r, 24u);
    p_t->enc[1]    = br_get_s(&r, 24u);
    p_t->dist      = br_get(&r, 20u);
    p_t->line_l    = (int8_t)br_get_s(&r, 2u);
    p_t->line_r    = (int8_t)br_get_s(&r, 2u);
    p_t->ir        = (uint16_t)br_get(&r, 12u);
    p_t->ir_min    = (uint16_t)br_get(&r, 12u);
    p_t->ir_max    = (uint16_t)br_get(&r, 12u);
    p_t->thr_valid = (uint8_t)br_get(&r, 1u);
    p_t->thr_b     = (uint16_t)br_get(&r, 12u);
    p_t->thr_w     = (uint16_t)br_get(&r, 12u);
    p_t->bc        = (uint8_t)br_get(&r, 6u);
    p_t->pitch     = (int16_t)br_get_s(&r, 11u);
    p_t->hump_h    = (int16_t)br_get_s(&r, 15u);
    p_t->hump_peak = (uint16_t)br_get(&r, 14u);
    p_t->hump_max  = (uint16_t)br_get(&r, 14u);
    p_t->us        = (uint16_t)br_get(&r, 13u);
    for (uint32_t i = 0u; i < 5u; i++)
    {
        p_t->scan[i] = (uint16_t)br_get(&r, 13u);
    }
    p_t->side      = (uint8_t)br_get(&r, 2u);
    p_t->imu_ev    = (uint8_t)br_get(&r, 3u);
    p_t->turn      = (int16_t)br_get_s(&r, 14u);
    p_t->faults    = (uint16_t)br_get(&r, 16u);
    p_t->vbat      = (uint16_t)br_get(&r, 13u);
    p_t->rssi      = (uint8_t)br_get(&r, 8u);
    p_t->buf       = (uint8_t)br_get(&r, 7u);
    if ((p_t->thr_valid == 0u) && ((p_t->thr_b != 0u) || (p_t->thr_w != 0u)))
    {
        return WIRE_BAD_VALUE;
    }
    return r.b_short ? WIRE_BAD_LENGTH : WIRE_OK;
}

/* ------------------------------------------------------------------------ */
/* Heartbeat_Frame                                                           */
/* ------------------------------------------------------------------------ */

uint32_t wire_enc_heartbeat(const wire_hb_t *p_h, uint8_t *p_out)
{
    bw_t w;

    bw_init(&w, p_out, WIRE_HB_LEN);
    put_header(&w);
    bw_put(&w, p_h->seq & 0x3FFFu, 14u);           /* M14: modular */
    bw_put(&w, p_h->t_ms, 24u);
    bw_put(&w, p_h->state, 3u);
    bw_put(&w, p_h->rssi, 8u);
    bw_put(&w, sat_u(p_h->buf, 7u), 7u);
    bw_put(&w, sat_u(p_h->uptime_s, 17u), 17u);
    bw_put(&w, p_h->reconnects, 16u);
    bw_put(&w, p_h->dropped, 16u);
    bw_put(&w, p_h->sat, 16u);
    bw_put(&w, p_h->faults, 16u);
    bw_put(&w, sat_u(p_h->n_params, 6u), 6u);
    bw_put(&w, p_h->param_crc, 16u);
    bw_copy(&w, p_h->ip, 32u);
    return (w.b_overflow || (w.pos != WIRE_HB_BITS)) ? 0u : bw_bytes(&w);
}

int wire_dec_heartbeat(const uint8_t *p, uint32_t len, wire_hb_t *p_h)
{
    br_t r;
    int  st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    if (len != WIRE_HB_LEN)
    {
        return WIRE_BAD_LENGTH;
    }
    (void)memset(p_h, 0, sizeof(*p_h));
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    p_h->seq        = (uint16_t)br_get(&r, 14u);
    p_h->t_ms       = br_get(&r, 24u);
    p_h->state      = (uint8_t)br_get(&r, 3u);
    p_h->rssi       = (uint8_t)br_get(&r, 8u);
    p_h->buf        = (uint8_t)br_get(&r, 7u);
    p_h->uptime_s   = br_get(&r, 17u);
    p_h->reconnects = (uint16_t)br_get(&r, 16u);
    p_h->dropped    = (uint16_t)br_get(&r, 16u);
    p_h->sat        = (uint16_t)br_get(&r, 16u);
    p_h->faults     = (uint16_t)br_get(&r, 16u);
    p_h->n_params   = (uint8_t)br_get(&r, 6u);
    p_h->param_crc  = (uint16_t)br_get(&r, 16u);
    for (uint32_t i = 0u; i < 4u; i++)
    {
        p_h->ip[i] = (uint8_t)br_get(&r, 8u);
    }
    return rest_zero(&r) ? WIRE_OK : WIRE_BAD_VALUE;
}

/* ------------------------------------------------------------------------ */
/* Event_Frame                                                               */
/* ------------------------------------------------------------------------ */

/* BC_DBG variants (evt.h): bits after the 3-bit variant field. */
#define BCV_OFF      (0u)
#define BCV_WINDOW   (1u)     /* n U7, res E2, code U6, rev U1, thr_b U12, thr_w U12, ovr U16 */
#define BCV_MULTI    (2u)     /* BCV_WINDOW + level U3, conflict U1 */
#define BCV_LEVELS   (3u)     /* n_levels U3 (1..6), 6 x (res E2, code C6, n U7) (D10) */
#define BCV_WIDTHS   (4u)     /* part U3 (1..6), count U5 (0..16), count x U12 */

int wire_evt_arg_bits(uint32_t code, const uint8_t *p_args, uint32_t avail, uint32_t *p_bits)
{
    br_t     r;
    uint32_t n;

    if (wire_evt_name(code) == NULL)
    {
        return WIRE_BAD_CODE;
    }
    br_init(&r, p_args, (avail + 7u) / 8u);
    r.len_bits = avail;
    if (code == (uint32_t)EVC_FINE_SCAN)
    {
        (void)br_get(&r, 7u);
        n = br_get(&r, 4u);
        if (r.b_short)
        {
            return WIRE_BAD_LENGTH;
        }
        if ((n < 1u) || (n > WIRE_FINE_SCAN_MAX))
        {
            return WIRE_BAD_COUNT;
        }
        *p_bits = 11u + (9u * n);
        return WIRE_OK;
    }
    if (code == (uint32_t)EVC_BC_DBG)
    {
        uint32_t v = br_get(&r, 3u);

        if (r.b_short)
        {
            return WIRE_BAD_LENGTH;
        }
        switch (v)
        {
            case BCV_OFF:    *p_bits = 3u;               return WIRE_OK;
            case BCV_WINDOW: *p_bits = 3u + 56u;         return WIRE_OK;
            case BCV_MULTI:  *p_bits = 3u + 60u;         return WIRE_OK;
            case BCV_LEVELS:
                n = br_get(&r, 3u);
                if (r.b_short)
                {
                    return WIRE_BAD_LENGTH;
                }
                if ((n < 1u) || (n > WIRE_BC_PARTS_MAX))
                {
                    return WIRE_BAD_COUNT;
                }
                *p_bits = 6u + (15u * WIRE_BC_PARTS_MAX);   /* always 6 entries */
                return WIRE_OK;
            case BCV_WIDTHS:
            {
                uint32_t part = br_get(&r, 3u);

                n = br_get(&r, 5u);
                if (r.b_short)
                {
                    return WIRE_BAD_LENGTH;
                }
                if ((part < 1u) || (part > WIRE_BC_PARTS_MAX) || (n > WIRE_BC_WIDTHS_MAX))
                {
                    return WIRE_BAD_COUNT;
                }
                *p_bits = 11u + (12u * n);
                return WIRE_OK;
            }
            default:
                return WIRE_BAD_VALUE;
        }
    }
    *p_bits = wire_evt_arg_bits_max(code);
    return WIRE_OK;
}

uint32_t wire_enc_event(uint32_t t_ms, const wire_evt_t *p_e, uint8_t *p_out, uint32_t cap)
{
    bw_t     w;
    br_t     r;
    uint32_t need;
    uint32_t len;

    if ((p_e->nbits > WIRE_EVT_ARG_CAP_BITS) ||
        (wire_evt_arg_bits(p_e->code, p_e->args, p_e->nbits, &need) != WIRE_OK) ||
        (need != p_e->nbits))
    {
        return 0u;
    }
    len = WIRE_EVT_LEN(need);
    if (len > cap)
    {
        return 0u;
    }
    bw_init(&w, p_out, len);
    put_header(&w);
    bw_put(&w, t_ms, WIRE_TIME_BITS);
    bw_put(&w, p_e->code, WIRE_EVT_CODE_BITS);
    br_init(&r, p_e->args, WIRE_EVT_ARG_BYTES);
    for (uint32_t done = 0u; done < need; )
    {
        uint32_t k = ((need - done) > 16u) ? 16u : (need - done);

        bw_put(&w, br_get(&r, k), k);
        done += k;
    }
    return w.b_overflow ? 0u : bw_bytes(&w);
}

int wire_dec_event(const uint8_t *p, uint32_t len, uint32_t *p_t_ms, wire_evt_t *p_e)
{
    br_t     r;
    bw_t     w;
    uint32_t code;
    uint32_t need;
    uint32_t avail;
    int      st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    if (len < WIRE_BYTES(WIRE_EVT_HDR_BITS))
    {
        return WIRE_BAD_LENGTH;
    }
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    *p_t_ms = br_get(&r, WIRE_TIME_BITS);
    code    = br_get(&r, WIRE_EVT_CODE_BITS);
    avail   = (len * 8u) - WIRE_EVT_HDR_BITS;
    if (avail > WIRE_EVT_ARG_CAP_BITS)
    {
        avail = WIRE_EVT_ARG_CAP_BITS;
    }
    (void)memset(p_e, 0, sizeof(*p_e));
    bw_init(&w, p_e->args, WIRE_EVT_ARG_BYTES);
    {
        br_t r2 = r;

        for (uint32_t done = 0u; done < avail; )
        {
            uint32_t k = ((avail - done) > 16u) ? 16u : (avail - done);

            bw_put(&w, br_get(&r2, k), k);
            done += k;
        }
    }
    st = wire_evt_arg_bits(code, p_e->args, avail, &need);
    if (st != WIRE_OK)
    {
        return st;
    }
    if (len != WIRE_EVT_LEN(need))
    {
        return WIRE_BAD_LENGTH;
    }
    p_e->code  = (uint8_t)code;
    p_e->nbits = (uint8_t)need;
    /* clear the copied bits past the arguments, then check the padding */
    {
        uint32_t keep_bytes = (need + 7u) / 8u;

        if ((need & 7u) != 0u)
        {
            p_e->args[keep_bytes - 1u] &= (uint8_t)(0xFFu << (8u - (need & 7u)));
        }
        for (uint32_t i = keep_bytes; i < WIRE_EVT_ARG_BYTES; i++)
        {
            p_e->args[i] = 0u;
        }
    }
    r.pos += need;
    return rest_zero(&r) ? WIRE_OK : WIRE_BAD_VALUE;
}

/* ------------------------------------------------------------------------ */
/* Ack_Frame                                                                 */
/* ------------------------------------------------------------------------ */

static void put_map(bw_t *p_w, const wire_ack_t *p_a)
{
    for (uint32_t i = 0u; i < WIRE_MAP_LETTERS; i++)
    {
        bw_put(p_w, p_a->a.map.act[i], 3u);
    }
    bw_put(p_w, p_a->a.map.n, 5u);
}

uint32_t wire_enc_ack(const wire_ack_t *p_a, uint8_t *p_out, uint32_t cap)
{
    bw_t w;

    if ((p_a->reason >= (uint8_t)RSN_COUNT) ||
        ((p_a->cmd >= (uint8_t)WCMD_TABLE_COUNT) && (p_a->cmd != (uint8_t)WCMD_UNPARSED)))
    {
        return 0u;
    }
    bw_init(&w, p_out, cap);
    put_header(&w);
    bw_put(&w, p_a->t_ms, WIRE_TIME_BITS);
    bw_put(&w, p_a->cmd, WIRE_CMD_CODE_BITS);
    bw_put(&w, p_a->reason, WIRE_REASON_BITS);
    switch (p_a->cmd)
    {
        case WCMD_ESTOP:
            bw_put(&w, p_a->a.pwm_off_us, 16u);
            break;
        case WCMD_BARCODE:
            bw_put(&w, p_a->a.letter, 5u);
            break;
        case WCMD_MAP:
        case WCMD_MAP_CLEAR:
        case WCMD_GET_MAP:
            if (p_a->a.map.n > WIRE_MAP_LETTERS)
            {
                return 0u;
            }
            put_map(&w, p_a);
            break;
        case WCMD_PING:
            bw_put(&w, p_a->a.ping, 16u);
            break;
        case WCMD_SET:
            bw_put(&w, p_a->a.param_id, 6u);
            break;
        case WCMD_GET_PARAMS:
            if ((p_a->a.getp.part < 1u) || (p_a->a.getp.part > 7u) ||
                (p_a->a.getp.count > WIRE_GETP_PER_PART))
            {
                return 0u;
            }
            bw_put(&w, p_a->a.getp.part, 3u);
            bw_put(&w, p_a->a.getp.count, 4u);
            for (uint32_t i = 0u; i < p_a->a.getp.count; i++)
            {
                uint32_t wd = p_a->a.getp.v[i].w;

                if ((wd == 0u) || ((wd > 3u) && (wd != 32u)))
                {
                    return 0u;
                }
                bw_put(&w, p_a->a.getp.v[i].id, 6u);
                bw_put(&w, p_a->a.getp.v[i].code, wd);
            }
            break;
        case WCMD_CONFIG:
            if (p_a->a.cfg.n_rej > WIRE_CFG_REJ_MAX)
            {
                return 0u;
            }
            bw_put(&w, p_a->a.cfg.applied, 6u);
            bw_put(&w, p_a->a.cfg.n_rej, 6u);
            for (uint32_t i = 0u; i < p_a->a.cfg.n_rej; i++)
            {
                bw_put(&w, p_a->a.cfg.rej[i], 6u);
            }
            break;
        default:                                  /* RESUME, START, STOP, MAP:CLEAR, UNPARSED */
            break;
    }
    return w.b_overflow ? 0u : bw_bytes(&w);
}

int wire_dec_ack(const uint8_t *p, uint32_t len, wire_pbits_fn p_pbits, wire_ack_t *p_a)
{
    br_t r;
    int  st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    if (len < WIRE_BYTES(WIRE_ACK_HDR_BITS))
    {
        return WIRE_BAD_LENGTH;
    }
    (void)memset(p_a, 0, sizeof(*p_a));
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    p_a->t_ms   = br_get(&r, WIRE_TIME_BITS);
    p_a->cmd    = (uint8_t)br_get(&r, WIRE_CMD_CODE_BITS);
    p_a->reason = (uint8_t)br_get(&r, WIRE_REASON_BITS);
    if ((p_a->cmd >= (uint8_t)WCMD_TABLE_COUNT) && (p_a->cmd != (uint8_t)WCMD_UNPARSED))
    {
        return WIRE_BAD_CODE;
    }
    if (p_a->reason >= (uint8_t)RSN_COUNT)
    {
        return WIRE_BAD_VALUE;
    }
    if (wire_ack_len(p_a->cmd) != 0u)
    {
        if (len != wire_ack_len(p_a->cmd))
        {
            return WIRE_BAD_LENGTH;
        }
    }
    switch (p_a->cmd)
    {
        case WCMD_ESTOP:
            p_a->a.pwm_off_us = (uint16_t)br_get(&r, 16u);
            break;
        case WCMD_BARCODE:
            p_a->a.letter = (uint8_t)br_get(&r, 5u);
            break;
        case WCMD_MAP:
        case WCMD_MAP_CLEAR:
        case WCMD_GET_MAP:
            for (uint32_t i = 0u; i < WIRE_MAP_LETTERS; i++)
            {
                p_a->a.map.act[i] = (uint8_t)br_get(&r, 3u);
            }
            p_a->a.map.n = (uint8_t)br_get(&r, 5u);
            if (p_a->a.map.n > WIRE_MAP_LETTERS)
            {
                return WIRE_BAD_COUNT;
            }
            break;
        case WCMD_PING:
            p_a->a.ping = (uint16_t)br_get(&r, 16u);
            break;
        case WCMD_SET:
            p_a->a.param_id = (uint8_t)br_get(&r, 6u);
            break;
        case WCMD_GET_PARAMS:
            p_a->a.getp.part  = (uint8_t)br_get(&r, 3u);
            p_a->a.getp.count = (uint8_t)br_get(&r, 4u);
            if ((p_a->a.getp.part < 1u) || (p_a->a.getp.count > WIRE_GETP_PER_PART))
            {
                return WIRE_BAD_COUNT;
            }
            for (uint32_t i = 0u; i < p_a->a.getp.count; i++)
            {
                uint32_t id = br_get(&r, 6u);
                uint32_t wd = (p_pbits != NULL) ? p_pbits(id) : 32u;

                if (wd == 0u)
                {
                    wd = 32u;
                }
                p_a->a.getp.v[i].id   = (uint8_t)id;
                p_a->a.getp.v[i].w    = (uint8_t)wd;
                p_a->a.getp.v[i].code = br_get(&r, wd);
            }
            break;
        case WCMD_CONFIG:
            p_a->a.cfg.applied = (uint8_t)br_get(&r, 6u);
            p_a->a.cfg.n_rej   = (uint8_t)br_get(&r, 6u);
            if (p_a->a.cfg.n_rej > WIRE_CFG_REJ_MAX)
            {
                return WIRE_BAD_COUNT;
            }
            for (uint32_t i = 0u; i < p_a->a.cfg.n_rej; i++)
            {
                p_a->a.cfg.rej[i] = (uint8_t)br_get(&r, 6u);
            }
            break;
        default:
            break;
    }
    if (r.b_short)
    {
        return WIRE_BAD_LENGTH;
    }
    if (len != ((r.pos + 7u) / 8u))
    {
        return WIRE_BAD_LENGTH;
    }
    return rest_zero(&r) ? WIRE_OK : WIRE_BAD_VALUE;
}

/* ------------------------------------------------------------------------ */
/* Command_Frame, Map_Frame, Config_Frame (laptop -> robot)                  */
/* ------------------------------------------------------------------------ */

static int get_map_body(br_t *p_r, wire_cmd_t *p_c)
{
    uint32_t seen = 0u;

    for (uint32_t i = 0u; i < 4u; i++)
    {
        p_c->map_present[i] = (uint8_t)br_get(p_r, 1u);
        p_c->map_letter[i]  = (uint8_t)br_get(p_r, 5u);
        if (p_c->map_present[i] != 0u)
        {
            if (p_c->map_letter[i] >= WIRE_MAP_LETTERS)
            {
                return WIRE_BAD_LETTER;
            }
            if ((seen & (1u << p_c->map_letter[i])) != 0u)
            {
                return WIRE_BAD_DUP_LETTER;
            }
            seen |= 1u << p_c->map_letter[i];
        }
        else if (p_c->map_letter[i] != 0u)
        {
            return WIRE_BAD_VALUE;
        }
    }
    return WIRE_OK;
}

int wire_dec_cmd(const uint8_t *p, uint32_t len, wire_cmd_t *p_c)
{
    br_t r;
    int  st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    (void)memset(p_c, 0, sizeof(*p_c));
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    p_c->cmd = (uint8_t)br_get(&r, WIRE_CMD_CODE_BITS);
    if (wire_cmd_len(p_c->cmd) == 0u)
    {
        return WIRE_BAD_CODE;
    }
    if (len != wire_cmd_len(p_c->cmd))
    {
        return WIRE_BAD_LENGTH;
    }
    switch (p_c->cmd)
    {
        case WCMD_BARCODE:
            p_c->letter = (uint8_t)br_get(&r, 5u);
            if (p_c->letter >= WIRE_MAP_LETTERS)
            {
                return WIRE_BAD_LETTER;
            }
            break;
        case WCMD_MAP:
            st = get_map_body(&r, p_c);
            if (st != WIRE_OK)
            {
                return st;
            }
            break;
        case WCMD_PING:
            p_c->ping = (uint16_t)br_get(&r, 16u);
            break;
        case WCMD_SET:
            p_c->param_id   = (uint8_t)br_get(&r, 6u);
            p_c->value_bits = br_get(&r, 32u);
            break;
        default:
            break;
    }
    return rest_zero(&r) ? WIRE_OK : WIRE_BAD_VALUE;
}

uint32_t wire_enc_cmd(const wire_cmd_t *p_c, uint8_t *p_out, uint32_t cap)
{
    bw_t     w;
    uint32_t len = wire_cmd_len(p_c->cmd);

    if ((len == 0u) || (len > cap))
    {
        return 0u;
    }
    bw_init(&w, p_out, len);
    put_header(&w);
    bw_put(&w, p_c->cmd, WIRE_CMD_CODE_BITS);
    switch (p_c->cmd)
    {
        case WCMD_BARCODE:
            bw_put(&w, p_c->letter, 5u);
            break;
        case WCMD_MAP:
            for (uint32_t i = 0u; i < 4u; i++)
            {
                bw_put(&w, (p_c->map_present[i] != 0u) ? 1u : 0u, 1u);
                bw_put(&w, (p_c->map_present[i] != 0u) ? p_c->map_letter[i] : 0u, 5u);
            }
            break;
        case WCMD_PING:
            bw_put(&w, p_c->ping, 16u);
            break;
        case WCMD_SET:
            bw_put(&w, p_c->param_id, 6u);
            bw_put(&w, p_c->value_bits, 32u);
            break;
        default:
            break;
    }
    return w.b_overflow ? 0u : len;
}

int wire_dec_map(const uint8_t *p, uint32_t len, wire_cmd_t *p_c)
{
    br_t r;
    int  st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    if (len != WIRE_MAP_LEN)
    {
        return WIRE_BAD_LENGTH;
    }
    (void)memset(p_c, 0, sizeof(*p_c));
    p_c->cmd = (uint8_t)WCMD_MAP;
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    st = get_map_body(&r, p_c);
    return st;
}

int wire_dec_config(const uint8_t *p, uint32_t len, wire_cfg_fn p_apply, void *p_ctx)
{
    br_t     r;
    uint32_t n;
    int      st = check_version(p, len);

    if (st != WIRE_OK)
    {
        return st;
    }
    br_init(&r, p, len);
    (void)br_get(&r, WIRE_VER_BITS);
    n = br_get(&r, 6u);
    if (r.b_short)
    {
        return WIRE_BAD_LENGTH;
    }
    if (n > WIRE_PARAMS_MAX)
    {
        return WIRE_BAD_COUNT;
    }
    if (len != WIRE_CFG_LEN(n))
    {
        return WIRE_BAD_LENGTH;
    }
    for (uint32_t i = 0u; i < n; i++)
    {
        uint32_t id = br_get(&r, 6u);
        uint32_t v  = br_get(&r, 32u);

        if (p_apply != NULL)
        {
            p_apply(id, v, p_ctx);
        }
    }
    return rest_zero(&r) ? WIRE_OK : WIRE_BAD_VALUE;
}

/* Inverse encoders for the host tests and tools (spec task 3.10). */
uint32_t wire_enc_map(const wire_cmd_t *p_c, uint8_t *p_out, uint32_t cap)
{
    bw_t w;

    if (cap < WIRE_MAP_LEN)
    {
        return 0u;
    }
    bw_init(&w, p_out, WIRE_MAP_LEN);
    put_header(&w);
    for (uint32_t i = 0u; i < 4u; i++)
    {
        bw_put(&w, (p_c->map_present[i] != 0u) ? 1u : 0u, 1u);
        bw_put(&w, (p_c->map_present[i] != 0u) ? p_c->map_letter[i] : 0u, 5u);
    }
    return w.b_overflow ? 0u : WIRE_MAP_LEN;
}

uint32_t wire_enc_config(uint32_t n, const uint8_t *p_ids, const uint32_t *p_bits, uint8_t *p_out, uint32_t cap)
{
    bw_t     w;
    uint32_t len = WIRE_CFG_LEN(n);

    if ((n > WIRE_PARAMS_MAX) || (len > cap))
    {
        return 0u;
    }
    bw_init(&w, p_out, len);
    put_header(&w);
    bw_put(&w, n, 6u);
    for (uint32_t i = 0u; i < n; i++)
    {
        bw_put(&w, p_ids[i], 6u);
        bw_put(&w, p_bits[i], 32u);
    }
    return w.b_overflow ? 0u : len;
}
