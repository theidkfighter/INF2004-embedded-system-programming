/**
 * @file    msg_codec.c
 * @brief   Inter-task message packing (see msg_codec.h).
 */
#include "msg_codec.h"
#include "bitpack.h"

#define P_HDR    (21u)                         /* first type-specific bit */

static const bp_qspec_t Q_SPD13  = { 10,  -4095,  4095,  0 };    /* S13 0.1 mm/s */
static const bp_qspec_t Q_SPD14  = { 10,  -8191,  8191,  0 };    /* S14 0.1 mm/s (line speeds) */
static const bp_qspec_t Q_DEG13  = { 10,  -3600,  3600,  0 };    /* S13 0.1 deg */
static const bp_qspec_t Q_DIST16 = { 10,  -30000, 30000, 0 };    /* S16 0.1 mm */
static const bp_qspec_t Q_SCALE7 = { 100, 0,      100,   0 };    /* U7 0.01 */
static const bp_qspec_t Q_DEG11  = { 10,  -1000,  1000,  0 };    /* S11 0.1 deg */
static const bp_qspec_t Q_CM13S  = { 10,  -4000,  4000,  0 };    /* S13 0.1 cm */
static const bp_qspec_t Q_CM13U  = { 10,  0,      8000,  0 };    /* U13 0.1 cm (width) */
static const bp_qspec_t Q_US13   = { 10,  0,      4310,  8191 }; /* U13 0.1 cm, 8191 = none */

static uint32_t qs(float x, const bp_qspec_t *p_q, uint32_t w)
{
    return bp_tc(bp_quant(x, p_q, (bool *)0), w);
}

static float dq(uint32_t code, uint32_t w, int32_t scale)
{
    return bp_dequant(bp_sext(code, w), scale);
}

static uint32_t us13(float cm)
{
    return ((cm != cm) || (cm < 0.0f)) ? 8191u : (uint32_t)bp_quant(cm, &Q_US13, (bool *)0);
}

static float us13_val(uint32_t c)
{
    return (c == 8191u) ? -1.0f : bp_dequant((int32_t)c, 10);
}

/* ---- motion ------------------------------------------------------------------ */

void msg_motion_pack(const motion_cmd_t *p_c, motion_msg_t *p_m)
{
    uint32_t *w = p_m->w;
    uint32_t  to = (p_c->timeout_ms > 524287u) ? 524287u : p_c->timeout_ms;

    w[0] = 0u;
    w[1] = 0u;
    w[2] = 0u;
    bp_wput(w, 0u, 3u, p_c->type);
    bp_wput(w, 3u, 2u, p_c->src);
    bp_wput(w, 5u, 16u, p_c->id);
    switch (p_c->type)
    {
        case MC_LINE:
            bp_wput(w, P_HDR, 14u, qs(p_c->left_mm_s, &Q_SPD14, 14u));
            bp_wput(w, P_HDR + 14u, 14u, qs(p_c->right_mm_s, &Q_SPD14, 14u));
            break;
        case MC_TURN:
            bp_wput(w, P_HDR, 13u, qs(p_c->turn_deg, &Q_DEG13, 13u));
            bp_wput(w, P_HDR + 13u, 13u, qs(p_c->speed_mm_s, &Q_SPD13, 13u));
            bp_wput(w, P_HDR + 26u, 19u, to);
            break;
        case MC_BYPASS:
            bp_wput(w, P_HDR, 16u, qs(p_c->dist_mm, &Q_DIST16, 16u));
            bp_wput(w, P_HDR + 16u, 13u, qs(p_c->speed_mm_s, &Q_SPD13, 13u));
            bp_wput(w, P_HDR + 29u, 19u, to);
            break;
        case MC_SPEED_LIMIT:
            bp_wput(w, P_HDR, 7u, (uint32_t)bp_quant(p_c->scale, &Q_SCALE7, (bool *)0));
            break;
        default:                                  /* MC_STOP */
            break;
    }
}

void msg_motion_unpack(const motion_msg_t *p_m, motion_cmd_t *p_c)
{
    const uint32_t *w = p_m->w;

    p_c->type       = (uint8_t)bp_wget(w, 0u, 3u);
    p_c->src        = (uint8_t)bp_wget(w, 3u, 2u);
    p_c->id         = (uint16_t)bp_wget(w, 5u, 16u);
    p_c->left_mm_s  = 0.0f;
    p_c->right_mm_s = 0.0f;
    p_c->turn_deg   = 0.0f;
    p_c->dist_mm    = 0.0f;
    p_c->speed_mm_s = 0.0f;
    p_c->scale      = 0.0f;
    p_c->timeout_ms = 0u;
    switch (p_c->type)
    {
        case MC_LINE:
            p_c->left_mm_s  = dq(bp_wget(w, P_HDR, 14u), 14u, 10);
            p_c->right_mm_s = dq(bp_wget(w, P_HDR + 14u, 14u), 14u, 10);
            break;
        case MC_TURN:
            p_c->turn_deg   = dq(bp_wget(w, P_HDR, 13u), 13u, 10);
            p_c->speed_mm_s = dq(bp_wget(w, P_HDR + 13u, 13u), 13u, 10);
            p_c->timeout_ms = bp_wget(w, P_HDR + 26u, 19u);
            break;
        case MC_BYPASS:
            p_c->dist_mm    = dq(bp_wget(w, P_HDR, 16u), 16u, 10);
            p_c->speed_mm_s = dq(bp_wget(w, P_HDR + 16u, 13u), 13u, 10);
            p_c->timeout_ms = bp_wget(w, P_HDR + 29u, 19u);
            break;
        case MC_SPEED_LIMIT:
            p_c->scale = bp_dequant((int32_t)bp_wget(w, P_HDR, 7u), 100);
            break;
        default:
            break;
    }
}

/* ---- nav ----------------------------------------------------------------------- */

void msg_nav_pack(const nav_event_t *p_e, nav_msg_t *p_m)
{
    uint32_t letter = ((p_e->code >= 'A') && (p_e->code <= 'Z')) ? (uint32_t)(p_e->code - 'A') : 0u;

    p_m->b = (uint8_t)(((uint32_t)p_e->type & 7u) | (letter << 3));
}

void msg_nav_unpack(const nav_msg_t *p_m, nav_event_t *p_e)
{
    p_e->type = (uint8_t)(p_m->b & 7u);
    p_e->code = (char)('A' + (p_m->b >> 3));
}

/* ---- obstacle profile -------------------------------------------------------------- */

void msg_obst_pack(const obstacle_profile_t *p_o, obst_msg_t *p_m)
{
    uint32_t *w = p_m->w;
    uint32_t  pos = 0u;

    for (uint32_t i = 0u; i < 6u; i++)
    {
        w[i] = 0u;
    }
    bp_wput(w, pos, 1u, p_o->b_valid ? 1u : 0u);                         pos += 1u;
    bp_wput(w, pos, 2u, (p_o->side == 'L') ? 1u : ((p_o->side == 'R') ? 2u : 0u)); pos += 2u;
    bp_wput(w, pos, 13u, us13(p_o->r_min_cm));                           pos += 13u;
    bp_wput(w, pos, 11u, qs(p_o->theta_l_deg, &Q_DEG11, 11u));          pos += 11u;
    bp_wput(w, pos, 11u, qs(p_o->theta_r_deg, &Q_DEG11, 11u));          pos += 11u;
    bp_wput(w, pos, 13u, qs(p_o->x_l_cm, &Q_CM13S, 13u));               pos += 13u;
    bp_wput(w, pos, 13u, qs(p_o->x_r_cm, &Q_CM13S, 13u));               pos += 13u;
    bp_wput(w, pos, 13u, (uint32_t)bp_quant(p_o->width_cm, &Q_CM13U, (bool *)0)); pos += 13u;
    bp_wput(w, pos, 13u, qs(p_o->offset_cm, &Q_CM13S, 13u));            pos += 13u;
    bp_wput(w, pos, 13u, qs(p_o->clear_left_cm, &Q_CM13S, 13u));        pos += 13u;
    bp_wput(w, pos, 13u, qs(p_o->clear_right_cm, &Q_CM13S, 13u));       pos += 13u;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        bp_wput(w, pos, 13u, us13(p_o->scan_cm[i]));                     pos += 13u;
    }
}

void msg_obst_unpack(const obst_msg_t *p_m, obstacle_profile_t *p_o)
{
    const uint32_t *w = p_m->w;
    uint32_t        pos = 0u;
    uint32_t        sd;

    p_o->b_valid        = (bp_wget(w, pos, 1u) != 0u);                  pos += 1u;
    sd                  = bp_wget(w, pos, 2u);                           pos += 2u;
    p_o->side           = (sd == 1u) ? 'L' : ((sd == 2u) ? 'R' : '\0');
    p_o->r_min_cm       = us13_val(bp_wget(w, pos, 13u));                pos += 13u;
    p_o->theta_l_deg    = dq(bp_wget(w, pos, 11u), 11u, 10);            pos += 11u;
    p_o->theta_r_deg    = dq(bp_wget(w, pos, 11u), 11u, 10);            pos += 11u;
    p_o->x_l_cm         = dq(bp_wget(w, pos, 13u), 13u, 10);            pos += 13u;
    p_o->x_r_cm         = dq(bp_wget(w, pos, 13u), 13u, 10);            pos += 13u;
    p_o->width_cm       = bp_dequant((int32_t)bp_wget(w, pos, 13u), 10); pos += 13u;
    p_o->offset_cm      = dq(bp_wget(w, pos, 13u), 13u, 10);            pos += 13u;
    p_o->clear_left_cm  = dq(bp_wget(w, pos, 13u), 13u, 10);            pos += 13u;
    p_o->clear_right_cm = dq(bp_wget(w, pos, 13u), 13u, 10);            pos += 13u;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        p_o->scan_cm[i] = us13_val(bp_wget(w, pos, 13u));                pos += 13u;
    }
}
