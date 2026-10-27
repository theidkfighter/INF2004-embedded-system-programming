/**
 * @file    bc_multi.c
 * @brief   Barcode reading at several switch levels at once (see bc_multi.h).
 */
#include "bc_multi.h"

static const float LEVEL[BCM_MAX_LEVELS] = { 0.50f, 0.35f, 0.65f, 0.25f, 0.75f };

uint32_t bcm_level_pct(uint32_t i)
{
    return (i < BCM_MAX_LEVELS) ? (uint32_t)((LEVEL[i] * 100.0f) + 0.5f) : 0u;
}

void bcm_init(bcm_t *p_m, uint32_t white, uint32_t black, float hys_frac, uint32_t n_levels)
{
    if (n_levels < 1u)
    {
        n_levels = 1u;
    }
    if (n_levels > BCM_MAX_LEVELS)
    {
        n_levels = BCM_MAX_LEVELS;
    }
    p_m->n_levels   = n_levels;
    p_m->b_pass     = false;
    p_m->b_settle   = false;
    p_m->settle_um  = 0u;
    p_m->fails      = 0u;
    p_m->used       = -1;
    p_m->b_conflict = false;
    for (uint32_t i = 0u; i < BCM_MAX_LEVELS; i++)
    {
        det_init_level(&p_m->det[i], white, black, hys_frac, LEVEL[i]);
        bc_reset(&p_m->bc[i]);
        p_m->res[i]       = BC_NONE;
        p_m->code[i]      = 0;
        p_m->rev[i]       = false;
        p_m->last_res[i]  = BC_NONE;
        p_m->last_code[i] = 0;
        p_m->last_n[i]    = 0u;
    }
}

bool bcm_open(const bcm_t *p_m)
{
    for (uint32_t i = 0u; i < p_m->n_levels; i++)
    {
        if (bc_window_open(&p_m->bc[i]))
        {
            return true;
        }
    }
    return false;
}

static void note(bcm_t *p_m, uint32_t i, bc_result_t r, char code, bool b_rev, uint32_t pos_um)
{
    if ((r != BC_NONE) && (p_m->res[i] == BC_NONE))
    {
        if ((r == BC_DECODED) && !p_m->b_settle)
        {
            p_m->b_settle  = true;
            p_m->settle_um = pos_um + BCM_SETTLE_UM;
        }
        p_m->res[i]  = (r == BC_DECODED) ? BC_DECODED : BC_FAILED;   /* faults are counted per pass */
        p_m->code[i] = code;
        p_m->rev[i]  = b_rev;
    }
}

/* every level's window has closed: one result for the whole pass */
static bc_result_t finish_pass(bcm_t *p_m, char *p_code, bool *p_rev)
{
    bc_result_t out = BC_NONE;
    bool        b_failed = false;
    int32_t     first = -1;

    p_m->b_conflict = false;
    for (uint32_t i = 0u; i < p_m->n_levels; i++)
    {
        if (p_m->res[i] == BC_DECODED)
        {
            if (first < 0)
            {
                first = (int32_t)i;
            }
            else if (p_m->code[i] != p_m->code[first])
            {
                p_m->b_conflict = true;
            }
            else
            {
                /* agrees */
            }
        }
        else if (p_m->res[i] == BC_FAILED)
        {
            b_failed = true;
        }
        else
        {
            /* stray mark or nothing at this level */
        }
    }
    if ((first >= 0) && !p_m->b_conflict)
    {
        *p_code  = p_m->code[first];
        *p_rev   = p_m->rev[first];
        p_m->fails = 0u;
        out = BC_DECODED;
    }
    else if ((first >= 0) || b_failed)
    {
        p_m->fails++;
        if (p_m->fails >= BC_FAULT_COUNT)
        {
            p_m->fails = 0u;
            out = BC_FAULT;
        }
        else
        {
            out = BC_FAILED;
        }
    }
    else
    {
        /* only stray marks */
    }
    p_m->used = ((first >= 0) && !p_m->b_conflict) ? first : -1;
    for (uint32_t i = 0u; i < BCM_MAX_LEVELS; i++)
    {
        p_m->last_res[i]  = p_m->res[i];
        p_m->last_code[i] = p_m->code[i];
        p_m->last_n[i]    = p_m->bc[i].last_n;
        p_m->res[i]       = BC_NONE;
        p_m->code[i]      = 0;
    }
    p_m->b_pass   = false;
    p_m->b_settle = false;
    return out;
}

bc_result_t bcm_sample(bcm_t *p_m, const bc_params_t *p_par, uint32_t pos_um, uint32_t adc,
                       char *p_code, bool *p_rev)
{
    for (uint32_t i = 0u; i < p_m->n_levels; i++)
    {
        det_edge_t  e[DET_MAX_OUT];
        uint32_t    n = det_sample(&p_m->det[i], pos_um, adc, e);
        char        code = 0;
        bool        b_rev = false;
        bc_result_t r;

        for (uint32_t q = 0u; q < n; q++)
        {
            bool b_was_open = bc_window_open(&p_m->bc[i]);

            r = bc_edge(&p_m->bc[i], p_par, e[q].pos_um, e[q].b_black, &code, &b_rev);
            note(p_m, i, r, code, b_rev, pos_um);
            if (!b_was_open && bc_window_open(&p_m->bc[i]) && !p_m->b_pass)
            {
                /* a new pass starts: forget the element counts of the last one */
                p_m->b_pass = true;
                for (uint32_t k = 0u; k < BCM_MAX_LEVELS; k++)
                {
                    p_m->bc[k].last_n = 0u;
                }
            }
        }
        r = bc_poll(&p_m->bc[i], p_par, pos_um, &code, &b_rev);
        note(p_m, i, r, code, b_rev, pos_um);
    }
    if (p_m->b_pass && p_m->b_settle && ((int32_t)(pos_um - p_m->settle_um) >= 0))
    {
        for (uint32_t i = 0u; i < p_m->n_levels; i++)
        {
            bc_cancel(&p_m->bc[i]);          /* still open: no result from this level */
        }
    }
    if (p_m->b_pass && !bcm_open(p_m))
    {
        return finish_pass(p_m, p_code, p_rev);
    }
    return BC_NONE;
}

void bcm_cancel(bcm_t *p_m)
{
    for (uint32_t i = 0u; i < BCM_MAX_LEVELS; i++)
    {
        bc_cancel(&p_m->bc[i]);
        p_m->res[i]  = BC_NONE;
        p_m->code[i] = 0;
    }
    p_m->b_pass   = false;
    p_m->b_settle = false;
}
