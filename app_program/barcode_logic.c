/**
 * @file    barcode_logic.c
 * @brief   Barcode window handling and decode (see barcode_logic.h).
 */
#include "barcode_logic.h"
#include "code39.h"

void bc_reset(bc_ctx_t *p_bc)
{
    p_bc->n        = 0u;
    p_bc->b_open   = false;
    p_bc->b_black  = false;
    p_bc->start_um = 0u;
    p_bc->edge_um  = 0u;
    p_bc->min_um   = 0xFFFFFFFFu;
    p_bc->max_um   = 0u;
    p_bc->fails    = 0u;
    p_bc->last_n   = 0u;
}

void bc_cancel(bc_ctx_t *p_bc)
{
    if (p_bc->b_open)
    {
        p_bc->last_n = p_bc->n;   /* w[] / bar[] keep the widths for a diagnostic dump */
    }
    p_bc->n      = 0u;
    p_bc->b_open = false;
    p_bc->min_um = 0xFFFFFFFFu;
    p_bc->max_um = 0u;
}

bool bc_window_open(const bc_ctx_t *p_bc)
{
    return p_bc->b_open;
}

/* Decodes the finished window and resets it. */
static bc_result_t finish(bc_ctx_t *p_bc, const bc_params_t *p_par, char *p_code, bool *p_reversed)
{
    char        text[8];
    bool        b_rev = false;
    bc_result_t res;

    if (p_bc->n < p_par->min_elements)
    {
        bc_cancel(p_bc);
        return BC_NONE;                      /* stray mark: ignore */
    }
    if (code39_decode(p_bc->w, p_bc->bar, p_bc->n, text, sizeof(text), &b_rev) && (text[0] != '\0'))
    {
        *p_code     = text[0];
        *p_reversed = b_rev;
        p_bc->fails = 0u;
        res = BC_DECODED;
    }
    else
    {
        p_bc->fails++;
        if (p_bc->fails >= BC_FAULT_COUNT)
        {
            p_bc->fails = 0u;
            res = BC_FAULT;
        }
        else
        {
            res = BC_FAILED;
        }
    }
    bc_cancel(p_bc);
    return res;
}

bc_result_t bc_edge(bc_ctx_t *p_bc, const bc_params_t *p_par, uint32_t pos_um, bool b_black,
                    char *p_code, bool *p_reversed)
{
    bc_result_t res = BC_NONE;

    if (b_black == p_bc->b_black)
    {
        return BC_NONE;                      /* not an edge */
    }
    if (!p_bc->b_open)
    {
        if (b_black)
        {
            /* first black bar: open the sampling window */
            p_bc->b_open   = true;
            p_bc->n        = 0u;
            p_bc->start_um = pos_um;
            p_bc->min_um   = 0xFFFFFFFFu;
            p_bc->max_um   = 0u;
        }
    }
    else
    {
        uint32_t width = pos_um - p_bc->edge_um;

        if (p_bc->n < BC_MAX_ELEMENTS)
        {
            p_bc->w[p_bc->n]   = width;
            p_bc->bar[p_bc->n] = p_bc->b_black;   /* the element that just ended */
            p_bc->n++;
            if ((width > 0u) && (width < p_bc->min_um))
            {
                p_bc->min_um = width;
            }
            if (width > p_bc->max_um)
            {
                p_bc->max_um = width;
            }
        }
        else
        {
            res = finish(p_bc, p_par, p_code, p_reversed);
        }
    }
    p_bc->b_black = b_black;
    p_bc->edge_um = pos_um;
    return res;
}

bc_result_t bc_poll(bc_ctx_t *p_bc, const bc_params_t *p_par, uint32_t pos_um,
                    char *p_code, bool *p_reversed)
{
    uint32_t gap;

    if (!p_bc->b_open)
    {
        return BC_NONE;
    }
    if ((pos_um - p_bc->start_um) > p_par->max_len_um)
    {
        return finish(p_bc, p_par, p_code, p_reversed);
    }
    if (p_bc->b_black)
    {
        return BC_NONE;                      /* still on a bar */
    }
    gap = pos_um - p_bc->edge_um;
    if (p_bc->n < p_par->min_elements)
    {
        if (gap > p_par->stray_gap_um)
        {
            bc_cancel(p_bc);                 /* a lone mark, not a barcode */
        }
        return BC_NONE;
    }
    /* Quiet zone: white wider than 2x the widest element seen so far (a wide
       element is 3 narrow, so a wide space can never end the window, even
       when sensor bleed shrinks the spaces) and at least 15 mm. */
    if ((gap > p_par->max_gap_um) ||
        ((gap > (2u * p_bc->max_um)) && (gap > 15000u)))
    {
        return finish(p_bc, p_par, p_code, p_reversed);
    }
    return BC_NONE;
}
