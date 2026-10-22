/**
 * @file    code39.c
 * @brief   Code 39 decoder (see code39.h).
 */
#include "code39.h"

#include <string.h>

/* Code 39 table: 9 elements bar/space/bar/..., 'w' = wide, 'n' = narrow. */
static const struct
{
    char        ch;
    const char *p_pat;
} TABLE[] =
{
    { '0', "nnnwwnwnn" }, { '1', "wnnwnnnnw" }, { '2', "nnwwnnnnw" }, { '3', "wnwwnnnnn" },
    { '4', "nnnwwnnnw" }, { '5', "wnnwwnnnn" }, { '6', "nnwwwnnnn" }, { '7', "nnnwnnwnw" },
    { '8', "wnnwnnwnn" }, { '9', "nnwwnnwnn" }, { 'A', "wnnnnwnnw" }, { 'B', "nnwnnwnnw" },
    { 'C', "wnwnnwnnn" }, { 'D', "nnnnwwnnw" }, { 'E', "wnnnwwnnn" }, { 'F', "nnwnwwnnn" },
    { 'G', "nnnnnwwnw" }, { 'H', "wnnnnwwnn" }, { 'I', "nnwnnwwnn" }, { 'J', "nnnnwwwnn" },
    { 'K', "wnnnnnnww" }, { 'L', "nnwnnnnww" }, { 'M', "wnwnnnnwn" }, { 'N', "nnnnwnnww" },
    { 'O', "wnnnwnnwn" }, { 'P', "nnwnwnnwn" }, { 'Q', "nnnnnnwww" }, { 'R', "wnnnnnwwn" },
    { 'S', "nnwnnnwwn" }, { 'T', "nnnnwnwwn" }, { 'U', "wwnnnnnnw" }, { 'V', "nwwnnnnnw" },
    { 'W', "wwwnnnnnn" }, { 'X', "nwnnwnnnw" }, { 'Y', "wwnnwnnnn" }, { 'Z', "nwwnwnnnn" },
    { '-', "nwnnnnwnw" }, { '.', "wwnnnnwnn" }, { ' ', "nwwnnnwnn" }, { '*', "nwnnwnwnn" },
    { '$', "nwnwnwnnn" }, { '/', "nwnwnnnwn" }, { '+', "nwnnnwnwn" }, { '%', "nnnwnwnwn" },
};

char code39_char(const uint32_t *p_w9)
{
    char     pat[10];
    uint32_t idx[9];

    /* mark the 3 widest as wide */
    for (uint32_t i = 0u; i < 9u; i++)
    {
        idx[i] = i;
        pat[i] = 'n';
    }
    for (uint32_t k = 0u; k < 3u; k++)
    {
        for (uint32_t j = k + 1u; j < 9u; j++)
        {
            if (p_w9[idx[j]] > p_w9[idx[k]])
            {
                uint32_t t = idx[k];

                idx[k] = idx[j];
                idx[j] = t;
            }
        }
        pat[idx[k]] = 'w';
    }
    pat[9] = '\0';

    /* the 3rd-widest must clearly be wider than the 4th (rejects noise) */
    {
        uint32_t third = p_w9[idx[2]];
        uint32_t fourth = 0u;

        for (uint32_t i = 0u; i < 9u; i++)
        {
            if ((pat[i] == 'n') && (p_w9[i] > fourth))
            {
                fourth = p_w9[i];
            }
        }
        if ((third * 2u) < (fourth * 3u))     /* wide must be >= 1.5 x widest narrow */
        {
            return 0;
        }
    }

    for (size_t i = 0u; i < (sizeof(TABLE) / sizeof(TABLE[0])); i++)
    {
        if (strcmp(pat, TABLE[i].p_pat) == 0)
        {
            return TABLE[i].ch;
        }
    }
    return 0;
}

static bool decode_one_way(const uint32_t *p_w, const bool *p_is_bar, uint32_t n,
                           char *p_out, size_t out_len)
{
    for (uint32_t start = 0u; (start + 9u) <= n; start++)
    {
        uint32_t pos;
        size_t   len = 0u;

        if (!p_is_bar[start] || (code39_char(&p_w[start]) != '*'))
        {
            continue;
        }
        pos = start + 10u;                    /* 9 elements + 1 inter-character gap */
        while ((pos + 9u) <= n)
        {
            char c = code39_char(&p_w[pos]);

            if (c == 0)
            {
                break;
            }
            if (c == '*')
            {
                if (len > 0u)
                {
                    p_out[len] = '\0';
                    return true;
                }
                break;
            }
            if (len < (out_len - 1u))
            {
                p_out[len] = c;
                len++;
            }
            pos += 10u;
        }
    }
    return false;
}

bool code39_decode(const uint32_t *p_w, const bool *p_is_bar, uint32_t n,
                   char *p_out, size_t out_len, bool *p_reversed)
{
    static uint32_t rw[128];
    static bool     rb[128];

    if ((out_len < 2u) || (n > 128u))
    {
        return false;
    }
    if (decode_one_way(p_w, p_is_bar, n, p_out, out_len))
    {
        *p_reversed = false;
        return true;
    }
    for (uint32_t i = 0u; i < n; i++)
    {
        rw[i] = p_w[n - 1u - i];
        rb[i] = p_is_bar[n - 1u - i];
    }
    if (decode_one_way(rw, rb, n, p_out, out_len))
    {
        *p_reversed = true;
        return true;
    }
    return false;
}
