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

/* Bars and spaces are classified separately: every letter, digit and '*'
   has exactly 2 wide bars (of 5) and 1 wide space (of 4). Comparing bars
   only with bars and spaces only with spaces cancels the IR sensor's "ink
   bleed" (bars read wider and spaces narrower by the spot size), which
   breaks a plain "3 widest of 9" rule once the bleed reaches ~1.5 mm. */
static char char_bars_spaces(const uint16_t *p_w9)
{
    char     pat[10] = "nnnnnnnnn";
    uint32_t b1 = 0u, b2 = 2u, s1 = 1u;

    if (p_w9[b2] > p_w9[b1]) { b1 = 2u; b2 = 0u; }
    for (uint32_t i = 4u; i <= 8u; i += 2u)
    {
        if (p_w9[i] > p_w9[b1])      { b2 = b1; b1 = i; }
        else if (p_w9[i] > p_w9[b2]) { b2 = i; }
    }
    for (uint32_t i = 3u; i <= 7u; i += 2u)
    {
        if (p_w9[i] > p_w9[s1]) { s1 = i; }
    }
    /* contrast checks inside each group: wide >= 1.5 x widest narrow */
    {
        uint32_t nb = 0u, ns = 0u;

        for (uint32_t i = 0u; i <= 8u; i += 2u)
        {
            if ((i != b1) && (i != b2) && (p_w9[i] > nb)) { nb = p_w9[i]; }
        }
        for (uint32_t i = 1u; i <= 7u; i += 2u)
        {
            if ((i != s1) && (p_w9[i] > ns)) { ns = p_w9[i]; }
        }
        if (((p_w9[b2] * 2u) < (nb * 3u)) || ((p_w9[s1] * 2u) < (ns * 3u)))
        {
            return 0;
        }
    }
    pat[b1] = 'w';
    pat[b2] = 'w';
    pat[s1] = 'w';
    for (size_t i = 0u; i < (sizeof(TABLE) / sizeof(TABLE[0])); i++)
    {
        if (strcmp(pat, TABLE[i].p_pat) == 0)
        {
            return TABLE[i].ch;
        }
    }
    return 0;
}

char code39_char(const uint16_t *p_w9)
{
    {
        char c = char_bars_spaces(p_w9);

        if (c != 0)
        {
            return c;
        }
    }
    /* fall back to "3 widest of 9" ($ / + % have no wide bar) */
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

/* Index order of the Data_Representation_Table (Requirement 5.3). */
static const char C39_CHARS[C39_COUNT + 1u] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

uint32_t c39_to_index(char c)
{
    for (uint32_t i = 0u; i < C39_COUNT; i++)
    {
        if (C39_CHARS[i] == c)
        {
            return i;
        }
    }
    return C39_NONE;
}

char c39_from_index(uint32_t idx)
{
    return (idx < C39_COUNT) ? C39_CHARS[idx] : (char)0;
}

static bool decode_one_way(const uint16_t *p_w, bool b_first_bar, uint32_t n,
                           char *p_out, size_t out_len)
{
    for (uint32_t start = 0u; (start + 9u) <= n; start++)
    {
        uint32_t pos;
        size_t   len = 0u;

        bool b_bar = b_first_bar ? ((start & 1u) == 0u) : ((start & 1u) != 0u);

        if (!b_bar || (code39_char(&p_w[start]) != '*'))
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

bool code39_decode(const uint16_t *p_w, bool b_first_bar, uint32_t n,
                   char *p_out, size_t out_len, bool *p_reversed)
{
    static uint16_t rw[128];

    if ((out_len < 2u) || (n > 128u))
    {
        return false;
    }
    if (decode_one_way(p_w, b_first_bar, n, p_out, out_len))
    {
        *p_reversed = false;
        return true;
    }
    for (uint32_t i = 0u; i < n; i++)
    {
        rw[i] = p_w[n - 1u - i];
    }
    /* reversed, element 0 is the old element n-1 */
    if (decode_one_way(rw, (((n - 1u) & 1u) == 0u) ? b_first_bar : !b_first_bar, n, p_out, out_len))
    {
        *p_reversed = true;
        return true;
    }
    return false;
}
