/**
 * @file    nav_map.c
 * @brief   Barcode letter -> movement mapping (see nav_map.h).
 */
#include "nav_map.h"

/* One byte per letter: written by the telemetry task, read by the nav task.
   Single-byte stores are atomic on the RP2040; a whole-map update is built
   in a scratch copy first and then copied, so a half-parsed map is never
   used for more than one letter. */
static volatile uint8_t g_map[26];

static char up(char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - 'a' + 'A') : c;
}

static bool is_sep(char c)
{
    return (c == ',') || (c == ';') || (c == ' ') || (c == '\t') || (c == '\n') || (c == '\r');
}

/* Compares the word [p, p+n) with p_word, ignoring case and '-' / '_'. */
static bool word_is(const char *p, uint32_t n, const char *p_word)
{
    uint32_t i = 0u;

    for (uint32_t k = 0u; k < n; k++)
    {
        char c = up(p[k]);

        if ((c == '-') || (c == '_'))
        {
            continue;
        }
        if ((p_word[i] == '\0') || (p_word[i] != c))
        {
            return false;
        }
        i++;
    }
    return p_word[i] == '\0';
}

static nav_action_t parse_action(const char *p, uint32_t n)
{
    if (word_is(p, n, "FWD") || word_is(p, n, "FORWARD") || word_is(p, n, "STRAIGHT") || word_is(p, n, "F"))
    {
        return ACT_FORWARD;
    }
    if (word_is(p, n, "LEFT") || word_is(p, n, "L"))
    {
        return ACT_LEFT;
    }
    if (word_is(p, n, "RIGHT") || word_is(p, n, "R"))
    {
        return ACT_RIGHT;
    }
    if (word_is(p, n, "UTURN") || word_is(p, n, "U") || word_is(p, n, "BACK"))
    {
        return ACT_UTURN;
    }
    return ACT_NONE;
}

static void set_err(char *p_err, uint32_t size, const char *p_msg)
{
    uint32_t i = 0u;

    if ((p_err == 0) || (size == 0u))
    {
        return;
    }
    while ((p_msg[i] != '\0') && ((i + 1u) < size))
    {
        p_err[i] = p_msg[i];
        i++;
    }
    p_err[i] = '\0';
}

void nav_map_clear(void)
{
    for (uint32_t i = 0u; i < 26u; i++)
    {
        g_map[i] = (uint8_t)ACT_NONE;
    }
}

nav_action_t nav_map_get(char letter)
{
    letter = up(letter);
    if ((letter < 'A') || (letter > 'Z'))
    {
        return ACT_NONE;
    }
    return (nav_action_t)g_map[letter - 'A'];
}

float nav_action_deg(nav_action_t act)
{
    switch (act)
    {
        case ACT_LEFT:  return -90.0f;
        case ACT_RIGHT: return 90.0f;
        case ACT_UTURN: return 180.0f;
        default:        return 0.0f;
    }
}

const char *nav_action_name(nav_action_t act)
{
    switch (act)
    {
        case ACT_FORWARD: return "FWD";
        case ACT_UTURN:   return "UTURN";
        case ACT_LEFT:    return "LEFT";
        case ACT_RIGHT:   return "RIGHT";
        default:          return "NONE";
    }
}

int32_t nav_map_parse(const char *p_text, char *p_err, uint32_t err_size)
{
    static const nav_action_t POSITIONAL[4] = { ACT_FORWARD, ACT_UTURN, ACT_LEFT, ACT_RIGHT };
    uint8_t  tmp[26] = { 0u };
    uint32_t n_pos = 0u;
    uint32_t n_pairs = 0u;
    int32_t  count = 0;
    const char *p = p_text;

    if ((p[0] == 'M') && (p[1] == 'A') && (p[2] == 'P') && (p[3] == ':'))
    {
        p = &p[4];
    }
    while (is_sep(*p))
    {
        p++;
    }
    if (word_is(p, 5u, "CLEAR"))
    {
        nav_map_clear();
        return 0;
    }

    while (*p != '\0')
    {
        const char *p_item;
        uint32_t    n = 0u;
        char        letter;

        while (is_sep(*p))
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        p_item = p;
        while ((*p != '\0') && !is_sep(*p))
        {
            p++;
            n++;
        }
        letter = up(p_item[0]);
        if ((letter < 'A') || (letter > 'Z'))
        {
            set_err(p_err, err_size, "letters must be A-Z");
            return -1;
        }
        if ((n >= 3u) && (p_item[1] == '='))
        {
            /* "K=LEFT" */
            nav_action_t act = parse_action(&p_item[2], n - 2u);

            if (act == ACT_NONE)
            {
                set_err(p_err, err_size, "unknown movement (use FWD, LEFT, RIGHT, UTURN)");
                return -1;
            }
            tmp[letter - 'A'] = (uint8_t)act;
            n_pairs++;
        }
        else if (n == 1u)
        {
            /* positional "F,U,K,R" */
            if (n_pos >= 4u)
            {
                set_err(p_err, err_size, "positional form takes exactly 4 letters");
                return -1;
            }
            tmp[letter - 'A'] = (uint8_t)POSITIONAL[n_pos];
            n_pos++;
        }
        else
        {
            set_err(p_err, err_size, "expected L=ACTION or a single letter");
            return -1;
        }
    }

    if ((n_pos > 0u) && (n_pairs > 0u))
    {
        set_err(p_err, err_size, "do not mix the two forms");
        return -1;
    }
    if ((n_pos > 0u) && (n_pos != 4u))
    {
        set_err(p_err, err_size, "positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT");
        return -1;
    }
    for (uint32_t i = 0u; i < 26u; i++)
    {
        if (tmp[i] != (uint8_t)ACT_NONE)
        {
            count++;
        }
    }
    if (count == 0)
    {
        set_err(p_err, err_size, "empty map");
        return -1;
    }
    if ((n_pos == 4u) && (count != 4))
    {
        set_err(p_err, err_size, "the same letter was given twice");
        return -1;
    }
    for (uint32_t i = 0u; i < 26u; i++)
    {
        g_map[i] = tmp[i];
    }
    return count;
}

uint32_t nav_map_describe(char *p_buf, uint32_t size)
{
    uint32_t len = 0u;
    uint32_t n = 0u;

    if (size == 0u)
    {
        return 0u;
    }
    for (uint32_t i = 0u; i < 26u; i++)
    {
        const char *p_name;

        if (g_map[i] == (uint8_t)ACT_NONE)
        {
            continue;
        }
        p_name = nav_action_name((nav_action_t)g_map[i]);
        if ((n > 0u) && ((len + 1u) < size))
        {
            p_buf[len++] = ',';
        }
        if ((len + 2u) < size)
        {
            p_buf[len++] = (char)('A' + i);
            p_buf[len++] = '=';
        }
        while ((*p_name != '\0') && ((len + 1u) < size))
        {
            p_buf[len++] = *p_name++;
        }
        n++;
    }
    p_buf[len] = '\0';
    return n;
}
