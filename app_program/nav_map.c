/**
 * @file    nav_map.c
 * @brief   Barcode letter -> movement mapping (see nav_map.h).
 */
#include "nav_map.h"

#include "bitpack.h"
#include "crit.h"

#define MAP_LETTERS   (26u)
#define MAP_WORDS     (3u)
#define MAP_PER_WORD  (10u)
#define MAP_ACT_W     (3u)                /* nav_action_t 0..4 (E3) */
#define MAP_SH(i)     (MAP_ACT_W * ((i) % MAP_PER_WORD))

BP_STATIC_ASSERT((MAP_PER_WORD * MAP_ACT_W) <= 32u, "g_map_w: ten actions fit in one word");
BP_STATIC_ASSERT((MAP_WORDS * MAP_PER_WORD) >= MAP_LETTERS, "g_map_w: every letter has a slot");
BP_STATIC_ASSERT((uint32_t)ACT_RIGHT <= BP_MASK(MAP_ACT_W), "g_map_w: nav_action_t fits in E3");

/* Letter i (0 = A) is in word i / 10, bits 3 * (i mod 10) .. + 2. Written by
   the telemetry task, read by the nav task. nav_map_get() is one load, so it
   gets the letter's whole action. A new map is built in a local first and its
   three words are stored in one dispatch-disabled section, and the whole-map
   readers copy the three words in one too, so a half-updated map is never
   seen (Requirement 9.7). */
volatile uint32_t g_map_w[MAP_WORDS];

static uint32_t map_word_get(uint32_t word, uint32_t i)
{
    return BP_GET(word, MAP_SH(i), MAP_ACT_W);
}

/* Replaces letter i: a letter given twice keeps its last action. */
static void map_put(uint32_t p_w[MAP_WORDS], uint32_t i, nav_action_t act)
{
    p_w[i / MAP_PER_WORD] = BP_PUT(p_w[i / MAP_PER_WORD], MAP_SH(i), MAP_ACT_W, (uint32_t)act);
}

static uint32_t map_at(const uint32_t p_w[MAP_WORDS], uint32_t i)
{
    return map_word_get(p_w[i / MAP_PER_WORD], i);
}

static uint32_t map_count(const uint32_t p_w[MAP_WORDS])
{
    uint32_t n = 0u;

    for (uint32_t i = 0u; i < MAP_LETTERS; i++)
    {
        n += (map_at(p_w, i) != (uint32_t)ACT_NONE) ? 1u : 0u;
    }
    return n;
}

static void map_store(const uint32_t p_w[MAP_WORDS])
{
    crit_dsp_enter();
    g_map_w[0] = p_w[0];
    g_map_w[1] = p_w[1];
    g_map_w[2] = p_w[2];
    crit_dsp_exit();
}

static void map_copy(uint32_t p_w[MAP_WORDS])
{
    crit_dsp_enter();
    p_w[0] = g_map_w[0];
    p_w[1] = g_map_w[1];
    p_w[2] = g_map_w[2];
    crit_dsp_exit();
}

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
    const uint32_t w[MAP_WORDS] = { 0u };

    map_store(w);
}

nav_action_t nav_map_get(char letter)
{
    uint32_t i;

    letter = up(letter);
    if ((letter < 'A') || (letter > 'Z'))
    {
        return ACT_NONE;
    }
    i = (uint32_t)(letter - 'A');
    return (nav_action_t)map_word_get(g_map_w[i / MAP_PER_WORD], i);
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
    uint32_t tmp[MAP_WORDS] = { 0u };
    uint32_t n_pos = 0u;
    uint32_t n_pairs = 0u;
    int32_t  count;
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
            map_put(tmp, (uint32_t)(letter - 'A'), act);
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
            map_put(tmp, (uint32_t)(letter - 'A'), POSITIONAL[n_pos]);
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
    count = (int32_t)map_count(tmp);
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
    map_store(tmp);
    return count;
}

uint32_t nav_map_actions(uint8_t p_act[26])
{
    uint32_t w[MAP_WORDS];

    map_copy(w);
    for (uint32_t i = 0u; i < MAP_LETTERS; i++)
    {
        p_act[i] = (uint8_t)map_at(w, i);
    }
    return map_count(w);
}

uint32_t nav_map_set_slots(const uint8_t p_present[4], const uint8_t p_letter[4])
{
    static const nav_action_t SLOT[4] = { ACT_FORWARD, ACT_LEFT, ACT_RIGHT, ACT_UTURN };   /* design D13 */
    uint32_t tmp[MAP_WORDS] = { 0u };

    for (uint32_t k = 0u; k < 4u; k++)
    {
        if ((p_present[k] != 0u) && (p_letter[k] < MAP_LETTERS))
        {
            map_put(tmp, p_letter[k], SLOT[k]);
        }
    }
    map_store(tmp);
    return map_count(tmp);
}

uint32_t nav_map_describe(char *p_buf, uint32_t size)
{
    uint32_t w[MAP_WORDS];
    uint32_t len = 0u;
    uint32_t n = 0u;

    if (size == 0u)
    {
        return 0u;
    }
    map_copy(w);
    for (uint32_t i = 0u; i < MAP_LETTERS; i++)
    {
        const char *p_name;

        if (map_at(w, i) == (uint32_t)ACT_NONE)
        {
            continue;
        }
        p_name = nav_action_name((nav_action_t)map_at(w, i));
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
