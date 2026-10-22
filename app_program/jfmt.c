/**
 * @file    jfmt.c
 * @brief   Small, allocation-free text/JSON builder (see jfmt.h).
 */
#include "jfmt.h"

void jb_init(jbuf_t *p_jb, char *p_buf, uint32_t size)
{
    p_jb->p_buf      = p_buf;
    p_jb->size       = size;
    p_jb->len        = 0u;
    p_jb->b_overflow = false;
    if (size > 0u)
    {
        p_buf[0] = '\0';
    }
}

void jb_char(jbuf_t *p_jb, char c)
{
    if ((p_jb->len + 1u) < p_jb->size)
    {
        p_jb->p_buf[p_jb->len] = c;
        p_jb->len++;
        p_jb->p_buf[p_jb->len] = '\0';
    }
    else
    {
        p_jb->b_overflow = true;
    }
}

void jb_raw(jbuf_t *p_jb, const char *p_text)
{
    while (*p_text != '\0')
    {
        jb_char(p_jb, *p_text);
        p_text++;
    }
}

void jb_uint(jbuf_t *p_jb, uint32_t value)
{
    char     digits[10];
    uint32_t n = 0u;

    do
    {
        digits[n] = (char)('0' + (value % 10u));
        n++;
        value /= 10u;
    } while ((value > 0u) && (n < sizeof(digits)));

    while (n > 0u)
    {
        n--;
        jb_char(p_jb, digits[n]);
    }
}

void jb_int(jbuf_t *p_jb, int32_t value)
{
    if (value < 0)
    {
        jb_char(p_jb, '-');
        jb_uint(p_jb, (uint32_t)(-(value + 1)) + 1u);   /* safe for INT32_MIN */
    }
    else
    {
        jb_uint(p_jb, (uint32_t)value);
    }
}

void jb_fixed(jbuf_t *p_jb, float value, uint32_t decimals)
{
    uint32_t scale = 1u;
    uint32_t whole;
    uint32_t frac;
    float    mag;

    if (decimals > 4u)
    {
        decimals = 4u;
    }
    for (uint32_t i = 0u; i < decimals; i++)
    {
        scale *= 10u;
    }
    if (value != value)                     /* NaN */
    {
        jb_raw(p_jb, "null");
        return;
    }
    mag = (value < 0.0f) ? -value : value;
    if (mag > (4.0e9f / (float)scale))
    {
        mag = 4.0e9f / (float)scale;        /* keep the scaled value inside uint32 */
    }
    /* round to the last printed digit */
    {
        uint32_t total = (uint32_t)((mag * (float)scale) + 0.5f);

        whole = total / scale;
        frac  = total % scale;
    }
    if ((value < 0.0f) && ((whole > 0u) || (frac > 0u)))
    {
        jb_char(p_jb, '-');
    }
    jb_uint(p_jb, whole);
    if (decimals > 0u)
    {
        uint32_t div = scale / 10u;

        jb_char(p_jb, '.');
        while (div > 0u)
        {
            jb_char(p_jb, (char)('0' + ((frac / div) % 10u)));
            div /= 10u;
        }
    }
}

void jb_str(jbuf_t *p_jb, const char *p_text)
{
    jb_char(p_jb, '"');
    while (*p_text != '\0')
    {
        char c = *p_text;

        if ((c == '"') || (c == '\\'))
        {
            jb_char(p_jb, '\\');
            jb_char(p_jb, c);
        }
        else if ((unsigned char)c < 0x20u)
        {
            jb_char(p_jb, ' ');
        }
        else
        {
            jb_char(p_jb, c);
        }
        p_text++;
    }
    jb_char(p_jb, '"');
}

void jb_key(jbuf_t *p_jb, const char *p_key)
{
    jb_str(p_jb, p_key);
    jb_char(p_jb, ':');
}

uint32_t jb_len(const jbuf_t *p_jb)
{
    return p_jb->len;
}

bool jf_parse_float(const char *p_text, float *p_value)
{
    float sign = 1.0f;
    float val = 0.0f;
    float div = 1.0f;
    bool  b_digit = false;
    bool  b_frac = false;

    while (*p_text == ' ')
    {
        p_text++;
    }
    if (*p_text == '-')
    {
        sign = -1.0f;
        p_text++;
    }
    else if (*p_text == '+')
    {
        p_text++;
    }
    else
    {
        /* no sign */
    }
    while (*p_text != '\0')
    {
        char c = *p_text;

        if ((c >= '0') && (c <= '9'))
        {
            b_digit = true;
            if (b_frac)
            {
                div *= 10.0f;
                val += (float)(c - '0') / div;
            }
            else
            {
                val = (val * 10.0f) + (float)(c - '0');
            }
        }
        else if ((c == '.') && !b_frac)
        {
            b_frac = true;
        }
        else if ((c == ' ') || (c == '\r') || (c == '\n'))
        {
            break;
        }
        else
        {
            return false;
        }
        p_text++;
    }
    if (b_digit)
    {
        *p_value = sign * val;
    }
    return b_digit;
}

uint32_t jf_strlen(const char *p_text)
{
    uint32_t n = 0u;

    while (p_text[n] != '\0')
    {
        n++;
    }
    return n;
}

bool jf_starts_with(const char *p_text, const char *p_prefix)
{
    while (*p_prefix != '\0')
    {
        if (*p_text != *p_prefix)
        {
            return false;
        }
        p_text++;
        p_prefix++;
    }
    return true;
}

void jf_copy(char *p_dst, const char *p_src, uint32_t size)
{
    uint32_t i = 0u;

    if (size == 0u)
    {
        return;
    }
    while ((p_src[i] != '\0') && ((i + 1u) < size))
    {
        p_dst[i] = p_src[i];
        i++;
    }
    p_dst[i] = '\0';
}
