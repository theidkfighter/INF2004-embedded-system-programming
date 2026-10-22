/**
 * @file    jfmt.h
 * @brief   Small, allocation-free text/JSON builder.
 *
 * The robot does not link the C library's printf family (it needs a heap and
 * soft-float conversion code); telemetry is built with these helpers instead.
 */
#ifndef JFMT_H
#define JFMT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    char    *p_buf;
    uint32_t size;
    uint32_t len;
    bool     b_overflow;
} jbuf_t;

void     jb_init(jbuf_t *p_jb, char *p_buf, uint32_t size);
void     jb_raw(jbuf_t *p_jb, const char *p_text);         /* copied as is */
void     jb_char(jbuf_t *p_jb, char c);
void     jb_int(jbuf_t *p_jb, int32_t value);
void     jb_uint(jbuf_t *p_jb, uint32_t value);
void     jb_fixed(jbuf_t *p_jb, float value, uint32_t decimals); /* e.g. 12.3 */
void     jb_str(jbuf_t *p_jb, const char *p_text);         /* "quoted", escaped */
uint32_t jb_len(const jbuf_t *p_jb);

/* "key": helpers (no leading comma) */
void     jb_key(jbuf_t *p_jb, const char *p_key);

/* Parses "[-]digits[.digits]" (no exponent). Returns false on bad input. */
bool     jf_parse_float(const char *p_text, float *p_value);

uint32_t jf_strlen(const char *p_text);
bool     jf_starts_with(const char *p_text, const char *p_prefix);
void     jf_copy(char *p_dst, const char *p_src, uint32_t size);   /* always terminated */

#endif /* JFMT_H */
