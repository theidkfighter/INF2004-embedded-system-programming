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
uint32_t jb_len(const jbuf_t *p_jb);

/* "key": helpers (no leading comma) */
void     jb_key(jbuf_t *p_jb, const char *p_key);

#endif /* JFMT_H */
