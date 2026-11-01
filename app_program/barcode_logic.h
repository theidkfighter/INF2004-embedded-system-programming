/**
 * @file    barcode_logic.h
 * @brief   Barcode window handling and decode (DR 6.1 INTERSECTION_DETECTED ->
 *          BARCODE_SAMPLING_WINDOW -> BARCODE_DECODE_EVAL).
 *
 * Input: black/white edges of the barcode sensor (GP26), each stamped with
 * the distance travelled (micrometres, from the wheel encoders). The edges
 * are found by a 1 kHz timer interrupt (hal_isr.c): the DR's 100 Hz is too
 * slow, a 3 mm narrow bar lasts 20 ms at 150 mm/s.
 *
 * Widths are measured in distance, not time, so a speed change while the bar
 * code passes does not upset the wide/narrow decision ("robust at varying
 * speeds"). The window opens on the first black bar, and closes when the
 * white gap is more than 5 x the narrowest element (the quiet zone) or the
 * window passes BARCODE_MAX_LEN_MM. Code 39 is decoded in both directions
 * (FR7). Windows with fewer than 9 elements (a crossing line under the
 * barcode sensor, a stain) are ignored silently.
 *
 * Pure logic: unit-tested on the PC (tests/).
 */
#ifndef BARCODE_LOGIC_H
#define BARCODE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#define BC_MAX_ELEMENTS   (96u)
#define BC_FAULT_COUNT    (3u)    /* DR: 3 ambiguous barcodes in a row -> fault */

typedef enum
{
    BC_NONE = 0,       /* nothing finished */
    BC_DECODED,        /* *code is valid */
    BC_FAILED,         /* a real barcode-sized window that did not decode */
    BC_FAULT           /* BC_FAILED for the 3rd time in a row */
} bc_result_t;

typedef struct
{
    uint32_t min_elements;    /* BARCODE_MIN_ELEMENTS */
    uint32_t max_len_um;      /* BARCODE_MAX_LEN_MM * 1000 */
    uint32_t stray_gap_um;    /* a short window ends after this much white */
    uint32_t max_gap_um;      /* a long window ends after this much white anyway */
} bc_params_t;

/* Change Requirement 1 (minimum bits), 212 bytes, was 512:
 *   w        U16 um per element (saturates at 65.535 mm; a Code 39 element on
 *            the course is 2-8 mm, so every real width is exact)
 *   (bar/space flags removed: a window opens on a black edge and edges
 *    alternate, so element i is a bar exactly when i is even)
 *   n, last_n U7 (0..96), fails U2 (0..BC_FAULT_COUNT-1), b_open, b_black U1
 *   start_um, edge_um U32: absolute path positions (the odometer path in um
 *            wraps only after 4295 m; widths are modulo-2^32 differences)
 *   min_um, max_um U32: kept exact, bc_poll() compares them with um gaps.
 * One task (TSK_LINE_BARCODE) owns every bc_ctx_t, so no Critical_Section. */
typedef struct
{
    uint16_t w[BC_MAX_ELEMENTS];
    uint32_t start_um;
    uint32_t edge_um;         /* position of the last edge */
    uint32_t min_um;          /* narrowest element so far */
    uint32_t max_um;          /* widest element so far */
    uint32_t n       : 7;
    uint32_t last_n  : 7;     /* elements in the window that was just closed (diagnostics) */
    uint32_t fails   : 2;
    uint32_t b_open  : 1;
    uint32_t b_black : 1;     /* current level */
} bc_ctx_t;

#define BC_W_MAX          (0xFFFFu)

void        bc_reset(bc_ctx_t *p_bc);
void        bc_cancel(bc_ctx_t *p_bc);      /* drop an open window (e.g. during a turn) */
bc_result_t bc_edge(bc_ctx_t *p_bc, const bc_params_t *p_par, uint32_t pos_um, bool b_black,
                    char *p_code, bool *p_reversed);
bc_result_t bc_poll(bc_ctx_t *p_bc, const bc_params_t *p_par, uint32_t pos_um,
                    char *p_code, bool *p_reversed);
bool        bc_window_open(const bc_ctx_t *p_bc);

/* Element i of the last window in 0.1 mm (BC_DBG widths). */
static inline uint32_t bc_width_01mm(const bc_ctx_t *p_bc, uint32_t i)
{
    return (p_bc->w[i] + 50u) / 100u;
}

#endif /* BARCODE_LOGIC_H */
