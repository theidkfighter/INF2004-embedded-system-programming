/**
 * @file    bc_multi.h
 * @brief   Barcode reading at several switch levels at once (BARCODE_LEVELS).
 *
 * A faded or grey print (inkjet ink that reflects IR, a dusty sensor,
 * glare) does not reach the mid level between white and black: its narrow
 * bars or narrow spaces then vanish and the window does not decode. Up to 5
 * copies of the bar detector (bc_detect.c) and of the Code 39 window
 * (barcode_logic.c) run side by side on the same samples, switching at
 *     50 %, 35 %, 65 %, 25 %, 75 %   of the way from white to black.
 *
 * A "pass" lasts from the first window opening at any level until every
 * level's window has closed, or BCM_SETTLE_UM (20 mm) after the first level
 * DECODED (a faint level can merge bars into one wide element and would
 * otherwise keep its window open for up to 60 mm of white, delaying the
 * letter past a junction close behind the barcode). Then:
 *   - one or more levels decoded:   the first one in the order above is used,
 *                                   if all decoded levels agree on the letter
 *                                   (a disagreement counts as a failed read);
 *   - none decoded, one failed:     one failed read (BC_FAILED / BC_FAULT on
 *                                   the 3rd in a row, as before);
 *   - only stray marks:             nothing.
 * Every level still needs a full *X* (both start / stop characters), so the
 * protection against a wrong letter is the same as with one level.
 *
 * Simulated (26 letters x 2 directions, print 60 % as dark as the calibrated
 * black, sensor blur sigma 2 mm): big print 3 -> 52 of 52 decoded, medium
 * 0 -> 52; sharp full-black prints are read the same as with one level.
 *
 * Pure logic: unit-tested on the PC (tests/), shared with part_tests.
 */
#ifndef BC_MULTI_H
#define BC_MULTI_H

#include <stdbool.h>
#include <stdint.h>
#include "bc_detect.h"
#include "barcode_logic.h"

#define BCM_MAX_LEVELS  (5u)
#define BCM_SETTLE_UM   (20000u)   /* after the first level decodes, wait at most this far for the others */

typedef struct
{
    uint32_t    n_levels;                 /* 1, 3 or 5 in use */
    det_t       det[BCM_MAX_LEVELS];
    bc_ctx_t    bc[BCM_MAX_LEVELS];
    bc_result_t res[BCM_MAX_LEVELS];      /* this pass, per level */
    char        code[BCM_MAX_LEVELS];
    bool        rev[BCM_MAX_LEVELS];
    bool        b_pass;
    bool        b_settle;                 /* a level has finished: deadline running */
    uint32_t    settle_um;
    uint32_t    fails;                    /* failed passes in a row */
    /* the pass that just finished (diagnostics) */
    int32_t     used;                     /* level index that gave the letter, -1 = none */
    bool        b_conflict;               /* levels decoded different letters */
    bc_result_t last_res[BCM_MAX_LEVELS];
    char        last_code[BCM_MAX_LEVELS];
    uint32_t    last_n[BCM_MAX_LEVELS];   /* elements per level */
} bcm_t;

/* level_pct(i): switch level of copy i in percent (50, 35, 65, 25, 75) */
uint32_t    bcm_level_pct(uint32_t i);

/* white / black: IR_WHITE_ADC / IR_BLACK_ADC; hys_frac: BARCODE_DET_HYS;
   n_levels: BARCODE_LEVELS (clamped to 1..5) */
void        bcm_init(bcm_t *p_m, uint32_t white, uint32_t black, float hys_frac, uint32_t n_levels);

/* One ADC sample at position pos_um (never decreasing). Returns BC_NONE
   until a pass finishes, then BC_DECODED (*p_code / *p_rev), BC_FAILED or
   BC_FAULT. */
bc_result_t bcm_sample(bcm_t *p_m, const bc_params_t *p_par, uint32_t pos_um, uint32_t adc,
                       char *p_code, bool *p_rev);

/* Drop every open window without a result (e.g. during a turn). */
void        bcm_cancel(bcm_t *p_m);

bool        bcm_open(const bcm_t *p_m);

#endif /* BC_MULTI_H */
