/**
 * @file    bc_detect.h
 * @brief   Barcode bar detector on distance bins (BARCODE_DETECT = 1).
 *
 * The 1 kHz interrupt used to decide black / white on every raw ADC sample
 * with two switch levels (mid +/- BARCODE_HYS x span), and the edge was
 * where a level was crossed. With a blurred sensor spot (sensor a few mm
 * too high, or a 75 % / 50 % print) the narrow bars only just reach the mid
 * level, so:
 *   - the hysteresis band has to stay wide enough for the raw ADC noise,
 *     and then the faint narrow bars never cross it (3-5 elements instead
 *     of 29, nothing decoded);
 *   - the edge lands on the black or white level instead of the mid level,
 *     which makes every bar narrower and every space wider (or the
 *     reverse) by the hysteresis.
 *
 * This detector:
 *   1. averages the samples into 0.25 mm bins of encoder distance (speed and
 *      stops do not matter, noise drops by the number of samples per bin),
 *      then a 1-2-1 smoothing over neighbouring bins;
 *   2. decides black / white with a narrow band (BARCODE_DET_HYS x span,
 *      0.02 by default, which the averaging allows; part test B2 suggests a
 *      value from the measured noise);
 *   3. puts each edge where the signal crossed the MID level (interpolated
 *      between bins), so blur and the band do not shift the widths.
 *
 * Simulated (26 letters x 2 directions, 3 % ADC noise), all 52 decode with
 * sensor blur sigma up to: 100 % print 1.8 mm (old ISR method 1.2 mm),
 * 75 % print 1.2 mm, 50 % print 0.9 mm. Beyond that the narrow bars are
 * physically gone from the signal and only a sharper sensor spot helps
 * (mount 2-5 mm above the paper, black tube / 1-2 mm slit).
 *
 * Output: the same edges the interrupt produced (position in um, new
 * level), so barcode_logic.c / code39.c are used unchanged.
 * Pure logic: unit-tested on the PC and shared with part_tests/test_barcode.
 */
#ifndef BC_DETECT_H
#define BC_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#define DET_BIN_UM      (250u)    /* 0.25 mm bins */
#define DET_HYS_MIN     (8)       /* smallest band half-width (ADC counts) */
#define DET_MAX_OUT     (4u)      /* edges one call can return */

typedef struct
{
    uint32_t pos_um;
    bool     b_black;             /* level after the edge */
} det_edge_t;

typedef struct
{
    /* configuration */
    bool     b_inv;               /* black reads lower: work on 4095 - adc */
    int32_t  mid;                 /* darker-is-higher scale */
    int32_t  hys;
    /* current bin */
    uint32_t bin_idx;
    uint32_t sum;
    uint32_t cnt;
    bool     b_started;
    /* smoothing */
    int32_t  raw0;
    int32_t  raw1;
    uint32_t raw_n;
    /* level decision */
    bool     b_black;
    bool     b_have_prev;
    int32_t  prev_s;              /* previous smoothed bin */
    uint32_t prev_k;
    uint32_t cross_um;            /* last mid crossing */
    /* diagnostics: darkest point of each bar, lightest of each space */
    int32_t  run_ext;
    uint32_t bars;
    int32_t  faintest_bar;        /* lowest bar peak (darker = higher), raw ADC after det_stats */
    int32_t  palest_space;        /* highest space between bars */
    int32_t  last_space;
} det_t;

/* ---- local-contrast detector (the 6th read of BARCODE_LEVELS = 6) ----
 *
 * A pale or streaky bar (toner running out at the page ends, an inkjet
 * print, a crease) may reach only 20-40 % of the calibrated black, below
 * every fixed switch level, while the bars next to it are fully black. This
 * detector switches at the middle of the LOCAL darkest and lightest values
 * within +-ADT_HALF_MM of each point instead of a level fixed by
 * IR_WHITE_ADC / IR_BLACK_ADC:
 *   - local range (max - min) below swing_frac x span: no edge here (the
 *     middle of a wide bar, the white quiet zone, a light crease, noise);
 *   - otherwise switch at (max + min) / 2, with hysteresis
 *     max(hys, ADT_HYS_FRAC x local range), edge interpolated between bins
 *     like det_sample.
 * It needs ADT_HALF_MM of look-ahead, so its edges come about 4.5 mm late;
 * adt_done_um() tells the window how far the decisions have got.
 */
#define ADT_HALF_BINS   (16u)     /* +-4 mm window (0.25 mm bins) */
#define ADT_RING        (2u * ADT_HALF_BINS + 1u)
#define ADT_HYS_FRAC    (0.2f)    /* hysteresis: this fraction of the local range */
#define ADT_SWING_DEF   (0.15f)   /* BARCODE_ADAPT_SWING default */

typedef struct
{
    bool     b_inv;
    int32_t  span;
    int32_t  swing;               /* smallest local range that can make an edge */
    int32_t  hys;                 /* smallest hysteresis */
    /* binning + 1-2-1 smoothing (as det_t) */
    uint32_t bin_idx;
    uint32_t sum;
    uint32_t cnt;
    bool     b_started;
    int32_t  raw0;
    int32_t  raw1;
    uint32_t raw_n;
    /* smoothed bins, ring */
    int32_t  ring[ADT_RING];
    uint32_t n_s;                 /* smoothed bins so far (index of the next one) */
    /* decision for the bin in the middle of the ring */
    bool     b_black;
    int32_t  prev_c;              /* previous centre value */
    uint32_t done_um;             /* decisions made up to here */
} adt_t;

/* swing_frac: BARCODE_ADAPT_SWING (0.15); hys_frac: BARCODE_DET_HYS */
void     adt_init(adt_t *p_a, uint32_t white, uint32_t black, float hys_frac, float swing_frac);

/* As det_sample. Edges are up to ~4.5 mm behind pos_um. */
uint32_t adt_sample(adt_t *p_a, uint32_t pos_um, uint32_t adc, det_edge_t *p_out);

/* Position up to which edges have been decided: give THIS (not the sensor
   position) to bc_poll, or the window would see white that is not decided. */
uint32_t adt_done_um(const adt_t *p_a);

/* white / black: IR_WHITE_ADC / IR_BLACK_ADC. hys_frac: BARCODE_DET_HYS. */
void     det_init(det_t *p_d, uint32_t white, uint32_t black, float hys_frac);

/* Same, with the switch level at level_frac of the way from white to black
   (0.5 = the mid level, as det_init). Used by bc_multi.c. */
void     det_init_level(det_t *p_d, uint32_t white, uint32_t black, float hys_frac, float level_frac);

/* One ADC sample at odometer position pos_um (never decreasing).
   Writes up to DET_MAX_OUT edges to p_out, returns how many. */
uint32_t det_sample(det_t *p_d, uint32_t pos_um, uint32_t adc, det_edge_t *p_out);

/* Distance (mm) the barcode SENSOR moved along the car's heading for wheel
   steps d_r / d_l (4x counts, + = forwards). The sensor sits y_mm to the
   LEFT of the car's centre (negative = right): while the car turns left the
   sensor, on the inside, moves less than the wheels' mean, and more while it
   turns right. Without this, steering over a barcode (a braked inner wheel,
   or line corrections) stretches or squeezes single bars and spaces by up to
   2x, which is what broke the 26 Oct track windows. */
float    det_sensor_ds(int32_t d_r, int32_t d_l, float mm_per_count, float track_mm, float y_mm);

/* Bars seen since det_init, and the faintest bar / palest space between
   bars in raw ADC values (-1 if none). */
void     det_stats(const det_t *p_d, uint32_t *p_bars, int32_t *p_faintest_bar, int32_t *p_palest_space);

#endif /* BC_DETECT_H */
