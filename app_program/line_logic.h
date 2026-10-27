/**
 * @file    line_logic.h
 * @brief   Line following with two straddling IR sensors, and the line search
 *          (DR 6.1 LINE_CENTERED / LINE_DEVIATION_CORRECTION / LINE_LOST_SEARCH).
 *
 * The left and right sensors sit either side of the 18 mm line (about 24 mm
 * apart), so both read WHITE when the car is centred:
 *   both white   -> straight on (or keep a small "hold" correction for
 *                   LINE_HOLD_MM after a sensor clears, so a line that slipped
 *                   outside the sensor is brought back under it)
 *   left black   -> the line moved left  -> turn left
 *   right black  -> the line moved right -> turn right
 *   both black   -> junction / crossing  -> straight on, reported once
 * Steering grows the longer a sensor stays black (tighter curve).
 * Curve memory (LINE_BIAS_KI): an integral of the error that is NOT reset
 * when the line slips out past a sensor or touches the other sensor, so in
 * a tight curve the car keeps turning at the rate the curve needs instead of
 * restarting from LINE_STEER each time (the 21 Oct loop failure). It fades
 * over LINE_BIAS_DECAY_MM of white and is cleared at a junction.
 * The line counts as lost when, after a curve, no sensor has seen black for
 * LINE_LOST_MM. (On a straight, both-white is normal, so it is not lost.)
 *
 * Pure logic, no hardware access: unit-tested on the PC (tests/).
 */
#ifndef LINE_LOGIC_H
#define LINE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINE_CENTERED = 0,
    LINE_CORRECT_LEFT,
    LINE_CORRECT_RIGHT,
    LINE_HOLD_LEFT,
    LINE_HOLD_RIGHT,
    LINE_JUNCTION,
    LINE_LOST
} line_state_t;

typedef struct
{
    float steer_base;       /* LINE_STEER      */
    float steer_ramp;       /* LINE_STEER_RAMP, per second */
    float hold_steer;       /* LINE_HOLD_STEER */
    float hold_mm;          /* LINE_HOLD_MM    */
    float curve_mm;         /* LINE_CURVE_MM   */
    float lost_mm;          /* LINE_LOST_MM    */
    float bias_ki;          /* LINE_BIAS_KI: curve memory growth per second black (0 = off) */
    float bias_decay_mm;    /* LINE_BIAS_DECAY_MM: curve memory fades over this distance of white */
} line_params_t;

typedef struct
{
    line_state_t state;
    int8_t       side;           /* last side that saw black: -1 left, +1 right */
    float        black_s;        /* time the current side has been black */
    float        episode_mm;     /* distance the current side has been black */
    float        hold_left_mm;   /* remaining hold distance */
    float        since_black_mm; /* distance since any sensor last saw black */
    bool         b_last_curve;   /* the last black episode was a curve */
    bool         b_both_prev;
    float        bias;           /* curve memory: integral of the error, kept across
                                    episodes and while both sensors are white */
} line_ctx_t;

typedef struct
{
    float steer;            /* -1..+1, + = turn right */
    bool  b_junction;       /* both sensors just went black */
    bool  b_lost;           /* just became lost */
} line_out_t;

void       line_reset(line_ctx_t *p_ctx);
line_out_t line_step(line_ctx_t *p_ctx, const line_params_t *p_par,
                     bool b_left_black, bool b_right_black, float ds_mm, float dt_s);

/* Steering -> wheel speeds. steer > 0 = turn right: left faster, right slower. */
void       line_wheels(float base_mm_s, float steer, float *p_left, float *p_right);

/* ---- Line search: alternating pivots of growing amplitude ----
 * Targets: +A, -A, +2A, -2A, ... (first sign = side the line was last seen).
 * The caller pivots in the returned direction and feeds the heading change
 * measured by the encoders; the search ends when a sensor sees black (the
 * caller stops calling) or when the amplitude passes max_deg (returns 0). */
typedef struct
{
    float  heading_deg;     /* relative to the start, + = clockwise */
    float  amp_deg;
    float  max_deg;
    int8_t dir;             /* current pivot direction */
    bool   b_second;        /* second half of the current amplitude */
} search_ctx_t;

void   search_start(search_ctx_t *p_s, int8_t first_dir, float first_deg, float max_deg);
int8_t search_step(search_ctx_t *p_s, float dheading_deg);   /* +1 right, -1 left, 0 = give up */

#endif /* LINE_LOGIC_H */
