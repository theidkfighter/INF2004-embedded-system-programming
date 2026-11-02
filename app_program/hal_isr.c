/**
 * @file    hal_isr.c
 * @brief   IS-02 interrupt drivers (Pico SDK side, see hal.h).
 *
 * GPIO (IO_BANK0, IRQ 13): one dispatcher for every edge interrupt, as agreed
 * in DR Section 4 - wheel encoders (4x quadrature), HC-SR04 Echo (pulse
 * timing) and IMU INT1 (collision, FR23).
 *
 * TIMER alarm 3 (IRQ 3), 1 kHz: samples the barcode sensor on ADC0, finds
 * black/white edges with hysteresis and queues each edge with the encoder
 * odometer, so barcode widths are measured in distance. Every 250 ms it also
 * reads VBAT on ADC2 (when enabled), so the ADC has a single user.
 */
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/timer.h"

#include "bitpack.h"
#include "crit.h"
#include "hal.h"
#include "hal_internal.h"
#include "isr_ring.h"
#include "perf.h"
#include "pins.h"
#include "robot_config.h"

#define SAMPLE_US        (1000u)
#define VBAT_EVERY       (250u)

/* ---- encoders ---- */
/* Change Requirement 1, own units (Requirements 10.5, 10.6): g_enc[], g_odo and
   g_enc_err stay full 32-bit modular counters, one field per unit. The GPIO
   ISR adds to them on every encoder edge, the most frequent handler in the
   build. A narrower field would need sign or wrap handling there, and a unit
   shared with another field would need a masked section around each task
   write to it. */
static volatile int32_t  g_enc[2];      /* raw 4x counts, wheel 0 = right */
static volatile uint32_t g_odo;
static volatile uint32_t g_enc_err;
static uint8_t           g_enc_prev[2];
/* direction flags (Requirement 4.7): bit = wheel number, one load by each
   reader, one composed store by hal_enc_set_reversed() only */
#define ENC_REV_BIT(wheel)  (1u << (wheel))
static volatile uint8_t  g_enc_rev = (uint8_t)(((ENC_R_REVERSED != 0) ? ENC_REV_BIT(HAL_WHEEL_RIGHT) : 0u)
                                             | ((ENC_L_REVERSED != 0) ? ENC_REV_BIT(HAL_WHEEL_LEFT) : 0u));

/* quadrature table: index = (prev << 2) | now, value = step (0 = none/invalid) */
static const int8_t QUAD[16] =
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

/* ---- ultrasonic ---- */
/* Change Requirement 1: the echo state is one word written by the GPIO ISR
   and read by TSK_OBSTACLE with one 32-bit load (design D4):
     bits 14..0 echo width U15 (0..25000 us, 25001 = longer than the 25 ms
     timeout), bit 15 done, bit 16 rise seen.
   hal_us_arm() is the only task write, inside a masked section.
   The rise time keeps only the low 32 bits of the 1 MHz timer: an echo is at
   most a few tens of ms, so the modulo-2^32 difference is exact. */
#define ECHO_US_MASK     (0x7FFFu)
#define ECHO_US_MAX      (25000u)
#define ECHO_US_LONG     (ECHO_US_MAX + 1u)
#define ECHO_DONE        (1u << 15)
#define ECHO_RISE        (1u << 16)
#define ECHO_STATE_SH    (15u)          /* done and rise seen, without the width */
static volatile uint32_t g_echo_rise_us;
static volatile uint32_t g_echo;

/* ---- IMU INT1 ---- */
static void (*volatile gp_int1_cb)(void) = NULL;

/* ---- barcode sampling ---- */
static volatile uint16_t g_adc_last;           /* 12-bit ADC: U12 in a halfword */
/* range since hal_ir_adc_range(): min U12 @0, max U12 @12, have F1 @24.
   have = 0 means no sample yet, so no marker above 4095 is needed. */
#define ADC_W            (12u)
#define ADC_MIN_SH       (0u)
#define ADC_MAX_SH       (12u)
#define ADC_HAVE         (1u << 24)
#define ADC_RNG(mn, mx)  (ADC_HAVE | ((mx) << ADC_MAX_SH) | ((mn) << ADC_MIN_SH))
static volatile uint32_t g_adc_rng;
/* sampling flags (Requirements 5.6, 5.7): the timer ISR changes black, tasks
   change the unit inside a masked section. The ISR takes one load per tick. */
#define BC_F_ENABLED     (1u << 0)
#define BC_F_BLACK_HIGH  (1u << 1)
#define BC_F_BLACK       (1u << 2)
#define BC_F_RAW         (1u << 3)
static volatile uint8_t  g_bc_flags = (uint8_t)BC_F_BLACK_HIGH;
/* thresholds: crossing thr_black -> black (U12 @0), crossing thr_white -> white
   (U12 @12). One store, while enabled = 0. */
#define BC_THR_W         (12u)
#define BC_THR_BLACK_SH  (0u)
#define BC_THR_WHITE_SH  (12u)
#define BC_THR(blk, wht) (((uint32_t)(blk) << BC_THR_BLACK_SH) | ((uint32_t)(wht) << BC_THR_WHITE_SH))
static volatile uint32_t g_bc_thr = BC_THR(3000u, 1000u);
/* layout in isr_ring.h; the time of the edge was never read, so it is no
   longer stored */
static uint32_t          g_bc_ring[BC_RING];
static volatile uint8_t  g_bc_head;                /* M7, written by the ISR */
static volatile uint8_t  g_bc_tail;                /* M7, written by the reader */
/* own 32-bit unit (Requirements 10.5, 10.6): a diagnostic counter that the
   timer ISR adds to and tasks only read, so it needs no section and shares
   with nothing */
static volatile uint32_t g_bc_overruns;
static uint32_t          g_raw[RAW_RING];          /* layout in isr_ring.h */
static int32_t           g_raw_full_r;             /* reader side: rebuilt counts */
static int32_t           g_raw_full_l;
static uint32_t          g_raw_ref_r;              /* reader side: low bits of the last sample */
static uint32_t          g_raw_ref_l;
static bool              g_raw_gap;                /* ISR side: a sample was dropped */
static volatile uint16_t g_raw_head;               /* M9, written by the ISR */
static volatile uint16_t g_raw_tail;               /* M9, written by the reader */
static volatile uint16_t g_vbat_mv;               /* U13 mV in a halfword */
static uint32_t          g_alarm_target;
#if VBAT_ENABLED
BP_STATIC_ASSERT(VBAT_EVERY <= 256u, "g_vbat_div: 0 .. VBAT_EVERY - 1 fits in 8 bits");
static uint8_t           g_vbat_div;               /* timer ISR only: samples since the last VBAT read */
#endif

/* ------------------------------------------------------------------------ */

/* Count of one wheel with the direction flag from one load of g_enc_rev. */
static inline int32_t enc_signed(uint32_t wheel, uint32_t rev)
{
    int32_t c = g_enc[wheel];

    return ((rev & ENC_REV_BIT(wheel)) != 0u) ? -c : c;
}

static uint8_t enc_state(uint32_t pin_a, uint32_t pin_b)
{
    return (uint8_t)(((gpio_get(pin_a) ? 1u : 0u) << 1) | (gpio_get(pin_b) ? 1u : 0u));
}

static void enc_update(uint32_t wheel, uint32_t pin_a, uint32_t pin_b)
{
    uint8_t now  = enc_state(pin_a, pin_b);
    uint8_t prev = g_enc_prev[wheel];
    int8_t  step = QUAD[(prev << 2) | now];

    if (step != 0)
    {
        g_enc[wheel] += step;
        g_odo++;
    }
    else if (now != prev)
    {
        g_enc_err++;                          /* both channels changed: a missed edge */
    }
    else
    {
        /* bounce back to the same state */
    }
    g_enc_prev[wheel] = now;
}

void hal_set_int1_callback(void (*p_cb)(void))
{
    gp_int1_cb = p_cb;
}

void hal_irq_enable_sources(void)
{
    const uint32_t both = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    g_enc_prev[HAL_WHEEL_RIGHT] = enc_state(PIN_ENC_R_A, PIN_ENC_R_B);
    g_enc_prev[HAL_WHEEL_LEFT]  = enc_state(PIN_ENC_L_A, PIN_ENC_L_B);

    gpio_set_irq_enabled(PIN_ENC_R_A, both, true);
    gpio_set_irq_enabled(PIN_ENC_R_B, both, true);
    gpio_set_irq_enabled(PIN_ENC_L_A, both, true);
    gpio_set_irq_enabled(PIN_ENC_L_B, both, true);
    gpio_set_irq_enabled(PIN_US_ECHO, both, true);
    gpio_set_irq_enabled(PIN_IMU_INT1, GPIO_IRQ_EDGE_RISE, true);

    /* 1 kHz sampling on TIMER alarm 3 */
    g_alarm_target = timer_hw->timerawl + SAMPLE_US;
    timer_hw->alarm[3] = g_alarm_target;
    hw_set_bits(&timer_hw->inte, 1u << 3);
}

/* Reads the edge events of one pin (4 bits) from the latched status. */
static uint32_t pin_events(const uint32_t *p_ints, uint32_t pin)
{
    return (p_ints[pin >> 3] >> (4u * (pin & 7u))) & 0xFu;
}

void hal_gpio_isr(uint32_t intno)
{
    PERF_BEGIN(t_perf);
    uint32_t ints[4];

    (void)intno;
    for (uint32_t i = 0u; i < 4u; i++)
    {
        ints[i] = io_bank0_hw->proc0_irq_ctrl.ints[i];
        io_bank0_hw->intr[i] = ints[i];       /* acknowledge the edge events */
    }

    if ((pin_events(ints, PIN_ENC_R_A) | pin_events(ints, PIN_ENC_R_B)) != 0u)
    {
        enc_update(HAL_WHEEL_RIGHT, PIN_ENC_R_A, PIN_ENC_R_B);
    }
    if ((pin_events(ints, PIN_ENC_L_A) | pin_events(ints, PIN_ENC_L_B)) != 0u)
    {
        enc_update(HAL_WHEEL_LEFT, PIN_ENC_L_A, PIN_ENC_L_B);
    }
    if (pin_events(ints, PIN_US_ECHO) != 0u)
    {
        uint32_t now  = (uint32_t)hal_time_us();
        uint32_t echo = g_echo;

        if (gpio_get(PIN_US_ECHO))
        {
            if ((echo & ECHO_DONE) == 0u)
            {
                g_echo_rise_us = now;
                g_echo         = ECHO_RISE;
            }
        }
        else if ((echo >> ECHO_STATE_SH) == (ECHO_RISE >> ECHO_STATE_SH))   /* rise seen, not done */
        {
            uint32_t us = now - g_echo_rise_us;

            g_echo = ECHO_DONE | ECHO_RISE | ((us > ECHO_US_MAX) ? ECHO_US_LONG : us);
        }
        else
        {
            /* stray falling edge */
        }
    }
    if ((pin_events(ints, PIN_IMU_INT1) & GPIO_IRQ_EDGE_RISE) != 0u)
    {
        if ((gp_int1_cb != NULL) && hal_imu_int1_used())   /* LSM303D: INT1 not configured */
        {
            gp_int1_cb();
        }
    }
    PERF_END(PERF_ISR_GPIO, t_perf, PERF_NO_LIMIT);
}

int32_t hal_enc_count(uint32_t wheel)
{
    wheel = (wheel == HAL_WHEEL_RIGHT) ? HAL_WHEEL_RIGHT : HAL_WHEEL_LEFT;
    return enc_signed(wheel, g_enc_rev);
}

void hal_enc_set_reversed(bool b_right, bool b_left)
{
    g_enc_rev = (uint8_t)((b_right ? ENC_REV_BIT(HAL_WHEEL_RIGHT) : 0u)
                        | (b_left ? ENC_REV_BIT(HAL_WHEEL_LEFT) : 0u));
}

uint32_t hal_odometer(void)
{
    return g_odo;
}

uint32_t hal_enc_errors(void)
{
    return g_enc_err;
}

/* ---- ultrasonic ---- */

void hal_us_arm(void)
{
    uint32_t saved = crit_irq_enter();

    g_echo_rise_us = 0u;
    g_echo         = 0u;
    crit_irq_exit(saved);
}

float hal_us_read_cm(void)
{
    uint32_t echo = g_echo;

    if ((echo & ECHO_DONE) == 0u)
    {
        return -1.0f;                         /* no echo (nothing within ~4 m) */
    }
    if ((echo & ECHO_US_MASK) > ECHO_US_MAX)
    {
        return -1.0f;
    }
    return (float)(echo & ECHO_US_MASK) / 58.0f;
}

/* ---- barcode sampling (1 kHz) ---- */

void hal_barcode_config(uint32_t thr_black, uint32_t thr_white, bool b_black_high, bool b_enable)
{
    uint32_t thr = BC_THR((thr_black > 4095u) ? 4095u : thr_black,
                          (thr_white > 4095u) ? 4095u : thr_white);
    uint32_t saved = crit_irq_enter();
    uint32_t flags = (uint32_t)g_bc_flags & BC_F_RAW;      /* black = 0, enabled = 0 */

    g_bc_flags = (uint8_t)flags;
    g_bc_thr   = thr;
    g_bc_tail  = g_bc_head;
    g_bc_flags = (uint8_t)(flags | (b_black_high ? BC_F_BLACK_HIGH : 0u) | (b_enable ? BC_F_ENABLED : 0u));
    crit_irq_exit(saved);
}

void hal_timer_isr(uint32_t intno)
{
    PERF_BEGIN(t_perf);
    uint32_t v;
    uint32_t rng;
    uint32_t mn;
    uint32_t mx;
    uint32_t flags;

    (void)intno;
    timer_hw->intr = 1u << 3;                     /* INTR is write-1-to-clear */

    /* next sample, without drift; resynchronise if we fell behind */
    g_alarm_target += SAMPLE_US;
    if ((int32_t)(g_alarm_target - timer_hw->timerawl) <= 0)
    {
        g_alarm_target = timer_hw->timerawl + SAMPLE_US;
    }
    timer_hw->alarm[3] = g_alarm_target;

    adc_select_input(ADC_CH_IR);
    v = adc_read();
    g_adc_last = (uint16_t)v;
    /* one load and at most one store, as only one limit can move; have = 0
       only in the reset word 0, so comparing with 0 costs less than a mask */
    rng = g_adc_rng;
    mn  = BP_GET(rng, ADC_MIN_SH, ADC_W);
    mx  = BP_GET(rng, ADC_MAX_SH, ADC_W);
    if (rng == 0u)
    {
        g_adc_rng = ADC_RNG(v, v);
    }
    else if (v < mn)
    {
        g_adc_rng = ADC_RNG(v, mx);
    }
    else if (v > mx)
    {
        g_adc_rng = ADC_RNG(mn, v);
    }
    else
    {
        /* inside the range */
    }

    flags = g_bc_flags;
    if ((flags & BC_F_RAW) != 0u)
    {
        uint32_t head = g_raw_head;

        if (ring_used(head, g_raw_tail, RAW_IDX_MASK) < RAW_RING)
        {
            uint32_t rev = g_enc_rev;
            uint32_t er  = (uint32_t)enc_signed(HAL_WHEEL_RIGHT, rev);
            uint32_t el  = (uint32_t)enc_signed(HAL_WHEEL_LEFT, rev);

            g_raw[head & RAW_SLOT_MASK] = raw_slot_pack(v, er, el, g_raw_gap);
            g_raw_gap = false;
            g_raw_head = (uint16_t)((head + 1u) & RAW_IDX_MASK);
        }
        else
        {
            g_raw_gap = true;
            g_bc_overruns++;
        }
    }
    else if ((flags & BC_F_ENABLED) != 0u)
    {
        bool     b_was = (flags & BC_F_BLACK) != 0u;
        uint32_t thr   = BP_GET(g_bc_thr, b_was ? BC_THR_WHITE_SH : BC_THR_BLACK_SH, BC_THR_W);
        bool     b_black;

        if ((flags & BC_F_BLACK_HIGH) != 0u)
        {
            b_black = b_was ? (v > thr) : (v >= thr);
        }
        else
        {
            b_black = b_was ? (v < thr) : (v <= thr);
        }
        if (b_black != b_was)
        {
            uint32_t head = g_bc_head;

            g_bc_flags = (uint8_t)(flags ^ BC_F_BLACK);
            if (ring_used(head, g_bc_tail, BC_IDX_MASK) < BC_RING)
            {
                g_bc_ring[head & BC_SLOT_MASK] = bc_edge_pack(g_odo, b_black);
                g_bc_head = (uint8_t)((head + 1u) & BC_IDX_MASK);
            }
            else
            {
                g_bc_overruns++;
            }
        }
    }

#if VBAT_ENABLED
    {
        uint32_t div = (uint32_t)g_vbat_div + 1u;

        if (div >= VBAT_EVERY)
        {
            div = 0u;
            adc_select_input(ADC_CH_VBAT);
            /* VADC2 = VBAT / 2; 3.3 V full scale on 12 bits */
            g_vbat_mv = (uint16_t)(((uint32_t)adc_read() * 3300u * 2u) / 4095u);
            adc_select_input(ADC_CH_IR);
        }
        g_vbat_div = (uint8_t)div;
    }
#endif
    PERF_END(PERF_ISR_TIMER, t_perf, PERF_NO_LIMIT);
}

bool hal_barcode_pop(hal_bc_edge_t *p_edge)
{
    uint32_t tail = g_bc_tail;

    if (tail == g_bc_head)
    {
        return false;
    }
    {
        uint32_t slot = g_bc_ring[tail & BC_SLOT_MASK];
        uint32_t now  = g_odo;

        g_bc_tail       = (uint8_t)((tail + 1u) & BC_IDX_MASK);
        p_edge->odo     = bc_edge_odo(slot, now);
        p_edge->b_black = bc_edge_black(slot);
    }
    return true;
}

void hal_barcode_raw(bool b_enable)
{
    uint32_t saved = crit_irq_enter();
    uint32_t rev   = g_enc_rev;
    uint32_t flags = (uint32_t)g_bc_flags & ~BC_F_RAW;

    g_raw_tail = g_raw_head;
    /* reader base = the counts now; the first sample's difference is small */
    g_raw_full_r = enc_signed(HAL_WHEEL_RIGHT, rev);
    g_raw_full_l = enc_signed(HAL_WHEEL_LEFT, rev);
    g_raw_ref_r  = (uint32_t)g_raw_full_r & RAW_CNT_MASK;
    g_raw_ref_l  = (uint32_t)g_raw_full_l & RAW_CNT_MASK;
    g_raw_gap    = false;
    if (b_enable)
    {
        flags = (flags & ~BC_F_ENABLED) | BC_F_RAW;   /* no threshold edges in raw mode */
    }
    g_bc_flags = (uint8_t)flags;
    crit_irq_exit(saved);
}

bool hal_barcode_pop_sample(int32_t *p_enc_r, int32_t *p_enc_l, uint32_t *p_adc)
{
    uint32_t tail = g_raw_tail;

    if (tail == g_raw_head)
    {
        return false;
    }
    {
        uint32_t slot = g_raw[tail & RAW_SLOT_MASK];

        g_raw_tail    = (uint16_t)((tail + 1u) & RAW_IDX_MASK);
        uint32_t lo_r = (slot >> RAW_ER_SHIFT) & RAW_CNT_MASK;
        uint32_t lo_l = (slot >> RAW_EL_SHIFT) & RAW_CNT_MASK;

        if ((slot & RAW_GAP) == 0u)               /* after a gap: no movement, new reference */
        {
            g_raw_full_r += raw_delta(lo_r, g_raw_ref_r);
            g_raw_full_l += raw_delta(lo_l, g_raw_ref_l);
        }
        g_raw_ref_r = lo_r;
        g_raw_ref_l = lo_l;
        *p_enc_r = g_raw_full_r;
        *p_enc_l = g_raw_full_l;
        *p_adc   = slot & RAW_ADC_MASK;
    }
    return true;
}

uint32_t hal_barcode_overruns(void)
{
    return g_bc_overruns;
}

uint32_t hal_ir_adc(void)
{
    return g_adc_last;
}

void hal_ir_adc_range(uint32_t *p_min, uint32_t *p_max)
{
    uint32_t saved = crit_irq_enter();
    uint32_t rng   = g_adc_rng;
    uint32_t last  = g_adc_last;

    g_adc_rng = 0u;                               /* read and reset as one step: no sample is lost */
    crit_irq_exit(saved);
    if ((rng & ADC_HAVE) == 0u)
    {
        *p_min = last;
        *p_max = last;
    }
    else
    {
        *p_min = BP_GET(rng, ADC_MIN_SH, ADC_W);
        *p_max = BP_GET(rng, ADC_MAX_SH, ADC_W);
    }
}

uint32_t hal_vbat_mv(void)
{
    return g_vbat_mv;
}
