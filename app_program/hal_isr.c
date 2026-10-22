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

#include "hal.h"
#include "hal_internal.h"
#include "pins.h"
#include "robot_config.h"

#define BC_RING          (64u)          /* power of 2 */
#define SAMPLE_US        (1000u)
#define VBAT_EVERY       (250u)

/* ---- encoders ---- */
static volatile int32_t  g_enc[2];      /* raw 4x counts, wheel 0 = right */
static volatile uint32_t g_odo;
static volatile uint32_t g_enc_err;
static uint8_t           g_enc_prev[2];
static volatile bool     g_enc_rev[2] = { (ENC_R_REVERSED != 0), (ENC_L_REVERSED != 0) };

/* quadrature table: index = (prev << 2) | now, value = step (0 = none/invalid) */
static const int8_t QUAD[16] =
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

/* ---- ultrasonic ---- */
static volatile uint64_t g_echo_rise_us;
static volatile uint32_t g_echo_us;
static volatile bool     g_echo_done;

/* ---- IMU INT1 ---- */
static void (*volatile gp_int1_cb)(void) = NULL;

/* ---- barcode sampling ---- */
static volatile uint32_t g_adc_last;
static volatile bool     g_bc_enabled;
static volatile bool     g_bc_black_high = true;
static volatile uint32_t g_bc_thr_black = 3000u;   /* crossing this way -> black */
static volatile uint32_t g_bc_thr_white = 1000u;   /* crossing this way -> white */
static bool              g_bc_black;
static hal_bc_edge_t     g_bc_ring[BC_RING];
static volatile uint32_t g_bc_head;                /* written by the ISR */
static volatile uint32_t g_bc_tail;                /* written by the reader */
static volatile uint32_t g_bc_overruns;
static volatile uint32_t g_vbat_mv;
static uint32_t          g_alarm_target;
static uint32_t          g_sample_n;

/* ------------------------------------------------------------------------ */

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
        uint64_t now = hal_time_us();

        if (gpio_get(PIN_US_ECHO))
        {
            g_echo_rise_us = now;
        }
        else if ((g_echo_rise_us != 0u) && !g_echo_done)
        {
            g_echo_us   = (uint32_t)(now - g_echo_rise_us);
            g_echo_done = true;
        }
        else
        {
            /* stray falling edge */
        }
    }
    if ((pin_events(ints, PIN_IMU_INT1) & GPIO_IRQ_EDGE_RISE) != 0u)
    {
        if (gp_int1_cb != NULL)
        {
            gp_int1_cb();
        }
    }
}

int32_t hal_enc_count(uint32_t wheel)
{
    int32_t c;

    wheel = (wheel == HAL_WHEEL_RIGHT) ? HAL_WHEEL_RIGHT : HAL_WHEEL_LEFT;
    c = g_enc[wheel];
    return g_enc_rev[wheel] ? -c : c;
}

void hal_enc_set_reversed(bool b_right, bool b_left)
{
    g_enc_rev[HAL_WHEEL_RIGHT] = b_right;
    g_enc_rev[HAL_WHEEL_LEFT]  = b_left;
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
    g_echo_done    = false;
    g_echo_rise_us = 0u;
}

float hal_us_read_cm(void)
{
    if (!g_echo_done)
    {
        return -1.0f;                         /* no echo (nothing within ~4 m) */
    }
    if (g_echo_us > 25000u)
    {
        return -1.0f;
    }
    return (float)g_echo_us / 58.0f;
}

/* ---- barcode sampling (1 kHz) ---- */

void hal_barcode_config(uint32_t thr_black, uint32_t thr_white, bool b_black_high, bool b_enable)
{
    g_bc_enabled    = false;
    g_bc_thr_black  = thr_black;
    g_bc_thr_white  = thr_white;
    g_bc_black_high = b_black_high;
    g_bc_black      = false;
    g_bc_tail       = g_bc_head;
    g_bc_enabled    = b_enable;
}

void hal_timer_isr(uint32_t intno)
{
    uint32_t v;

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
    g_adc_last = v;

    if (g_bc_enabled)
    {
        bool b_black = g_bc_black;

        if (g_bc_black_high)
        {
            b_black = g_bc_black ? (v > g_bc_thr_white) : (v >= g_bc_thr_black);
        }
        else
        {
            b_black = g_bc_black ? (v < g_bc_thr_white) : (v <= g_bc_thr_black);
        }
        if (b_black != g_bc_black)
        {
            uint32_t head = g_bc_head;

            g_bc_black = b_black;
            if ((head - g_bc_tail) < BC_RING)
            {
                g_bc_ring[head & (BC_RING - 1u)].odo     = g_odo;
                g_bc_ring[head & (BC_RING - 1u)].t_ms    = (uint32_t)(hal_time_us() / 1000u);
                g_bc_ring[head & (BC_RING - 1u)].b_black = b_black;
                g_bc_head = head + 1u;
            }
            else
            {
                g_bc_overruns++;
            }
        }
    }

    g_sample_n++;
#if VBAT_ENABLED
    if ((g_sample_n % VBAT_EVERY) == 0u)
    {
        adc_select_input(ADC_CH_VBAT);
        /* VADC2 = VBAT / 2; 3.3 V full scale on 12 bits */
        g_vbat_mv = ((uint32_t)adc_read() * 3300u * 2u) / 4095u;
        adc_select_input(ADC_CH_IR);
    }
#endif
}

bool hal_barcode_pop(hal_bc_edge_t *p_edge)
{
    uint32_t tail = g_bc_tail;

    if (tail == g_bc_head)
    {
        return false;
    }
    *p_edge   = g_bc_ring[tail & (BC_RING - 1u)];
    g_bc_tail = tail + 1u;
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

uint32_t hal_vbat_mv(void)
{
    return g_vbat_mv;
}
