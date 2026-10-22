/**
 * @file    hal_board.c
 * @brief   IS-02 drivers: pins, motors, servo, buzzer, NeoPixels, buttons,
 *          IR line sensors, HC-SR04 trigger (Pico SDK side, see hal.h).
 *
 * micro T-Kernel releases only the peripherals it uses from reset, so this
 * driver releases PWM itself before use. ADC and I2C1 are reset by the SDK's
 * adc_init() / i2c_init().
 */
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/resets.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/watchdog.h"

#include "hal.h"
#include "hal_internal.h"
#include "pins.h"
#include "robot_config.h"

#define SYS_HZ            (125000000u)
#define MOTOR_WRAP        ((SYS_HZ / MOTOR_PWM_FREQ_HZ) - 1u)     /* 6249 */
#define SERVO_WRAP        (19999u)                                /* 1 MHz tick, 20 ms */

/* ws2812 PIO program (pico-examples ws2812.pio, 800 kHz, T1=2 T2=5 T3=3) */
static const uint16_t WS2812_INSTR[] =
{
    0x6221u,   /* 0: out x, 1        side 0 [2] */
    0x1123u,   /* 1: jmp !x, 3       side 1 [1] */
    0x1400u,   /* 2: jmp 0           side 1 [4] */
    0xa442u,   /* 3: nop             side 0 [4] */
};
static const struct pio_program WS2812_PROGRAM =
{
    .instructions = WS2812_INSTR,
    .length       = 4,
    .origin       = -1,
#if PICO_PIO_VERSION > 0
    .pio_version  = 0,
#endif
};

static volatile bool g_motor_killed = false;
static uint32_t      g_servo_min_us = SERVO_MIN_US;
static uint32_t      g_servo_max_us = SERVO_MAX_US;
static PIO           g_np_pio = NULL;
static int32_t       g_np_sm = -1;

/* ------------------------------------------------------------------------ */

uint64_t hal_time_us(void)
{
    uint32_t hi0;
    uint32_t lo;
    uint32_t hi1;

    do
    {
        hi0 = timer_hw->timerawh;
        lo  = timer_hw->timerawl;
        hi1 = timer_hw->timerawh;
    } while (hi0 != hi1);
    return ((uint64_t)hi0 << 32) | lo;
}

uint32_t hal_time_ms(void)
{
    return (uint32_t)(hal_time_us() / 1000u);
}

static void pwm_pin(uint32_t pin, uint32_t wrap, float clkdiv)
{
    uint32_t slice = pwm_gpio_to_slice_num(pin);

    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_wrap(slice, (uint16_t)wrap);
    pwm_set_gpio_level(pin, 0u);
    pwm_set_enabled(slice, true);
}

static void input_pin(uint32_t pin, bool b_pull_up)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (b_pull_up)
    {
        gpio_pull_up(pin);
    }
    else
    {
        gpio_disable_pulls(pin);
    }
}

static void neopixel_init(void)
{
    PIO      pio = pio1;
    int32_t  sm = pio_claim_unused_sm(pio, false);
    int32_t  offset;
    pio_sm_config c;

    if (sm < 0)
    {
        return;
    }
    if (!pio_can_add_program(pio, &WS2812_PROGRAM))
    {
        pio_sm_unclaim(pio, (uint)sm);
        return;
    }
    offset = (int32_t)pio_add_program(pio, &WS2812_PROGRAM);
    pio_gpio_init(pio, PIN_NEOPIXEL);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm, PIN_NEOPIXEL, 1u, true);
    c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, (uint)offset, (uint)offset + 3u);
    sm_config_set_sideset(&c, 1u, false, false);
    sm_config_set_sideset_pins(&c, PIN_NEOPIXEL);
    sm_config_set_out_shift(&c, false, true, 24u);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)SYS_HZ / (800000.0f * 10.0f));
    pio_sm_init(pio, (uint)sm, (uint)offset, &c);
    pio_sm_set_enabled(pio, (uint)sm, true);
    g_np_pio = pio;
    g_np_sm  = sm;
}

void hal_init(void)
{
    /* PWM is left in reset by the kernel (USE_PTMR = 0). PIO0/PIO1 are
       already released for the radio: do not reset them again here. */
    reset_unreset_block_num_wait_blocking(RESET_PWM);

    /* TIMER counts in microseconds from the watchdog tick: clk_ref is the
       12 MHz crystal here, so 12 cycles per tick */
    watchdog_hw->tick = WATCHDOG_TICK_ENABLE_BITS | 12u;

    /* motors: brake first, then PWM */
    pwm_pin(PIN_M1A, MOTOR_WRAP, 1.0f);
    pwm_pin(PIN_M1B, MOTOR_WRAP, 1.0f);
    pwm_pin(PIN_M2A, MOTOR_WRAP, 1.0f);
    pwm_pin(PIN_M2B, MOTOR_WRAP, 1.0f);

    /* encoders: SIO inputs with pull-ups (GP0/GP1 were UART0 in the template) */
    input_pin(PIN_ENC_R_A, true);
    input_pin(PIN_ENC_R_B, true);
    input_pin(PIN_ENC_L_A, true);
    input_pin(PIN_ENC_L_B, true);

    /* IR line sensors: the modules have their own pull-ups; ours only keeps a
       loose wire at a defined level */
    input_pin(PIN_IR_LEFT_DO, true);
    input_pin(PIN_IR_RIGHT_DO, true);

    /* barcode sensor (and VBAT) on the ADC */
    adc_init();
    adc_gpio_init(PIN_IR_AOUT);
#if VBAT_ENABLED
    adc_gpio_init(PIN_VBAT_SENSE);
#endif

    /* HC-SR04 */
    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_put(PIN_US_TRIG, false);
    input_pin(PIN_US_ECHO, false);

    /* IMU INT1 (push-pull from the LSM303); pull-down so an unplugged IMU
       cannot float the pin high and fake a collision */
    gpio_init(PIN_IMU_INT1);
    gpio_set_dir(PIN_IMU_INT1, GPIO_IN);
    gpio_pull_down(PIN_IMU_INT1);

    /* buttons (active LOW) */
    input_pin(PIN_BTN1, true);
    input_pin(PIN_BTN2, true);

    /* servo 50 Hz: 125 MHz / 125 = 1 MHz tick */
    pwm_pin(PIN_SERVO, SERVO_WRAP, 125.0f);
    hal_servo_deg(90.0f);

    /* buzzer: 1 MHz tick, frequency set by the wrap */
    pwm_pin(PIN_BUZZER, 1000u, 125.0f);

    neopixel_init();
    hal_neopixel(0u, 0u, 8u);
}

/* ------------------------------------------------------------------------ */

static void motor_raw(uint32_t pin_a, uint32_t pin_b, float duty)
{
    uint32_t level;

    if (g_motor_killed)
    {
        duty = 0.0f;
    }
    if (duty > 1.0f)
    {
        duty = 1.0f;
    }
    if (duty < -1.0f)
    {
        duty = -1.0f;
    }
    level = (uint32_t)(((duty < 0.0f) ? -duty : duty) * (float)MOTOR_WRAP);
    if (duty > 0.0f)
    {
        pwm_set_gpio_level(pin_b, 0u);
        pwm_set_gpio_level(pin_a, (uint16_t)level);     /* High/Low = forward */
    }
    else if (duty < 0.0f)
    {
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, (uint16_t)level);     /* Low/High = backward */
    }
    else
    {
        pwm_set_gpio_level(pin_a, 0u);                  /* Low/Low = brake */
        pwm_set_gpio_level(pin_b, 0u);
    }
}

void hal_motor_set(uint32_t motor, float duty)
{
    if (motor == HAL_MOTOR_RIGHT)
    {
        motor_raw(PIN_M1A, PIN_M1B, duty);
    }
    else if (motor == HAL_MOTOR_LEFT)
    {
        motor_raw(PIN_M2A, PIN_M2B, duty);
    }
    else
    {
        /* no such motor */
    }
}

void hal_motor_kill(void)
{
    g_motor_killed = true;
    pwm_set_gpio_level(PIN_M1A, 0u);
    pwm_set_gpio_level(PIN_M1B, 0u);
    pwm_set_gpio_level(PIN_M2A, 0u);
    pwm_set_gpio_level(PIN_M2B, 0u);
}

bool hal_motor_killed(void)
{
    return g_motor_killed;
}

void hal_motor_unkill(void)
{
    g_motor_killed = false;
}

/* ------------------------------------------------------------------------ */

bool hal_ir_left_level(void)
{
    return gpio_get(PIN_IR_LEFT_DO);
}

bool hal_ir_right_level(void)
{
    return gpio_get(PIN_IR_RIGHT_DO);
}

void hal_us_trigger(void)
{
    uint64_t t_end;

    hal_us_arm();
    gpio_put(PIN_US_TRIG, true);
    t_end = hal_time_us() + 10u;
    while (hal_time_us() < t_end)
    {
        /* 10 us trigger pulse */
    }
    gpio_put(PIN_US_TRIG, false);
}

void hal_servo_deg(float physical_deg)
{
    float us;

    if (physical_deg < 0.0f)
    {
        physical_deg = 0.0f;
    }
    if (physical_deg > 180.0f)
    {
        physical_deg = 180.0f;
    }
    us = (float)g_servo_min_us + (((float)g_servo_max_us - (float)g_servo_min_us) * physical_deg / 180.0f);
    pwm_set_gpio_level(PIN_SERVO, (uint16_t)us);
}

void hal_servo_range(uint32_t min_us, uint32_t max_us)
{
    if ((min_us >= 300u) && (max_us <= 2800u) && (min_us < max_us))
    {
        g_servo_min_us = min_us;
        g_servo_max_us = max_us;
    }
}

void hal_buzzer(uint32_t freq_hz)
{
    uint32_t slice = pwm_gpio_to_slice_num(PIN_BUZZER);

    if ((freq_hz < 50u) || (freq_hz > 20000u))
    {
        pwm_set_gpio_level(PIN_BUZZER, 0u);
        return;
    }
    {
        uint32_t wrap = (1000000u / freq_hz) - 1u;

        pwm_set_wrap(slice, (uint16_t)wrap);
        pwm_set_gpio_level(PIN_BUZZER, (uint16_t)(wrap / 2u));
    }
}

void hal_neopixel(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    if ((g_np_pio == NULL) || (g_np_sm < 0))
    {
        return;
    }
    /* two pixels; the FIFO is 8 deep (joined), so this never blocks */
    pio_sm_put(g_np_pio, (uint)g_np_sm, grb << 8u);
    pio_sm_put(g_np_pio, (uint)g_np_sm, grb << 8u);
}

bool hal_btn1_down(void)
{
    return !gpio_get(PIN_BTN1);
}

bool hal_btn2_down(void)
{
    return !gpio_get(PIN_BTN2);
}
