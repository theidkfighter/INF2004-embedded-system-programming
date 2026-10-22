/**
 * @file    hal_imu.c
 * @brief   GY-511 (LSM303DLHC) accelerometer on I2C1 (Pico SDK side, see hal.h).
 *
 * Set-up follows DR 6.3: +-4 g full scale with high resolution, 100 Hz,
 * the internal high-pass filter on the INT1 path so gravity does not count,
 * INT1 = X/Y/Z high event (OR) above 2 g (63 LSB at 32 mg/LSB), latched.
 * Every transfer has a 2 ms timeout (DR: i2c_read_blocking_until).
 */
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/structs/timer.h"

#include "hal.h"
#include "pins.h"

#define IMU_I2C           (i2c1)
#define I2C_TIMEOUT_US    (2000u)

#define CTRL_REG1_A       (0x20u)
#define CTRL_REG2_A       (0x21u)
#define CTRL_REG3_A       (0x22u)
#define CTRL_REG4_A       (0x23u)
#define CTRL_REG5_A       (0x24u)
#define OUT_X_L_A         (0x28u)
#define INT1_CFG_A        (0x30u)
#define INT1_SRC_A        (0x31u)
#define INT1_THS_A        (0x32u)
#define INT1_DURATION_A   (0x33u)
#define AUTO_INC          (0x80u)

static absolute_time_t deadline(void)
{
    return from_us_since_boot(hal_time_us() + I2C_TIMEOUT_US);
}

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    return i2c_write_blocking_until(IMU_I2C, IMU_ACCEL_ADDR, buf, 2u, false, deadline()) == 2;
}

static bool reg_read(uint8_t reg, uint8_t *p_buf, uint32_t len)
{
    if (i2c_write_blocking_until(IMU_I2C, IMU_ACCEL_ADDR, &reg, 1u, true, deadline()) != 1)
    {
        return false;
    }
    return i2c_read_blocking_until(IMU_I2C, IMU_ACCEL_ADDR, p_buf, len, false, deadline()) == (int)len;
}

static void i2c_pins(void)
{
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

bool hal_imu_init(void)
{
    uint8_t check = 0u;
    uint8_t src;

    i2c_pins();
    if (!reg_write(CTRL_REG1_A, 0x57u))          /* 100 Hz, X/Y/Z on */
    {
        return false;
    }
    if (!reg_read(CTRL_REG1_A, &check, 1u) || (check != 0x57u))
    {
        return false;                             /* not an LSM303DLHC accelerometer */
    }
    return reg_write(CTRL_REG4_A, 0x98u) &&      /* BDU, +-4 g, high resolution */
           reg_write(CTRL_REG2_A, 0x01u) &&      /* high-pass filter on the INT1 path */
           reg_write(INT1_THS_A, 63u) &&         /* 2 g at 32 mg/LSB */
           reg_write(INT1_DURATION_A, 0u) &&
           reg_write(INT1_CFG_A, 0x2Au) &&       /* XH | YH | ZH, OR */
           reg_write(CTRL_REG5_A, 0x08u) &&      /* LIR_INT1: latch */
           reg_write(CTRL_REG3_A, 0x40u) &&      /* I1_AOI1: AOI1 on INT1 */
           hal_imu_int1_src(&src);               /* clear anything latched at power-up */
}

bool hal_imu_accel_g(float *p_x, float *p_y, float *p_z)
{
    uint8_t raw[6];

    if (!reg_read(OUT_X_L_A | AUTO_INC, raw, 6u))
    {
        return false;
    }
    /* 12-bit left-justified, 2 mg/LSB at +-4 g high resolution */
    *p_x = (float)((int16_t)(((uint16_t)raw[1] << 8) | raw[0]) >> 4) * 0.002f;
    *p_y = (float)((int16_t)(((uint16_t)raw[3] << 8) | raw[2]) >> 4) * 0.002f;
    *p_z = (float)((int16_t)(((uint16_t)raw[5] << 8) | raw[4]) >> 4) * 0.002f;
    return true;
}

bool hal_imu_int1_src(uint8_t *p_src)
{
    return reg_read(INT1_SRC_A, p_src, 1u);
}

bool hal_imu_int1_level(void)
{
    return gpio_get(PIN_IMU_INT1);
}

void hal_i2c_recover(void)
{
    /* DR 6.3 FAULT: I2C_BUS_RECOVERY - clock SCL nine times to free a slave
       that is holding SDA low, then re-initialise the peripheral */
    i2c_deinit(IMU_I2C);
    gpio_init(PIN_I2C_SCL);
    gpio_init(PIN_I2C_SDA);
    gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    gpio_pull_up(PIN_I2C_SDA);
    for (uint32_t i = 0u; i < 9u; i++)
    {
        uint64_t t;

        gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
        gpio_put(PIN_I2C_SCL, false);
        t = hal_time_us() + 5u;
        while (hal_time_us() < t)
        {
            /* 5 us low */
        }
        gpio_set_dir(PIN_I2C_SCL, GPIO_IN);       /* released: pulled high */
        gpio_pull_up(PIN_I2C_SCL);
        t = hal_time_us() + 5u;
        while (hal_time_us() < t)
        {
            /* 5 us high */
        }
    }
    i2c_pins();
}
