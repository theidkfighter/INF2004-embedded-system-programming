/**
 * @file    hal_imu.c
 * @brief   GY-511 accelerometer on I2C1 (Pico SDK side, see hal.h).
 *
 * Two chips are sold as "GY-511":
 *   - LSM303DLHC: accelerometer at 0x19, magnetometer at 0x1E (separate
 *     addresses; the DR's part). Set-up follows DR 6.3: +-4 g full scale with
 *     high resolution, 100 Hz, the internal high-pass filter on the INT1 path
 *     so gravity does not count, INT1 = X/Y/Z high event (OR) above 2 g
 *     (63 LSB at 32 mg/LSB), latched.
 *   - LSM303D: accelerometer AND magnetometer on ONE address, 0x1E (SA0 low)
 *     or 0x1D (SA0 high), WHO_AM_I (0x0F) = 0x49. An I2C scan then shows only
 *     0x1E, and the DLHC magnetometer ID registers read the Y magnetic data
 *     (28 Oct test_imu: "0xF6 0xF6 0xF6"). Set-up: 100 Hz, BDU, +-4 g
 *     (0.122 mg/LSB, 16 bit), ST LSM303D datasheet CTRL1 / CTRL2. Its INT1
 *     threshold step and pin polarity are not stated clearly in the
 *     datasheet, so INT1 is NOT used with this chip: collisions come from the
 *     software backup in tsk_imu.c (|a - low-pass| >= 2 g), and
 *     hal_imu_int1_level() / the GP6 interrupt are ignored.
 * hal_imu_init() tries the DLHC first, then the LSM303D at 0x1E and 0x1D.
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

/* LSM303DLHC accelerometer */
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

/* LSM303D (same OUT_X_L_A = 0x28, auto-increment = MSB of the register) */
#define D_WHO_AM_I        (0x0Fu)
#define D_WHO_AM_I_VALUE  (0x49u)
#define D_CTRL0           (0x1Fu)
#define D_CTRL1           (0x20u)
#define D_CTRL2           (0x21u)
#define D_CTRL3           (0x22u)
#define D_CTRL7           (0x26u)
#define D_ADDR_SA0_LOW    (0x1Eu)
#define D_ADDR_SA0_HIGH   (0x1Du)
#define D_G_PER_LSB       (0.000122f)  /* +-4 g */

static uint8_t       g_addr = IMU_ACCEL_ADDR;
static hal_imu_chip_t g_chip = HAL_IMU_NONE;

static absolute_time_t deadline(void)
{
    return from_us_since_boot(hal_time_us() + I2C_TIMEOUT_US);
}

static bool reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    return i2c_write_blocking_until(IMU_I2C, g_addr, buf, 2u, false, deadline()) == 2;
}

static bool reg_read(uint8_t reg, uint8_t *p_buf, uint32_t len)
{
    if (i2c_write_blocking_until(IMU_I2C, g_addr, &reg, 1u, true, deadline()) != 1)
    {
        return false;
    }
    return i2c_read_blocking_until(IMU_I2C, g_addr, p_buf, len, false, deadline()) == (int)len;
}

static void i2c_pins(void)
{
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

static bool init_dlhc(void)
{
    uint8_t check = 0u;
    uint8_t src;

    g_addr = IMU_ACCEL_ADDR;
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

static bool init_lsm303d(uint8_t addr)
{
    uint8_t who = 0u;
    uint8_t check = 0u;

    g_addr = addr;
    if (!reg_read(D_WHO_AM_I, &who, 1u) || (who != D_WHO_AM_I_VALUE))
    {
        return false;
    }
    if (!reg_write(D_CTRL0, 0x00u) ||            /* no FIFO, no high-pass */
        !reg_write(D_CTRL1, 0x6Fu) ||            /* AODR 0110 = 100 Hz, BDU, Z/Y/X on */
        !reg_write(D_CTRL2, 0x48u) ||            /* ABW 01 = 194 Hz anti-alias, AFS 001 = +-4 g */
        !reg_write(D_CTRL3, 0x00u) ||            /* nothing routed to INT1 (see the file header) */
        !reg_write(D_CTRL7, 0x02u))              /* magnetometer power-down (not used) */
    {
        return false;
    }
    return reg_read(D_CTRL1, &check, 1u) && (check == 0x6Fu);
}

bool hal_imu_init(void)
{
    i2c_pins();
    if (init_dlhc())
    {
        g_chip = HAL_IMU_LSM303DLHC;
        return true;
    }
    if (init_lsm303d(D_ADDR_SA0_LOW) || init_lsm303d(D_ADDR_SA0_HIGH))
    {
        g_chip = HAL_IMU_LSM303D;
        return true;
    }
    g_chip = HAL_IMU_NONE;
    g_addr = IMU_ACCEL_ADDR;
    return false;
}

hal_imu_chip_t hal_imu_chip(void)
{
    return g_chip;
}

const char *hal_imu_chip_name(void)
{
    return (g_chip == HAL_IMU_LSM303DLHC) ? "LSM303DLHC at 0x19" :
           (g_chip == HAL_IMU_LSM303D) ? ((g_addr == D_ADDR_SA0_LOW) ? "LSM303D at 0x1E" : "LSM303D at 0x1D") :
           "none";
}

bool hal_imu_int1_used(void)
{
    return g_chip == HAL_IMU_LSM303DLHC;
}

bool hal_imu_accel_g(float *p_x, float *p_y, float *p_z)
{
    uint8_t raw[6];

    if (!reg_read(OUT_X_L_A | AUTO_INC, raw, 6u))
    {
        return false;
    }
    if (g_chip == HAL_IMU_LSM303D)
    {
        /* 16 bit, 0.122 mg/LSB at +-4 g */
        *p_x = (float)(int16_t)(((uint16_t)raw[1] << 8) | raw[0]) * D_G_PER_LSB;
        *p_y = (float)(int16_t)(((uint16_t)raw[3] << 8) | raw[2]) * D_G_PER_LSB;
        *p_z = (float)(int16_t)(((uint16_t)raw[5] << 8) | raw[4]) * D_G_PER_LSB;
        return true;
    }
    /* DLHC: 12-bit left-justified, 2 mg/LSB at +-4 g high resolution */
    *p_x = (float)((int16_t)(((uint16_t)raw[1] << 8) | raw[0]) >> 4) * 0.002f;
    *p_y = (float)((int16_t)(((uint16_t)raw[3] << 8) | raw[2]) >> 4) * 0.002f;
    *p_z = (float)((int16_t)(((uint16_t)raw[5] << 8) | raw[4]) >> 4) * 0.002f;
    return true;
}

bool hal_imu_int1_src(uint8_t *p_src)
{
    if (g_chip == HAL_IMU_LSM303D)
    {
        *p_src = 0u;                              /* INT1 not used with this chip */
        return true;
    }
    return reg_read(INT1_SRC_A, p_src, 1u);
}

bool hal_imu_int1_level(void)
{
    return hal_imu_int1_used() && gpio_get(PIN_IMU_INT1);
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
