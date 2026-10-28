/**
 * @file    app_main.c
 * @brief   IS-02 Robo-Wheeler on micro T-Kernel 3.0 (template:
 *          sirfonzie/mtk3smp-rp2040). usermain() = DR INITIALISING state.
 *
 * Order: kernel objects -> drivers (motors braked first) -> IMU self-test ->
 * interrupts and the 10 ms tick -> the seven DR tasks. The CYW43 radio
 * service and the USB console are started by the template before usermain().
 * TSK_SAFETY_MON then moves INITIALISING -> IDLE, or -> ERROR if a
 * calibration value is missing (listed on the USB console).
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "os_glue.h"
#include "app_params.h"
#include "hal.h"
#include "usb_console_compat.h"

EXPORT INT usermain(void)
{
    INT i;

    /* give a USB host up to 2 s to enumerate so the banner is not lost;
       the robot runs the same without a laptop */
    for (i = 0; (i < 20) && (tm_usb_state() < 2u); i++)
    {
        (void)tk_dly_tsk(100);
    }
    tm_printf((UB *)"\n=== IS-02 Robo-Wheeler, micro T-Kernel 3.0 ===\n");

    g_status.state = (uint32_t)SYS_INIT;
    params_init();

    if (!os_create_objects())
    {
        tm_printf((UB *)"[init] kernel objects failed: stopped\n");
        (void)tk_slp_tsk(TMO_FEVR);
        return 0;
    }

    hal_init();                              /* motors braked before anything else */
    params_apply();

    if (!hal_imu_init())
    {
        /* degraded mode: the robot still drives; humps and collisions are
           not detected until the IMU answers (TSK_TERRAIN_IMU retries) */
        tm_printf((UB *)"[init] IMU (GY-511 on I2C1 GP2/GP3: LSM303DLHC at 0x19 or LSM303D at 0x1E/0x1D)"
                        " not answering: running WITHOUT hump/collision detection, retrying every 2 s\n");
        fault_set(FAULT_IMU_DEAD);
    }
    else
    {
        tm_printf((UB *)"[init] IMU: %s%s\n", hal_imu_chip_name(),
                  hal_imu_int1_used() ? "" : " (INT1 not used: software collision detection only)");
    }

    if (!os_start_interrupts() || !os_start_tasks())
    {
        hal_motor_kill();
        tm_printf((UB *)"[init] start-up failed: motors off\n");
    }
    else
    {
        tm_printf((UB *)"[init] 7 tasks running. BTN1 = start / end run, BTN2 = stop\n");
    }

    /* the initial task must not return (the kernel would shut down) */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
