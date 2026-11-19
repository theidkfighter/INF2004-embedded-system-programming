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
#include "status.h"
#include "app_params.h"
#include "hal.h"
#include "usb_console_compat.h"

#define USB_WAIT_TRIES      (20)    /* 20 x 100 ms = 2 s */
#define USB_WAIT_MS         (100)
#define USB_STATE_CONFIGURED (2u)     /* tm_usb_state(): enumerated by a host */

EXPORT INT usermain(void)
{
    INT i;

    /* give a USB host up to 2 s to enumerate so the banner is not lost;
       the robot runs the same without a laptop */
    for (i = 0; (i < USB_WAIT_TRIES) && (tm_usb_state() < USB_STATE_CONFIGURED); i++)
    {
        (void)tk_dly_tsk(USB_WAIT_MS);
    }
    (void)tm_printf((UB *)"\n=== IS-02 Robo-Wheeler, micro T-Kernel 3.0 ===\n");

    st_set_state(SYS_INIT);
    fault_set(FAULT_BOOT_VALUE);              /* no Wi-Fi / MQTT yet (design D1) */
    params_init();

    if (!os_create_objects())
    {
        (void)tm_printf((UB *)"[init] kernel objects failed: stopped\n");
        (void)tk_slp_tsk(TMO_FEVR);
        return 0;
    }

    hal_init();                              /* motors braked before anything else */
    params_apply();

    if (!hal_imu_init())
    {
        /* degraded mode: the robot still drives; humps and lifting are
           not detected until the IMU answers (TSK_TERRAIN_IMU retries) */
        (void)tm_printf((UB *)"[init] IMU (GY-511 on I2C1 GP2/GP3: LSM303DLHC at 0x19 or LSM303D at 0x1E/0x1D)"
                        " not answering: running WITHOUT hump/lift detection, retrying every 2 s\n");
        fault_set(FAULT_IMU_DEAD);
    }
    else
    {
        (void)tm_printf((UB *)"[init] IMU: %s\n", hal_imu_chip_name());
    }

    if (!os_start_interrupts() || !os_start_tasks())
    {
        hal_motor_kill();
        (void)tm_printf((UB *)"[init] start-up failed: motors off\n");
    }
    else
    {
        (void)tm_printf((UB *)"[init] 7 tasks running. BTN1 = start / end run, BTN2 = stop\n");
    }

    /* the initial task must not return (the kernel would shut down) */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
