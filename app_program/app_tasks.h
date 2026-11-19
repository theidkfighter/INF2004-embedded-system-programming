/**
 * @file    app_tasks.h
 * @brief   Entry points of the seven micro T-Kernel tasks (Barr 6.2.d, 6.4.a).
 *
 * Defined in tsk_*.c, started by os_glue.c (os_start_tasks()). Each task
 * runs forever; the parameters are the micro T-Kernel task arguments.
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <tk/tkernel.h>

void tsk_safety_task(INT stacd, void *exinf);       /* TSK_SAFETY_MON */
void tsk_motion_task(INT stacd, void *exinf);       /* TSK_MOTION_CTRL */
void tsk_line_task(INT stacd, void *exinf);         /* TSK_LINE_BARCODE */
void tsk_obstacle_task(INT stacd, void *exinf);     /* TSK_OBSTACLE_SCAN */
void tsk_imu_task(INT stacd, void *exinf);          /* TSK_TERRAIN_IMU */
void tsk_nav_task(INT stacd, void *exinf);          /* TSK_NAV_PLANNER */
void tsk_telemetry_task(INT stacd, void *exinf);    /* TSK_TELEMETRY */

#endif /* APP_TASKS_H */

/*** end of file ***/
