/**
 * @file    base_hump.c (Baseline_Build harness, host only)
 * @brief   Baseline_Build hump profiler set-up for the differential
 *          Property 15 (Requirement 7.7).
 *
 * The profiler itself (hump_reset(), hump_step()) is the Baseline_Build
 * imu_logic.c, compiled unchanged (base_hump_step() in libbase.so). The
 * profiler parameters are set in the task body of tsk_imu.c, which needs the
 * kernel, so fetch_baseline.sh copies those assignments ("hp.<field> = ...;")
 * out of the Baseline_Build tsk_imu.c into build/base/src/base_hump_params.inc
 * and this file includes them.
 *
 * Exported (prefixed base_ in libbase.so):
 *   bh_hump_params(p)   fills *p exactly as the Baseline_Build TSK_TERRAIN_IMU
 */
#include "imu_logic.h"
#include "robot_config.h"

void bh_hump_params(hump_params_t *p_out)
{
    hump_params_t hp;

#include "base_hump_params.inc"
    *p_out = hp;
}
