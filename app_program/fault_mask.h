/**
 * @file    fault_mask.h
 * @brief   Fault_Mask (Change Requirement 1, design D1): 16 individual
 *          fault bits in one Storage_Unit of the Status_Snapshot.
 *
 * Bits 0..12 are the Baseline_Build faults at their old positions. Bits
 * 13..15 replace separate variables: g_wifi_up / g_mqtt_up (app_net.c) and
 * the "no echo" result of the last ping. fault_set() / fault_clear() are one
 * dispatch-disabled read-modify-write each (app_status.c); readers use one
 * load. Plain C header: safe to include from lwIP code (no kernel types).
 */
#ifndef FAULT_MASK_H
#define FAULT_MASK_H

#include <stdbool.h>
#include <stdint.h>

#define FAULT_IMU_I2C        (1u << 0)
#define FAULT_IMU_DEAD       (1u << 1)
#define FAULT_ENCODER_LOSS   (1u << 2)
#define FAULT_MOTION_TIMEOUT (1u << 3)
#define FAULT_ULTRASONIC     (1u << 4)    /* defined, never set (as in the Baseline_Build) */
#define FAULT_LINE_LOST      (1u << 5)
#define FAULT_BYPASS_ABORT   (1u << 6)
#define FAULT_WATCHDOG       (1u << 7)
#define FAULT_BATTERY_LOW    (1u << 8)
#define FAULT_BATTERY_CRIT   (1u << 9)
#define FAULT_CONFIG         (1u << 10)
#define FAULT_BARCODE        (1u << 11)
#define FAULT_TELEMETRY      (1u << 12)   /* defined, never set */
#define FAULT_WIFI_LOSS      (1u << 13)   /* 1 at boot; radio-owner task */
#define FAULT_MQTT_LOSS      (1u << 14)   /* 1 at boot; radio-owner task */
#define FAULT_US_NO_ECHO     (1u << 15)   /* last ping had no echo; TSK_OBSTACLE_SCAN */
#define FAULT_BASE_MASK      (0x1FFFu)    /* the 13 Baseline_Build fault bits */
#define FAULT_BOOT_VALUE     (FAULT_WIFI_LOSS | FAULT_MQTT_LOSS)

void     fault_set(uint32_t fault_bits);
void     fault_clear(uint32_t fault_bits);
uint32_t st_faults(void);                 /* the whole 16-bit Fault_Mask, one load */

static inline bool fault_has(uint32_t bits)
{
    return (st_faults() & bits) != 0u;
}

#endif /* FAULT_MASK_H */
