/**
 * @file    app_status.c
 * @brief   The shared telemetry snapshot and the state / event names.
 */
#include "app_types.h"

robot_status_t g_status;

static const char * const STATE_NAMES[SYS_STATE_COUNT] =
{
    "INITIALISING", "IDLE", "LINE_FOLLOWING", "MANEUVER", "OBSTACLE_AVOIDANCE",
    "MISSION_COMPLETE", "EMERGENCY_STOP", "ERROR"
};

static const char * const MEV_NAMES[MEV_COUNT] =
{
    "STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING", "TURNING",
    "CLIMBING_HUMP", "DESCENDING_HUMP", "SUDDEN_IMPACT"
};

const char *sys_state_name(sys_state_t state)
{
    return ((uint32_t)state < (uint32_t)SYS_STATE_COUNT) ? STATE_NAMES[state] : "UNKNOWN";
}

const char *motion_event_name(motion_event_t ev)
{
    return ((uint32_t)ev < (uint32_t)MEV_COUNT) ? MEV_NAMES[ev] : "UNKNOWN";
}
