/**
 * @file    nav_map.h
 * @brief   Barcode letter -> movement mapping, set at run time over MQTT.
 *
 * The demo brief: the instructor names four letters for FORWARD, U-TURN,
 * LEFT and RIGHT just before the run and sends them from the PC over MQTT,
 * so nothing here is hard-coded. Accepted forms (is02/cmd or retained is02/map):
 *
 *   MAP:F=FWD,K=LEFT,R=RIGHT,U=UTURN     any order, any subset, ',' ';' or ' '
 *   MAP:F,U,K,R                          positional: FORWARD,UTURN,LEFT,RIGHT
 *   MAP:CLEAR                            forget everything
 *
 * Movement words (case-insensitive): FWD / FORWARD / STRAIGHT / F,
 * LEFT / L, RIGHT / R, UTURN / U-TURN / U / BACK.
 * A letter not in the map is reported and the car carries straight on.
 *
 * Pure logic, no hardware or kernel calls: unit-testable on the PC.
 */
#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ACT_NONE = 0,      /* not mapped */
    ACT_FORWARD,
    ACT_UTURN,
    ACT_LEFT,
    ACT_RIGHT
} nav_action_t;

void         nav_map_clear(void);
nav_action_t nav_map_get(char letter);              /* 'A'..'Z' (lower case accepted) */
float        nav_action_deg(nav_action_t act);      /* + = clockwise, 0 = straight */
const char  *nav_action_name(nav_action_t act);

/* Parses the text after "MAP:" (or the whole retained is02/map payload).
   On success replaces the map and returns the number of letters mapped.
   On error leaves the old map untouched and returns -1; p_err (may be NULL)
   gets a short reason. */
int32_t      nav_map_parse(const char *p_text, char *p_err, uint32_t err_size);

/* Writes "F=FWD,K=LEFT,..." into p_buf. Returns the number of mapped letters. */
uint32_t     nav_map_describe(char *p_buf, uint32_t size);

#endif /* NAV_MAP_H */
