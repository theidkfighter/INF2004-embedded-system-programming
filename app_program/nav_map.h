/**
 * @file    nav_map.h
 * @brief   Barcode letter -> movement mapping, set at run time over MQTT.
 *
 * The demo brief: the instructor names four letters for FORWARD, U-TURN,
 * LEFT and RIGHT just before the run and sends them from the PC over MQTT,
 * so nothing here is hard-coded. The map arrives as a binary Map_Frame
 * (retained is02/map) or the MAP / MAP:CLEAR commands on is02/cmd (wire.c,
 * design D13): one letter for each of FORWARD, LEFT, RIGHT and U-TURN.
 *
 * A letter not in the map is reported and the car carries straight on.
 *
 * Pure logic, no hardware or kernel calls: unit-testable on the PC. The map is
 * set and read from task context only, never from an interrupt handler or
 * inside a crit_dsp_* section: the setters and the whole-map readers open one.
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

/* The packed map g_map_w[MAP_WORDS] is private to nav_map.c (TEST_VISIBLE:
   the host tests reach it through the shared library); firmware code uses
   the functions below. */

void         nav_map_clear(void);
nav_action_t nav_map_get(char letter);              /* 'A'..'Z' (lower case accepted) */
float        nav_action_deg(nav_action_t act);      /* + = clockwise, 0 = straight */

#ifdef HOST_TEST
/* Host-test helper only (test_nav_map.py): the firmware receives the map in
   binary (wire.c). Parses the text after "MAP:". On success replaces the map
   and returns the number of letters mapped. On error leaves the old map
   untouched and returns -1; p_err (may be NULL) gets a short reason. */
int32_t      nav_map_parse(const char *p_text, char *p_err, uint32_t err_size);
#endif

/* The whole map: p_act[i] = nav_action_t of letter 'A' + i. Returns the count. */
#define NAV_MAP_LETTERS  (26u)   /* A..Z */
uint32_t     nav_map_actions(uint8_t p_act[NAV_MAP_LETTERS]);

/* Binary MAP (Command_Frame / Map_Frame): slot k = FORWARD, LEFT, RIGHT, UTURN (D13);
   p_present[k] != 0 maps letter p_letter[k] (0 = 'A') to that movement.
   Replaces the map; letters are checked by the decoder (A..Z, no repeats).
   Returns the number of letters mapped. */
#define NAV_MAP_SLOTS  (4u)   /* FORWARD, LEFT, RIGHT, UTURN (D13) */
uint32_t     nav_map_set_slots(const uint8_t p_present[NAV_MAP_SLOTS], const uint8_t p_letter[NAV_MAP_SLOTS]);

/* Writes "F=FWD,K=LEFT,..." into p_buf. Returns the number of mapped letters. */
#ifdef HOST_TEST   /* host tests only */
uint32_t     nav_map_describe(char *p_buf, uint32_t size);
#endif

#endif /* NAV_MAP_H */
