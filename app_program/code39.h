/**
 * @file    code39.h
 * @brief   Code 39 decoder for the IS-02 tests (course barcodes, see
 *          ../barcode/Barcode Sample.pdf: "*A*", "*Z*", wide = 3 x narrow,
 *          narrow inter-character gap, about 141 mm long on the course).
 *
 * Input is the list of measured bar/space widths in the order they passed
 * under the sensor. Each character is 9 elements (5 bars, 4 spaces) with
 * exactly 3 wide; the 3 widest of each 9 are taken as wide, so the speed only
 * needs to be steady within one character. Reads forwards or backwards (FR7).
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Returns the character for a 9-element window (bar first), or 0 if invalid. */
char code39_char(const uint16_t *p_w9);

/* Finds "*...*" in the segment list (widths in um, U16). Elements alternate
   bar / space; b_first_bar says whether element 0 is a bar (Change
   Requirement 1: one bit instead of a flag per element). On success writes
   the text between the stars to p_out, sets *p_reversed (true = read
   right-to-left) and returns true. */
bool code39_decode(const uint16_t *p_w, bool b_first_bar, uint32_t n,
                   char *p_out, size_t out_len, bool *p_reversed);

/* 6-bit Code 39 index (Requirement 5.3), in the Data_Representation_Table
   order: '0'..'9' = 0..9, 'A'..'Z' = 10..35, '-' = 36, '.' = 37, ' ' = 38,
   '$' = 39, '/' = 40, '+' = 41, '%' = 42. C39_NONE (63) means "none".
   Codes 43..62 are never stored. */
#define C39_COUNT   43u     /* data characters, indices 0..42 */
#define C39_NONE    63u     /* "none" */

/* Index of c (0..42), or C39_NONE for any other character, including 0 and
   the start/stop character '*'. */
uint32_t c39_to_index(char c);

/* Character for index idx (0..42). Returns 0 for C39_NONE and also for the
   never-stored codes 43..62 and anything larger, so a corrupt index reads
   back as "none". */
char c39_from_index(uint32_t idx);

#endif /* CODE39_H */
