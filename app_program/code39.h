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
char code39_char(const uint32_t *p_w9);

/* Finds "*...*" in the segment list. p_is_bar[i] says whether element i is a
   bar. On success writes the text between the stars to p_out, sets
   *p_reversed (true = read right-to-left) and returns true. */
bool code39_decode(const uint32_t *p_w, const bool *p_is_bar, uint32_t n,
                   char *p_out, size_t out_len, bool *p_reversed);

#endif /* CODE39_H */
