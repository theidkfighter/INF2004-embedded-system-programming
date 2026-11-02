/**
 * @file    crit.h
 * @brief   Critical_Section hooks and the firmware-wide Saturation_Counter
 *          (design C2).
 *
 * Plain C header (only C standard headers), so the packed-field accessor
 * headers and their modules can use it without the micro T-Kernel or the
 * Pico SDK (Requirement 9.5). Implementations:
 *
 *   target, kernel side  os_glue.c    crit_dsp_*, sat_note, sat_count
 *   target, SDK side     hal_board.c  crit_irq_*
 *   host tests           tests/host/stubs/crit_stub.c, sat_stub.c
 *
 * Dispatch-disabled sections (crit_dsp_enter/exit) are used by tasks for
 * Storage_Units that two tasks write. They do not nest (tk_dis_dsp does not
 * count) and interrupt handlers never use them. Inside a section: only
 * loads, mask operations and stores, no kernel call that can block
 * (Requirement 9.2).
 *
 * Interrupt-masked sections (crit_irq_enter/exit) are used for data shared
 * with an interrupt handler. They may nest; each crit_irq_exit() gets the
 * value its crit_irq_enter() returned, in LIFO order.
 *
 * Both kinds are timed by the perf.h probes PERF_CS_DSP and PERF_CS_IRQ
 * (APP_PERF = 1, limit PERF_LIMIT_CS_US = 10 us, Requirements 9.6, 10.2);
 * a nested interrupt-masked section is timed as part of its outermost one.
 */
#ifndef CRIT_H
#define CRIT_H

#include <stdint.h>

void     crit_dsp_enter(void);           /* tk_dis_dsp(), APP_PERF: start CS timer        */
void     crit_dsp_exit(void);            /* APP_PERF: record CS length, then tk_ena_dsp() */
uint32_t crit_irq_enter(void);           /* save_and_disable_interrupts()                 */
void     crit_irq_exit(uint32_t saved);  /* restore_interrupts(saved)                     */

/* Saturation_Counter (Requirement 1.10): 0 at boot, sat_note(n) adds n in one
   dispatch-disabled section and holds at 65,535 instead of wrapping. Called
   once per store that clamped a value or replaced a NaN; never from an
   interrupt handler and never inside a crit_dsp_* section. */
#define SAT_COUNT_MAX (65535u)

void     sat_note(uint32_t n);
uint16_t sat_count(void);                /* single 16-bit load */

#endif /* CRIT_H */
