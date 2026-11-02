/**
 * @file    host_stubs.h
 * @brief   Host_Test_Suite only: replacements for the kernel and SDK side of
 *          the firmware, plus the control/inspection API the tests call
 *          through ctypes (py/conftest.py).
 *
 * The firmware prototypes come from app_program/crit.h (and evt.h) once those
 * headers exist (task 3.5 / 3.16). Until then they are declared here with the
 * signatures given in the design (C2, C8), so the stubs and the firmware agree.
 */
#ifndef HOST_STUBS_H
#define HOST_STUBS_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__has_include)
#  if __has_include("crit.h")
#    include "crit.h"
#    define HOST_HAVE_CRIT_H 1
#  endif
#endif

#ifndef HOST_HAVE_CRIT_H
/* Same prototypes as app_program/crit.h (design C2). */
void     crit_dsp_enter(void);
void     crit_dsp_exit(void);
uint32_t crit_irq_enter(void);
void     crit_irq_exit(uint32_t saved);
void     sat_note(uint32_t n);
uint16_t sat_count(void);
#endif

/* ---- crit_stub.c ----------------------------------------------------------
 * Counts entries and exits of both Critical_Section kinds and checks nesting.
 * tk_dis_dsp() does not nest on the target, so a second crit_dsp_enter()
 * before crit_dsp_exit() is an error. save_and_disable_interrupts() does
 * nest, so IRQ sections may nest but must be left in LIFO order with the
 * value their crit_irq_enter() returned. */
#define CRIT_ERR_DSP_NESTED    (1u << 0)   /* crit_dsp_enter inside a DSP section  */
#define CRIT_ERR_DSP_UNDERFLOW (1u << 1)   /* crit_dsp_exit with no DSP section    */
#define CRIT_ERR_IRQ_UNDERFLOW (1u << 2)   /* crit_irq_exit with no IRQ section    */
#define CRIT_ERR_IRQ_TOKEN     (1u << 3)   /* crit_irq_exit with the wrong value   */

void     crit_stub_reset(void);
uint32_t crit_stub_dsp_enters(void);
uint32_t crit_stub_dsp_exits(void);
uint32_t crit_stub_dsp_depth(void);
uint32_t crit_stub_dsp_max_depth(void);
uint32_t crit_stub_irq_enters(void);
uint32_t crit_stub_irq_exits(void);
uint32_t crit_stub_irq_depth(void);
uint32_t crit_stub_irq_max_depth(void);
uint32_t crit_stub_errors(void);           /* number of nesting errors          */
uint32_t crit_stub_error_flags(void);      /* OR of the CRIT_ERR_* bits seen    */
bool     crit_stub_in_dsp(void);
bool     crit_stub_in_irq(void);
/* Watches and hooks (Property 18); crit_stub_reset() clears them. At most 8
   regions of 64 bytes; a region past that returns false. */
bool     crit_stub_watch(const volatile void *p, uint32_t n);
uint32_t crit_stub_outside_writes(void);   /* writes seen with no section open  */
uint32_t crit_stub_changed_sections(void); /* sections that changed a watched byte */
void     crit_stub_set_hooks(void (*p_enter)(void), void (*p_exit)(void));

/* ---- sat_stub.c -----------------------------------------------------------
 * Saturation_Counter with the firmware rule (Requirement 1.10): starts at 0,
 * adds n inside one dispatch-disabled section, holds at 65,535. */
void     sat_stub_reset(void);             /* count = 0, calls = 0              */
void     sat_stub_set(uint16_t value);     /* preload, e.g. 65,534              */
uint32_t sat_stub_calls(void);             /* number of sat_note() calls        */

/* ---- os_stub.c ------------------------------------------------------------
 * Captures posted Inter_Task_Message bodies in order and models the 40-block
 * Message_Pool (a post fails without waiting when no block is free). Also
 * replaces the g_flg_sys event flag helpers of os_glue.h. */
#define OS_STUB_MBX_MOTION    (0u)
#define OS_STUB_MBX_NAV       (1u)
#define OS_STUB_MBX_OBSTACLE  (2u)
#define OS_STUB_MBX_TLM       (3u)
#define OS_STUB_MBX_COUNT     (4u)
#define OS_STUB_POOL_BLOCKS   (40u)        /* Baseline_Build Message_Pool count */
#define OS_STUB_MAX_BODY      (128u)       /* bytes kept per captured message   */

void     os_stub_reset(void);              /* empty capture, pool full, flags 0 */
void     os_stub_set_pool_empty(bool b_empty);   /* force msg_alloc() == NULL   */
bool     os_stub_post(uint32_t mbx, const void *p_body, uint32_t len);
uint32_t os_stub_count(void);              /* messages captured                 */
uint32_t os_stub_count_mbx(uint32_t mbx);
uint32_t os_stub_rejected(void);           /* posts refused (pool empty, bad)   */
/* Copies captured message idx (0 = oldest) to p_out (up to cap bytes) and
   returns its body length, or 0 when idx is out of range. *p_mbx may be NULL. */
uint32_t os_stub_get(uint32_t idx, uint32_t *p_mbx, void *p_out, uint32_t cap);
void     os_stub_free(uint32_t n);         /* receiver released n blocks        */

/* Event flag helpers with the os_glue.h signatures. */
void     flag_set(uint32_t bits);
void     flag_clear(uint32_t bits);
uint32_t flag_peek(void);
bool     flag_take(uint32_t bits, uint32_t *p_got);
void     os_stub_flags_load(uint32_t bits);   /* set the whole flag pattern     */
uint32_t os_stub_flag_set_calls(void);

/* ---- bitpack_export.c -----------------------------------------------------
 * Out-of-line wrappers of the app_program/bitpack.h inline helpers and
 * macros (design C1). bpx_quant takes the bp_qspec_t members as arguments;
 * p_sat may be NULL. */
uint32_t bpx_mask(uint32_t w);
uint32_t bpx_get(uint32_t u, uint32_t sh, uint32_t w);
uint32_t bpx_put(uint32_t u, uint32_t sh, uint32_t w, uint32_t v);
int32_t  bpx_sext(uint32_t code, uint32_t w);
uint32_t bpx_tc(int32_t v, uint32_t w);
bool     bpx_set_checked(uint32_t *p_unit, uint32_t sh, uint32_t w,
                         uint32_t v, uint32_t lo, uint32_t hi);
int32_t  bpx_quant(float x, int32_t scale, int32_t lo_code, int32_t hi_code,
                   int32_t nan_code, bool *p_sat);
float    bpx_dequant(int32_t code, int32_t scale);
uint32_t bpx_sat_inc(uint32_t v, uint32_t max);

#endif /* HOST_STUBS_H */
