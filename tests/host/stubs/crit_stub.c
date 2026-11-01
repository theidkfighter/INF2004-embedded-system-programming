/**
 * @file    crit_stub.c
 * @brief   Host stand-in for the Critical_Section hooks of crit.h.
 *
 * Target: crit_dsp_* = tk_dis_dsp()/tk_ena_dsp() (os_glue.c),
 *         crit_irq_* = save_and_disable_interrupts()/restore_interrupts()
 *         (hal_board.c).
 * Host:   counts every enter and exit, tracks the nesting depth and records
 *         nesting errors, so tests can check that sections are balanced and
 *         that a shared update ran inside one (Property 18).
 */
#include "host_stubs.h"

#define IRQ_TOKEN_BASE (0x5EC70000u)    /* value returned by crit_irq_enter */

static uint32_t s_dsp_enters;
static uint32_t s_dsp_exits;
static uint32_t s_dsp_depth;
static uint32_t s_dsp_max;
static uint32_t s_irq_enters;
static uint32_t s_irq_exits;
static uint32_t s_irq_depth;
static uint32_t s_irq_max;
static uint32_t s_errors;
static uint32_t s_err_flags;

static void note_error(uint32_t flag)
{
    s_errors++;
    s_err_flags |= flag;
}

void crit_dsp_enter(void)
{
    s_dsp_enters++;
    if (s_dsp_depth > 0u)
    {
        note_error(CRIT_ERR_DSP_NESTED);
    }
    s_dsp_depth++;
    if (s_dsp_depth > s_dsp_max)
    {
        s_dsp_max = s_dsp_depth;
    }
}

void crit_dsp_exit(void)
{
    s_dsp_exits++;
    if (s_dsp_depth == 0u)
    {
        note_error(CRIT_ERR_DSP_UNDERFLOW);
        return;
    }
    s_dsp_depth--;
}

uint32_t crit_irq_enter(void)
{
    uint32_t token = IRQ_TOKEN_BASE | s_irq_depth;

    s_irq_enters++;
    s_irq_depth++;
    if (s_irq_depth > s_irq_max)
    {
        s_irq_max = s_irq_depth;
    }
    return token;
}

void crit_irq_exit(uint32_t saved)
{
    s_irq_exits++;
    if (s_irq_depth == 0u)
    {
        note_error(CRIT_ERR_IRQ_UNDERFLOW);
        return;
    }
    s_irq_depth--;
    if (saved != (IRQ_TOKEN_BASE | s_irq_depth))
    {
        note_error(CRIT_ERR_IRQ_TOKEN);
    }
}

void crit_stub_reset(void)
{
    s_dsp_enters = 0u;
    s_dsp_exits  = 0u;
    s_dsp_depth  = 0u;
    s_dsp_max    = 0u;
    s_irq_enters = 0u;
    s_irq_exits  = 0u;
    s_irq_depth  = 0u;
    s_irq_max    = 0u;
    s_errors     = 0u;
    s_err_flags  = 0u;
}

uint32_t crit_stub_dsp_enters(void)    { return s_dsp_enters; }
uint32_t crit_stub_dsp_exits(void)     { return s_dsp_exits; }
uint32_t crit_stub_dsp_depth(void)     { return s_dsp_depth; }
uint32_t crit_stub_dsp_max_depth(void) { return s_dsp_max; }
uint32_t crit_stub_irq_enters(void)    { return s_irq_enters; }
uint32_t crit_stub_irq_exits(void)     { return s_irq_exits; }
uint32_t crit_stub_irq_depth(void)     { return s_irq_depth; }
uint32_t crit_stub_irq_max_depth(void) { return s_irq_max; }
uint32_t crit_stub_errors(void)        { return s_errors; }
uint32_t crit_stub_error_flags(void)   { return s_err_flags; }
bool     crit_stub_in_dsp(void)        { return s_dsp_depth > 0u; }
bool     crit_stub_in_irq(void)        { return s_irq_depth > 0u; }
