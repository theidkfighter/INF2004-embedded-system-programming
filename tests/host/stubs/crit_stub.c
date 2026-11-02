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
 *
 * Watches: crit_stub_watch() registers shared Storage_Units. The stub keeps a
 * shadow copy, so a byte that changed while no dispatch-disabled section was
 * open counts as a write outside a section, and a section that changed a
 * watched byte is counted once.
 *
 * Hooks: on the target another task can run just before a section starts and
 * just after it ends, never inside it. The enter and exit hooks stand for that
 * task, so a test can show that a read or write really is inside the section.
 */
#include <string.h>
#include "host_stubs.h"

#define IRQ_TOKEN_BASE (0x5EC70000u)    /* value returned by crit_irq_enter */
#define WATCH_MAX      (8u)
#define WATCH_BYTES    (64u)

typedef struct
{
    const volatile uint8_t *p;
    uint32_t                n;
    uint8_t                 shadow[WATCH_BYTES];
} watch_t;

static watch_t  s_watch[WATCH_MAX];
static uint32_t s_n_watch;
static uint32_t s_outside;              /* checks that found an outside write */
static uint32_t s_changed;              /* sections that changed a watched byte */
static void   (*s_enter_hook)(void);
static void   (*s_exit_hook)(void);

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

static bool watch_changed(void)
{
    for (uint32_t i = 0u; i < s_n_watch; i++)
    {
        if (memcmp((const void *)s_watch[i].p, s_watch[i].shadow, s_watch[i].n) != 0)
        {
            return true;
        }
    }
    return false;
}

static void watch_sync(void)
{
    for (uint32_t i = 0u; i < s_n_watch; i++)
    {
        memcpy(s_watch[i].shadow, (const void *)s_watch[i].p, s_watch[i].n);
    }
}

void crit_dsp_enter(void)
{
    s_dsp_enters++;
    if (s_dsp_depth > 0u)
    {
        note_error(CRIT_ERR_DSP_NESTED);
    }
    else
    {
        s_outside += watch_changed() ? 1u : 0u;
        if (s_enter_hook != NULL)
        {
            s_enter_hook();
        }
        watch_sync();
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
    if (s_dsp_depth == 0u)
    {
        s_changed += watch_changed() ? 1u : 0u;
        watch_sync();
        if (s_exit_hook != NULL)
        {
            s_exit_hook();
            watch_sync();
        }
    }
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
    s_n_watch    = 0u;
    s_outside    = 0u;
    s_changed    = 0u;
    s_enter_hook = NULL;
    s_exit_hook  = NULL;
}

bool crit_stub_watch(const volatile void *p, uint32_t n)
{
    if ((s_n_watch >= WATCH_MAX) || (n > WATCH_BYTES))
    {
        return false;
    }
    s_watch[s_n_watch].p = (const volatile uint8_t *)p;
    s_watch[s_n_watch].n = n;
    s_n_watch++;
    watch_sync();
    return true;
}

uint32_t crit_stub_outside_writes(void)
{
    /* also catches a write after the last section closed */
    if (s_dsp_depth == 0u)
    {
        s_outside += watch_changed() ? 1u : 0u;
        watch_sync();
    }
    return s_outside;
}

uint32_t crit_stub_changed_sections(void) { return s_changed; }

void crit_stub_set_hooks(void (*p_enter)(void), void (*p_exit)(void))
{
    s_enter_hook = p_enter;
    s_exit_hook  = p_exit;
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
