/**
 * @file    perf.h
 * @brief   Timing probes for the performance evidence (build option APP_PERF).
 *
 * APP_PERF = 1: each probe records, per probe id, the number of samples, the
 * largest and the summed time in microseconds (RP2040 TIMER, one timerawl
 * read) and how many samples exceeded the limit given at the probe site.
 * TSK_TELEMETRY prints one line per probe every 2 s:
 *   [perf] <name> n=<count> max=<us> avg=<us> over=<count>
 * and runs the telemetry build benchmark once ("[perf] tlm_bench ...").
 *
 * APP_PERF = 0 (default): every probe macro expands to nothing and the
 * functions are empty inline functions, so the firmware is unchanged.
 *
 * Plain C header (no micro T-Kernel or Pico SDK header): used by the task
 * code and by the drivers. The table and the functions are in os_glue.c.
 *
 * Concurrency: each probe id is updated by one context only. The two
 * Critical_Section probes are updated just before the section ends, while
 * dispatch or interrupts are still disabled, so no update races another.
 */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>

#ifndef APP_PERF
#define APP_PERF 0
#endif

/* Probe ids. The report name is the id without PERF_, in lower case. */
enum
{
    PERF_ISR_GPIO = 0,      /* hal_gpio_isr, entry to exit */
    PERF_ISR_TIMER,         /* hal_timer_isr, entry to exit */
    PERF_TSK_SAFETY,        /* TSK_SAFETY_MON, release to end of cycle (10 ms) */
    PERF_TSK_LINE,          /* TSK_LINE_BARCODE, release to end of cycle (10 ms) */
    PERF_TSK_MOTION,        /* TSK_MOTION_CTRL, release to end of cycle (20 ms) */
    PERF_TSK_IMU,           /* TSK_TERRAIN_IMU, release to end of cycle (20 ms) */
    PERF_TLM_ENCODE,        /* one telemetry frame build, excluding net_post_frame */
    PERF_CS_DSP,            /* every Critical_Section with dispatch disabled */
    PERF_CS_IRQ,            /* every Critical_Section with interrupts masked */
    PERF_COUNT
};

/* Limits counted in "over" (Requirements 9.6, 10.2, 20.1) */
#define PERF_LIMIT_10MS_US   (10000u)
#define PERF_LIMIT_20MS_US   (20000u)
#define PERF_LIMIT_CS_US     (10u)
#define PERF_NO_LIMIT        (0xFFFFFFFFu)

#if APP_PERF

#include "hal.h"

typedef struct
{
    uint32_t n;             /* samples */
    uint32_t max_us;
    uint32_t sum_us;        /* saturates at 0xFFFFFFFF */
    uint32_t over;          /* samples with us > limit */
} perf_stat_t;

extern perf_stat_t g_perf[PERF_COUNT];

/* PERF_BEGIN declares the start timestamp v; PERF_END records the time since v. */
#define PERF_BEGIN(v)            uint32_t v = hal_time_us32()
#define PERF_END(id, v, lim)     perf_add((uint32_t)(id), hal_time_us32() - (v), (lim))
/* Periodic tasks: time from the cyclic handler's release (perf_release) to
   this point. Placed just before the task sleeps, so every path of the cycle
   (including "continue") ends here. */
#define PERF_CYCLE_END(id, lim)  perf_cycle_end((uint32_t)(id), (lim))

void perf_add(uint32_t id, uint32_t us, uint32_t limit_us);  /* n++, max, saturating sum, over++ if us > limit */
void perf_release(uint32_t id);                              /* cyclic handler: release timestamp of a task */
void perf_cycle_end(uint32_t id, uint32_t limit_us);         /* task: end of the cycle of the oldest release */
void perf_reset(void);                                       /* TSK_NAV_PLANNER start_run() */
void perf_report(void);                                      /* TSK_TELEMETRY console_status(), every 2 s */

#else /* !APP_PERF */

#define PERF_BEGIN(v)
#define PERF_END(id, v, lim)
#define PERF_CYCLE_END(id, lim)

static inline void perf_add(uint32_t id, uint32_t us, uint32_t limit_us)
{
    (void)id;
    (void)us;
    (void)limit_us;
}

static inline void perf_release(uint32_t id)
{
    (void)id;
}

static inline void perf_cycle_end(uint32_t id, uint32_t limit_us)
{
    (void)id;
    (void)limit_us;
}

static inline void perf_reset(void)
{
}

static inline void perf_report(void)
{
}

#endif /* APP_PERF */

#endif /* PERF_H */
