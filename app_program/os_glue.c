/**
 * @file    os_glue.c
 * @brief   IS-02 kernel objects, task creation, interrupts and helpers
 *          (see os_glue.h). micro T-Kernel side.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "os_glue.h"
#include "crit.h"
#include "evt.h"
#include "status.h"
#include "bitpack.h"
#include "hal.h"
#include "jfmt.h"
#include "perf.h"

#define MSG_BLOCKS      (40)
#define STACK_DEFAULT   (3072)
#define STACK_TLM       (4096)
#define TICK_MS         (10)

/* task bodies (tsk_*.c) */
IMPORT void tsk_safety(INT stacd, void *exinf);
IMPORT void tsk_motion(INT stacd, void *exinf);
IMPORT void tsk_line(INT stacd, void *exinf);
IMPORT void tsk_obstacle(INT stacd, void *exinf);
IMPORT void tsk_imu(INT stacd, void *exinf);
IMPORT void tsk_nav(INT stacd, void *exinf);
IMPORT void tsk_telemetry(INT stacd, void *exinf);

volatile uint32_t g_hb[HB_COUNT];
volatile uint32_t g_reacq_seq;

ID g_flg_sys;
ID g_mbx_motion;
ID g_mbx_nav;
ID g_mbx_obstacle;
ID g_mbx_tlm;
ID g_mtx_i2c;
ID g_tsk_safety, g_tsk_motion, g_tsk_line, g_tsk_obstacle, g_tsk_imu, g_tsk_nav, g_tsk_tlm;

LOCAL ID                g_mpf_msg;
LOCAL ID                g_cyc_tick;
LOCAL volatile uint32_t g_buzz_pattern;
LOCAL volatile uint32_t g_tick_n;
LOCAL odometry_t        g_odo;

/* message pool storage: blocks of T_MSG + body, word aligned */
#define MSG_BLK_WORDS   ((sizeof(app_msg_t) + 3u) / 4u)
LOCAL UW g_mpf_buf[(MSG_BLOCKS * MSG_BLK_WORDS) + MSG_BLOCKS];

/* ------------------------------------------------------------------------ */

LOCAL bool made(const char *p_what, ID id)
{
    if (id < E_OK)
    {
        tm_printf((UB *)"[init] %s failed, er=%d\n", p_what, (INT)id);
        return false;
    }
    return true;
}

bool os_create_objects(void)
{
    T_CFLG cflg = { .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0 };
    T_CMBX cmbx = { .mbxatr = TA_TFIFO | TA_MFIFO };
    T_CMTX cmtx = { .mtxatr = TA_TFIFO | TA_INHERIT };
    T_CMPF cmpf = { .mpfatr = TA_TFIFO | TA_RNG3, .mpfcnt = MSG_BLOCKS,
                    .blfsz = (SZ)(MSG_BLK_WORDS * 4u), .bufptr = g_mpf_buf };

    g_flg_sys      = tk_cre_flg(&cflg);
    g_mbx_motion   = tk_cre_mbx(&cmbx);
    g_mbx_nav      = tk_cre_mbx(&cmbx);
    g_mbx_obstacle = tk_cre_mbx(&cmbx);
    g_mbx_tlm      = tk_cre_mbx(&cmbx);
    g_mtx_i2c      = tk_cre_mtx(&cmtx);
    g_mpf_msg      = tk_cre_mpf(&cmpf);

    return made("tk_cre_flg", g_flg_sys) && made("tk_cre_mbx(motion)", g_mbx_motion) &&
           made("tk_cre_mbx(nav)", g_mbx_nav) && made("tk_cre_mbx(obstacle)", g_mbx_obstacle) &&
           made("tk_cre_mbx(telemetry)", g_mbx_tlm) && made("tk_cre_mtx(i2c)", g_mtx_i2c) &&
           made("tk_cre_mpf(msg)", g_mpf_msg);
}

LOCAL ID start_task(const char *p_name, FP task, PRI pri, SZ stack)
{
    T_CTSK ctsk = { 0 };
    ID     id;

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = task;
    ctsk.itskpri = pri;
    ctsk.stksz   = stack;
    id = tk_cre_tsk(&ctsk);
    if (made(p_name, id))
    {
        (void)tk_sta_tsk(id, 0);
    }
    return id;
}

bool os_start_tasks(void)
{
    g_tsk_safety   = start_task("TSK_SAFETY_MON",   (FP)tsk_safety,    PRI_SAFETY,    STACK_DEFAULT);
    g_tsk_motion   = start_task("TSK_MOTION_CTRL",  (FP)tsk_motion,    PRI_MOTION,    STACK_DEFAULT);
    g_tsk_line     = start_task("TSK_LINE_BARCODE", (FP)tsk_line,      PRI_LINE,      STACK_DEFAULT);
    g_tsk_obstacle = start_task("TSK_OBSTACLE_SCAN",(FP)tsk_obstacle,  PRI_OBSTACLE,  STACK_DEFAULT);
    g_tsk_imu      = start_task("TSK_TERRAIN_IMU",  (FP)tsk_imu,       PRI_IMU,       STACK_DEFAULT);
    g_tsk_nav      = start_task("TSK_NAV_PLANNER",  (FP)tsk_nav,       PRI_NAV,       STACK_DEFAULT);
    g_tsk_tlm      = start_task("TSK_TELEMETRY",    (FP)tsk_telemetry, PRI_TELEMETRY, STACK_TLM);
    return (g_tsk_safety > 0) && (g_tsk_motion > 0) && (g_tsk_line > 0) && (g_tsk_obstacle > 0) &&
           (g_tsk_imu > 0) && (g_tsk_nav > 0) && (g_tsk_tlm > 0);
}

/* ------------------------------------------------------------------------ */
/* Timing: one 10 ms cyclic handler releases the periodic tasks              */
/* (safety + line every 10 ms, motion + IMU every 20 ms on alternate ticks). */
/* ------------------------------------------------------------------------ */

LOCAL void cyc_tick(void *exinf)
{
    (void)exinf;
    g_tick_n++;
    perf_release(PERF_TSK_SAFETY);
    (void)tk_wup_tsk(g_tsk_safety);
    perf_release(PERF_TSK_LINE);
    (void)tk_wup_tsk(g_tsk_line);
    if ((g_tick_n & 1u) == 0u)
    {
        perf_release(PERF_TSK_MOTION);
        (void)tk_wup_tsk(g_tsk_motion);
    }
    else
    {
        perf_release(PERF_TSK_IMU);
        (void)tk_wup_tsk(g_tsk_imu);
    }
}

LOCAL void on_int1(void)
{
    (void)tk_set_flg(g_flg_sys, EVF_COLLISION);
    (void)tk_wup_tsk(g_tsk_safety);                 /* react now, not at the next tick */
}

bool os_start_interrupts(void)
{
    T_DINT dint;
    T_CCYC ccyc = { 0 };
    ER     er;

    hal_set_int1_callback(on_int1);

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)hal_gpio_isr;
    er = tk_def_int(HAL_IRQ_GPIO, &dint);
    if (er < E_OK)
    {
        tm_printf((UB *)"[init] tk_def_int(GPIO) failed, er=%d\n", (INT)er);
        return false;
    }
    dint.inthdr = (FP)hal_timer_isr;
    er = tk_def_int(HAL_IRQ_TIMER3, &dint);
    if (er < E_OK)
    {
        tm_printf((UB *)"[init] tk_def_int(TIMER3) failed, er=%d\n", (INT)er);
        return false;
    }
    hal_irq_enable_sources();
    ClearInt(HAL_IRQ_GPIO);
    ClearInt(HAL_IRQ_TIMER3);
    EnableInt(HAL_IRQ_GPIO, 1);       /* encoders first: an edge must never be missed */
    EnableInt(HAL_IRQ_TIMER3, 2);

    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)cyc_tick;
    ccyc.cyctim = TICK_MS;
    ccyc.cycphs = TICK_MS;
    g_cyc_tick  = tk_cre_cyc(&ccyc);
    return made("tk_cre_cyc(tick)", g_cyc_tick);
}

/* ------------------------------------------------------------------------ */
/* Messages                                                                  */
/* ------------------------------------------------------------------------ */

app_msg_t *msg_alloc(void)
{
    void *p_blk = NULL;

    if (tk_get_mpf(g_mpf_msg, &p_blk, TMO_POL) < E_OK)
    {
        return NULL;
    }
    return (app_msg_t *)p_blk;
}

void msg_free(app_msg_t *p_msg)
{
    if (p_msg != NULL)
    {
        (void)tk_rel_mpf(g_mpf_msg, p_msg);
    }
}

app_msg_t *msg_recv(ID mbxid, TMO tmo)
{
    T_MSG *p_msg = NULL;

    if (tk_rcv_mbx(mbxid, &p_msg, tmo) < E_OK)
    {
        return NULL;
    }
    return (app_msg_t *)p_msg;
}

LOCAL bool send(ID mbxid, app_msg_t *p_msg)
{
    if (tk_snd_mbx(mbxid, &p_msg->hdr) < E_OK)
    {
        msg_free(p_msg);
        return false;
    }
    return true;
}

bool post_motion(const motion_cmd_t *p_cmd)
{
    app_msg_t *p_msg = msg_alloc();

    if (p_msg == NULL)
    {
        return false;
    }
    p_msg->body.motion = *p_cmd;
    return send(g_mbx_motion, p_msg);
}

bool post_nav(nav_event_type_t type, char code)
{
    app_msg_t *p_msg = msg_alloc();

    if (p_msg == NULL)
    {
        return false;
    }
    p_msg->body.nav.type = (uint8_t)type;
    p_msg->body.nav.code = code;
    return send(g_mbx_nav, p_msg);
}

bool post_obstacle(const obstacle_profile_t *p_prof)
{
    app_msg_t *p_msg = msg_alloc();

    if (p_msg == NULL)
    {
        return false;
    }
    p_msg->body.obstacle = *p_prof;
    return send(g_mbx_obstacle, p_msg);
}

bool post_evt(const evt_msg_t *p_m)
{
    app_msg_t *p_msg = msg_alloc();

    if (p_msg == NULL)
    {
        return false;
    }
    p_msg->body.event = *p_m;
    return send(g_mbx_tlm, p_msg);
}

uint32_t evt_now_ms(void)
{
    return hal_time_ms();
}

/* ------------------------------------------------------------------------ */
/* Flags, buzzer, state                                                      */
/* ------------------------------------------------------------------------ */

void flag_set(uint32_t bits)
{
    (void)tk_set_flg(g_flg_sys, (UINT)bits);
}

void flag_clear(uint32_t bits)
{
    (void)tk_clr_flg(g_flg_sys, (UINT)~bits);
}

uint32_t flag_peek(void)
{
    T_RFLG rflg;

    if (tk_ref_flg(g_flg_sys, &rflg) < E_OK)
    {
        return 0u;
    }
    return (uint32_t)rflg.flgptn;
}

bool flag_take(uint32_t bits, uint32_t *p_got)
{
    UINT ptn = 0;

    if (tk_wai_flg(g_flg_sys, (UINT)bits, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) < E_OK)
    {
        *p_got = 0u;
        return false;
    }
    *p_got = (uint32_t)ptn & bits;
    return true;
}

void buzzer_request(uint32_t pattern)
{
    g_buzz_pattern = pattern;
    flag_set(EVF_BUZZER_REQ);
}

uint32_t buzzer_pattern_take(void)
{
    uint32_t p = g_buzz_pattern;

    g_buzz_pattern = BUZZ_NONE;
    return p;
}

bool sys_set_state(sys_state_t state, bool b_safety)
{
    sys_state_t old;
    bool        b_ok = false;

    /* compare and set of the state field (status.h sys unit), one section */
    crit_dsp_enter();
    old = (sys_state_t)BP_GET(g_status.sys, ST_SH_STATE, ST_W_STATE);
    if (b_safety || ((old != SYS_ESTOP) && (old != SYS_ERROR)))
    {
        g_status.sys = BP_PUT(g_status.sys, ST_SH_STATE, ST_W_STATE, (uint32_t)state);
        b_ok = true;
    }
    crit_dsp_exit();

    if (b_ok && (old != state))
    {
        char line[64];

        tm_sprintf((UB *)line, (UB *)"[state] %s -> %s", sys_state_name(old), sys_state_name(state));
        log_line(line);
        (void)evt_post1(EVC_STATE, (uint32_t)state, 3u);
    }
    return b_ok;
}

sys_state_t sys_state(void)
{
    return st_state();
}

void fault_set(uint32_t fault_bits)
{
    st_fault_set(fault_bits);
}

void fault_clear(uint32_t fault_bits)
{
    st_fault_clear(fault_bits);
}

void odometry_get(odometry_t *p_out)
{
    PERF_BEGIN(t_cs);

    (void)tk_dis_dsp();
    *p_out = g_odo;
    PERF_END(PERF_CS_DSP, t_cs, PERF_LIMIT_CS_US);
    (void)tk_ena_dsp();
}

void odometry_put(const odometry_t *p_in)
{
    PERF_BEGIN(t_cs);

    (void)tk_dis_dsp();
    g_odo = *p_in;
    PERF_END(PERF_CS_DSP, t_cs, PERF_LIMIT_CS_US);
    (void)tk_ena_dsp();
}

/* ------------------------------------------------------------------------ */
/* Critical_Section with dispatch disabled and Saturation_Counter (crit.h)   */
/* ------------------------------------------------------------------------ */

#if APP_PERF
/* Start of the open dispatch-disabled section. Sections do not nest and
   interrupt handlers never use them, so one variable is enough: it is written
   only while dispatch is disabled, by the task that owns the section. */
LOCAL uint32_t g_cs_dsp_t0;
#endif

/* Saturation_Counter: 0 at boot (.bss), written only inside sat_note(). */
LOCAL volatile uint16_t g_sat_count;

void crit_dsp_enter(void)
{
    (void)tk_dis_dsp();
#if APP_PERF
    g_cs_dsp_t0 = hal_time_us32();
#endif
}

void crit_dsp_exit(void)
{
    /* probe recorded while dispatch is still disabled (perf.h) */
    PERF_END(PERF_CS_DSP, g_cs_dsp_t0, PERF_LIMIT_CS_US);
    (void)tk_ena_dsp();
}

void sat_note(uint32_t n)
{
    uint32_t v;

    if (n == 0u)
    {
        return;
    }
    crit_dsp_enter();
    v = (uint32_t)g_sat_count;
    v = (n > (SAT_COUNT_MAX - v)) ? SAT_COUNT_MAX : (v + n);   /* holds at 65,535 */
    g_sat_count = (uint16_t)v;
    crit_dsp_exit();
}

uint16_t sat_count(void)
{
    return g_sat_count;
}

void log_line(const char *p_text)
{
    tm_printf((UB *)"%s\n", p_text);
}

/* ------------------------------------------------------------------------ */
/* Timing probes (perf.h), APP_PERF = 1 only                                 */
/* ------------------------------------------------------------------------ */

#if APP_PERF

#define PERF_BARRIER()   __asm__ volatile ("" ::: "memory")   /* compiler barrier */
#define PERF_REL_FIRST   ((uint32_t)PERF_TSK_SAFETY)          /* the four task ids are consecutive */
#define PERF_REL_TASKS   (4u)
#define PERF_REL_SLOTS   (4u)                                 /* power of 2 */

perf_stat_t g_perf[PERF_COUNT];

/* Release timestamps of one periodic task: a single-producer single-consumer
   ring. cyc_tick pushes one per release, the task pops the oldest at the end
   of each cycle, so a cycle that overruns into the next release is still
   measured from its own release. A wake-up without a release (INT1 wakes
   the safety monitor) finds the ring empty and records nothing. */
typedef struct
{
    uint32_t          t_us[PERF_REL_SLOTS];
    volatile uint32_t head;          /* cyc_tick */
    volatile uint32_t tail;          /* the task */
    volatile uint32_t lost;          /* cyc_tick: ring full, timestamp dropped */
    uint32_t          lost_seen;     /* the task */
} perf_rel_t;

LOCAL perf_rel_t g_perf_rel[PERF_REL_TASKS];

LOCAL const char * const PERF_NAMES[PERF_COUNT] =
{
    "isr_gpio", "isr_timer", "tsk_safety", "tsk_line", "tsk_motion", "tsk_imu",
    "tlm_encode", "cs_dsp", "cs_irq"
};

void perf_add(uint32_t id, uint32_t us, uint32_t limit_us)
{
    perf_stat_t *p_s;

    if (id >= (uint32_t)PERF_COUNT)
    {
        return;
    }
    p_s = &g_perf[id];
    p_s->n++;
    if (us > p_s->max_us)
    {
        p_s->max_us = us;
    }
    p_s->sum_us = (us > (0xFFFFFFFFu - p_s->sum_us)) ? 0xFFFFFFFFu : (p_s->sum_us + us);
    if (us > limit_us)
    {
        p_s->over++;
    }
}

void perf_release(uint32_t id)
{
    uint32_t    k = id - PERF_REL_FIRST;
    perf_rel_t *p_r;
    uint32_t    head;

    if (k >= PERF_REL_TASKS)
    {
        return;
    }
    p_r  = &g_perf_rel[k];
    head = p_r->head;
    if ((head - p_r->tail) < PERF_REL_SLOTS)
    {
        p_r->t_us[head & (PERF_REL_SLOTS - 1u)] = hal_time_us32();
        PERF_BARRIER();                       /* entry before the index */
        p_r->head = head + 1u;
    }
    else
    {
        p_r->lost++;
    }
}

void perf_cycle_end(uint32_t id, uint32_t limit_us)
{
    uint32_t    now = hal_time_us32();
    uint32_t    k = id - PERF_REL_FIRST;
    perf_rel_t *p_r;
    uint32_t    tail;

    if (k >= PERF_REL_TASKS)
    {
        return;
    }
    p_r  = &g_perf_rel[k];
    tail = p_r->tail;
    if (tail != p_r->head)
    {
        uint32_t t_rel = p_r->t_us[tail & (PERF_REL_SLOTS - 1u)];

        PERF_BARRIER();                       /* entry before the index */
        p_r->tail = tail + 1u;
        perf_add(id, now - t_rel, limit_us);
    }
    else if (p_r->lost != p_r->lost_seen)
    {
        /* this cycle served a release whose timestamp was dropped because
           PERF_REL_SLOTS releases were still pending: count it as late */
        p_r->lost_seen++;
        perf_add(id, limit_us + 1u, limit_us);
    }
    else
    {
        /* woken without a release: not a periodic cycle */
    }
}

void perf_reset(void)
{
    UW imask;

    DI(imask);                                /* no probe updates part-way */
    for (uint32_t i = 0u; i < (uint32_t)PERF_COUNT; i++)
    {
        g_perf[i].n      = 0u;
        g_perf[i].max_us = 0u;
        g_perf[i].sum_us = 0u;
        g_perf[i].over   = 0u;
    }
    PERF_BARRIER();
    EI(imask);
}

void perf_report(void)
{
    for (uint32_t i = 0u; i < (uint32_t)PERF_COUNT; i++)
    {
        perf_stat_t s;
        char        line[96];
        jbuf_t      jb;
        UW          imask;

        DI(imask);                            /* one consistent copy of the probe */
        s = g_perf[i];
        PERF_BARRIER();
        EI(imask);

        jb_init(&jb, line, sizeof(line));
        jb_raw(&jb, "[perf] ");
        jb_raw(&jb, PERF_NAMES[i]);
        jb_raw(&jb, " n=");
        jb_uint(&jb, s.n);
        jb_raw(&jb, " max=");
        jb_uint(&jb, s.max_us);
        jb_raw(&jb, " avg=");
        jb_fixed(&jb, (s.n > 0u) ? ((float)s.sum_us / (float)s.n) : 0.0f, 1u);
        jb_raw(&jb, " over=");
        jb_uint(&jb, s.over);
        log_line(line);
    }
}

#endif /* APP_PERF */
