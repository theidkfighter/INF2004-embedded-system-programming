/**
 * @file    base_motion.c (Baseline_Build harness, host only)
 * @brief   Baseline_Build MC_TURN / MC_BYPASS targets and timeouts for the
 *          differential Property 11 (Requirement 4.10).
 *
 * The Baseline_Build computes these inside tsk_motion.c, in LOCAL (static)
 * functions next to kernel calls: timeout_for() and motion_start_turn() /
 * motion_start_move() build the float command, handle_cmd() turns it into the
 * primitive's path length, run_primitive() ends the primitive when the mean
 * absolute wheel distance reaches that length. To run that exact code, this
 * file #includes the extracted tsk_motion.c (build/base/src/, found through
 * -I) and supplies the kernel, os_glue and hal symbols it references:
 *   - tk/tkernel.h comes from baseline/shim/ (no-op dispatch control);
 *   - post_motion() captures the command instead of posting it;
 *   - the hal_* functions return fixed values (time 0, encoders 0).
 * The task body tsk_motion() and motion_wait() are compiled but never called.
 *
 * Exported (all prefixed base_ in libbase.so by fetch_baseline.sh):
 *   bh_motion_reset()                 params_init(), no active primitive
 *   bh_timeout_for(path_mm, speed)    timeout_for() as is
 *   bh_motion_turn(deg, speed, ...)   motion_start_turn() + handle_cmd()
 *   bh_motion_move(dist, speed, ...)  motion_start_move() + handle_cmd()
 * The last two return 1 when a command was posted and fill:
 *   *p_timeout_ms   command timeout
 *   *p_target_mm    g_prim.target_mm (path length each wheel must cover)
 *   *p_counts       first encoder count n (both wheels at +n from the start)
 *                   at which run_primitive() reports MOTION_OK, -1 if
 *                   mm-per-count is 0 or more than 10^7 counts are needed
 */
#include "tsk_motion.c"

/* ---- host replacements for the kernel / os_glue / hal symbols ---------- */
volatile uint8_t  g_hb[HB_COUNT];
ID                g_flg_sys;
ID                g_mbx_motion;

static motion_cmd_t g_bh_last;
static uint32_t     g_bh_posts;
static uint32_t     g_bh_flags;
static uint32_t     g_bh_faults;

bool post_motion(const motion_cmd_t *p_cmd)
{
    g_bh_last = *p_cmd;
    g_bh_posts++;
    return true;
}

void        flag_set(uint32_t bits)   { g_bh_flags |= bits; }
void        flag_clear(uint32_t bits) { g_bh_flags &= ~bits; }
uint32_t    flag_peek(void)           { return g_bh_flags; }
void        fault_set(uint32_t bits)  { g_bh_faults |= bits; }
app_msg_t  *msg_recv(ID mbxid, TMO tmo) { (void)mbxid; (void)tmo; return NULL; }
void        msg_free(app_msg_t *p_msg)  { (void)p_msg; }
void        odometry_put(const odometry_t *p_in) { (void)p_in; }
void        motion_done_put(uint16_t id, motion_result_t result) { (void)id; (void)result; }
void        crit_dsp_enter(void)      { }
void        crit_dsp_exit(void)       { }
void        sat_note(uint32_t n)      { (void)n; }

uint64_t hal_time_us(void)                       { return 0u; }
uint32_t hal_time_ms(void)                       { return 0u; }
int32_t  hal_enc_count(uint32_t wheel)           { (void)wheel; return 0; }
uint32_t hal_odometer(void)                      { return 0u; }
bool     hal_motor_killed(void)                  { return false; }
void     hal_motor_set(uint32_t motor, float duty) { (void)motor; (void)duty; }

/* ---- harness API -------------------------------------------------------- */
#define BH_MAX_COUNTS (10000000)

void bh_motion_reset(void)
{
    params_init();
    g_prim.b_active = false;
    g_next_id       = 1u;
    g_bh_posts      = 0u;
    g_bh_flags      = 0u;
    g_bh_faults     = 0u;
}

uint32_t bh_timeout_for(float path_mm, float speed)
{
    return timeout_for(path_mm, speed);
}

static int32_t counts_to_done(void)
{
    float mmpc = params_mm_per_count();

    if (mmpc <= 0.0f)
    {
        return -1;
    }
    for (int32_t n = 0; n <= BH_MAX_COUNTS; n++)
    {
        run_primitive(n, n, mmpc, 0u);
        if (!g_prim.b_active)
        {
            return n;
        }
    }
    g_prim.b_active = false;
    return -1;
}

static int32_t load(uint16_t id, uint32_t *p_timeout_ms, float *p_target_mm, int32_t *p_counts)
{
    if ((id == 0u) || (g_bh_posts == 0u))
    {
        return 0;
    }
    g_prim.b_active = false;
    handle_cmd(&g_bh_last, 0, 0, 0u);
    *p_timeout_ms = g_bh_last.timeout_ms;
    *p_target_mm  = g_prim.target_mm;
    *p_counts     = counts_to_done();
    return 1;
}

int32_t bh_motion_turn(float deg_cw, float speed_mm_s,
                       uint32_t *p_timeout_ms, float *p_target_mm, int32_t *p_counts)
{
    g_bh_posts = 0u;
    return load(motion_start_turn(deg_cw, speed_mm_s, SRC_NAV), p_timeout_ms, p_target_mm, p_counts);
}

int32_t bh_motion_move(float dist_mm, float speed_mm_s,
                       uint32_t *p_timeout_ms, float *p_target_mm, int32_t *p_counts)
{
    g_bh_posts = 0u;
    return load(motion_start_move(dist_mm, speed_mm_s, SRC_NAV), p_timeout_ms, p_target_mm, p_counts);
}
