/**
 * @file    tsk_motion.c
 * @brief   TSK_MOTION_CTRL (DR Section 3, priority 2 in the DR, 20 ms):
 *          closed-loop wheel speed control, motion primitives, odometry.
 *
 * Each 20 ms cycle:
 *   1. odometry from the encoders (distance, heading, speed, forward accel)
 *   2. empty MBX_MOTION_CMD, keeping the latest command of each kind:
 *        STOP always wins; TURN/BYPASS replace the active primitive;
 *        LINE commands are ignored while a TURN/BYPASS is running
 *        (DR: "line corrections are ignored while a turn or bypass is in
 *        progress"); SPEED_LIMIT scales LINE speeds (hump: 60 %)
 *   3. run the active primitive (encoder-counted, with a timeout = encoder
 *      fault -> ERROR) or the latest LINE speeds (dropped after 100 ms)
 *   4. PI speed control per wheel -> PWM; encoder-loss check (DR 5)
 * The E-stop latch (EVF_ESTOP / hal_motor_killed) forces everything to 0.
 */
#include <tk/tkernel.h>

#include "os_glue.h"
#include "motion_api.h"
#include "speed_ctl.h"
#include "app_params.h"
#include "hal.h"
#include "robot_config.h"

#define LINE_STALE_MS     (100u)
#define DONE_WAIT_MARGIN  (1000u)
#define G_MM_S2           (9810.0f)
#define RAD_TO_DEG        (57.29578f)

typedef struct
{
    bool     b_active;
    uint8_t  type;           /* MC_TURN / MC_BYPASS */
    uint16_t id;
    float    target_mm;      /* path length each wheel must cover */
    float    speed;          /* mm/s, sign = direction for BYPASS */
    float    dir;            /* TURN: +1 clockwise */
    int32_t  start_r;
    int32_t  start_l;
    uint32_t t_start;
    uint32_t timeout_ms;
} primitive_t;

LOCAL wheel_ctl_t       g_w[2];            /* 0 = right, 1 = left */
LOCAL primitive_t       g_prim;
LOCAL float             g_line_l;
LOCAL float             g_line_r;
LOCAL uint32_t          g_line_t;
LOCAL float             g_scale = 1.0f;
LOCAL volatile uint16_t g_next_id = 1u;
LOCAL volatile uint16_t g_done_id;
LOCAL volatile uint32_t g_done_result;

/* ------------------------------------------------------------------------ */
/* API used by the other tasks                                               */
/* ------------------------------------------------------------------------ */

LOCAL uint16_t new_id(void)
{
    uint16_t id;

    (void)tk_dis_dsp();
    id = g_next_id;
    g_next_id = (uint16_t)((g_next_id == 0xFFFFu) ? 1u : (g_next_id + 1u));
    (void)tk_ena_dsp();
    return id;
}

LOCAL uint32_t timeout_for(float path_mm, float speed)
{
    float s = (speed < 0.0f) ? -speed : speed;

    if (s < 20.0f)
    {
        s = 20.0f;
    }
    return (uint32_t)((2000.0f * path_mm) / s) + 2000u;     /* 2x the ideal time + 2 s */
}

uint16_t motion_start_move(float dist_mm, float speed_mm_s, cmd_src_t src)
{
    motion_cmd_t c = { 0 };

    c.type       = MC_BYPASS;
    c.src        = (uint8_t)src;
    c.id         = new_id();
    c.dist_mm    = dist_mm;
    c.speed_mm_s = (speed_mm_s < 0.0f) ? -speed_mm_s : speed_mm_s;
    c.timeout_ms = timeout_for((dist_mm < 0.0f) ? -dist_mm : dist_mm, c.speed_mm_s);
    flag_clear(EVF_MOTION_DONE);
    return post_motion(&c) ? c.id : 0u;
}

uint16_t motion_start_turn(float deg_cw, float speed_mm_s, cmd_src_t src)
{
    motion_cmd_t c = { 0 };
    float        arc = ((deg_cw < 0.0f) ? -deg_cw : deg_cw) / RAD_TO_DEG * 0.5f * g_params.track_width_mm;

    c.type       = MC_TURN;
    c.src        = (uint8_t)src;
    c.id         = new_id();
    c.turn_deg   = deg_cw;
    c.speed_mm_s = (speed_mm_s < 0.0f) ? -speed_mm_s : speed_mm_s;
    c.timeout_ms = timeout_for(arc, c.speed_mm_s);
    flag_clear(EVF_MOTION_DONE);
    return post_motion(&c) ? c.id : 0u;
}

bool motion_wait(uint16_t id, uint32_t other_bits, uint32_t timeout_ms,
                 motion_result_t *p_result, uint32_t *p_other)
{
    uint32_t t0 = hal_time_ms();

    *p_other = 0u;
    if (id == 0u)
    {
        *p_result = MOTION_STOPPED;
        return true;                      /* could not even be posted */
    }
    for (;;)
    {
        UINT     ptn = 0;
        uint32_t left = timeout_ms - (hal_time_ms() - t0);
        ER       er;

        if ((hal_time_ms() - t0) >= timeout_ms)
        {
            return false;
        }
        er = tk_wai_flg(g_flg_sys, (UINT)(EVF_MOTION_DONE | other_bits), TWF_ORW, &ptn, (TMO)left);
        if (er < E_OK)
        {
            return false;
        }
        if (((uint32_t)ptn & other_bits) != 0u)
        {
            *p_other = (uint32_t)ptn & other_bits;
            return false;
        }
        flag_clear(EVF_MOTION_DONE);
        if (g_done_id == id)
        {
            *p_result = (motion_result_t)g_done_result;
            return true;
        }
    }
}

motion_result_t motion_move(float dist_mm, float speed_mm_s)
{
    uint16_t        id = motion_start_move(dist_mm, speed_mm_s, SRC_NAV);
    motion_result_t res = MOTION_TIMEOUT;
    uint32_t        other;
    float           d = (dist_mm < 0.0f) ? -dist_mm : dist_mm;

    (void)motion_wait(id, 0u, timeout_for(d, speed_mm_s) + DONE_WAIT_MARGIN, &res, &other);
    return res;
}

motion_result_t motion_turn(float deg_cw, float speed_mm_s)
{
    uint16_t        id = motion_start_turn(deg_cw, speed_mm_s, SRC_NAV);
    motion_result_t res = MOTION_TIMEOUT;
    uint32_t        other;

    (void)motion_wait(id, 0u, 20000u, &res, &other);
    return res;
}

motion_result_t motion_move_forward(float dist_mm)  { return motion_move(dist_mm, g_params.bypass_mm_s); }
motion_result_t motion_move_backward(float dist_mm) { return motion_move(-dist_mm, g_params.bypass_mm_s); }
motion_result_t motion_turn_left(float deg)         { return motion_turn(-deg, g_params.turn_mm_s); }
motion_result_t motion_turn_right(float deg)        { return motion_turn(deg, g_params.turn_mm_s); }

void motion_stop(cmd_src_t src)
{
    motion_cmd_t c = { 0 };

    c.type = MC_STOP;
    c.src  = (uint8_t)src;
    (void)post_motion(&c);
}

void motion_speed_limit(float scale)
{
    motion_cmd_t c = { 0 };

    c.type  = MC_SPEED_LIMIT;
    c.src   = (uint8_t)SRC_NAV;
    c.scale = scale;
    (void)post_motion(&c);
}

bool motion_line(float left_mm_s, float right_mm_s)
{
    motion_cmd_t c = { 0 };

    c.type       = MC_LINE;
    c.src        = (uint8_t)SRC_LINE;
    c.left_mm_s  = left_mm_s;
    c.right_mm_s = right_mm_s;
    return post_motion(&c);
}

/* ------------------------------------------------------------------------ */
/* Task                                                                      */
/* ------------------------------------------------------------------------ */

LOCAL void finish(motion_result_t res)
{
    if (!g_prim.b_active)
    {
        return;
    }
    g_prim.b_active = false;
    g_done_id       = g_prim.id;
    g_done_result   = (uint32_t)res;
    g_w[0].target   = 0.0f;
    g_w[1].target   = 0.0f;
    flag_set(EVF_MOTION_DONE);
}

LOCAL float clamp_speed(float v)
{
    const float max = (float)SPEED_MAX_MM_S;

    return (v > max) ? max : ((v < -max) ? -max : v);
}

LOCAL void handle_cmd(const motion_cmd_t *p_c, int32_t enc_r, int32_t enc_l, uint32_t now)
{
    switch (p_c->type)
    {
        case MC_STOP:
            finish(MOTION_STOPPED);
            g_line_t      = 0u;
            g_line_l      = 0.0f;
            g_line_r      = 0.0f;
            g_w[0].target = 0.0f;
            g_w[1].target = 0.0f;
            break;

        case MC_TURN:
        case MC_BYPASS:
            finish(MOTION_STOPPED);            /* a new primitive replaces the old one */
            g_prim.b_active   = true;
            g_prim.type       = p_c->type;
            g_prim.id         = p_c->id;
            g_prim.start_r    = enc_r;
            g_prim.start_l    = enc_l;
            g_prim.t_start    = now;
            g_prim.timeout_ms = p_c->timeout_ms;
            if (p_c->type == MC_TURN)
            {
                float a = (p_c->turn_deg < 0.0f) ? -p_c->turn_deg : p_c->turn_deg;

                g_prim.target_mm = (a / RAD_TO_DEG) * 0.5f * g_params.track_width_mm;
                g_prim.dir       = (p_c->turn_deg < 0.0f) ? -1.0f : 1.0f;
                g_prim.speed     = p_c->speed_mm_s;
            }
            else
            {
                g_prim.target_mm = (p_c->dist_mm < 0.0f) ? -p_c->dist_mm : p_c->dist_mm;
                g_prim.dir       = 1.0f;
                g_prim.speed     = (p_c->dist_mm < 0.0f) ? -p_c->speed_mm_s : p_c->speed_mm_s;
            }
            break;

        case MC_LINE:
            if (!g_prim.b_active)
            {
                g_line_l = p_c->left_mm_s;
                g_line_r = p_c->right_mm_s;
                g_line_t = now;
            }
            break;

        case MC_SPEED_LIMIT:
            g_scale = (p_c->scale < 0.1f) ? 0.1f : ((p_c->scale > 1.0f) ? 1.0f : p_c->scale);
            break;

        default:
            break;
    }
}

LOCAL void run_primitive(int32_t enc_r, int32_t enc_l, float mmpc, uint32_t now)
{
    float dr = (float)(enc_r - g_prim.start_r) * mmpc;
    float dl = (float)(enc_l - g_prim.start_l) * mmpc;
    float adr = (dr < 0.0f) ? -dr : dr;
    float adl = (dl < 0.0f) ? -dl : dl;
    float done = 0.5f * (adr + adl);
    float remaining = g_prim.target_mm - done;
    float v = g_prim.speed;

    if ((now - g_prim.t_start) > g_prim.timeout_ms)
    {
        fault_set(FAULT_MOTION_TIMEOUT);
        finish(MOTION_TIMEOUT);
        flag_set(EVF_ERROR_REQ);              /* DR: timeout = encoder fault -> ERROR */
        return;
    }
    if (remaining <= 0.0f)
    {
        finish(MOTION_OK);
        return;
    }
    if (remaining < 25.0f)
    {
        v *= 0.5f;                            /* slow down for the last 25 mm */
        if ((v > 0.0f) && (v < 40.0f))
        {
            v = 40.0f;
        }
        if ((v < 0.0f) && (v > -40.0f))
        {
            v = -40.0f;
        }
    }
    if (g_prim.type == MC_TURN)
    {
        g_w[1].target = g_prim.dir * v;      /* clockwise: left forward ...  */
        g_w[0].target = -g_prim.dir * v;     /* ... right backward           */
    }
    else
    {
        /* straight-line correction: keep both wheels' distances equal */
        float corr = 2.0f * (adl - adr);     /* mm/s per mm of difference */

        corr = (corr > 30.0f) ? 30.0f : ((corr < -30.0f) ? -30.0f : corr);
        if (v >= 0.0f)
        {
            g_w[1].target = v - corr;
            g_w[0].target = v + corr;
        }
        else
        {
            g_w[1].target = v + corr;
            g_w[0].target = v - corr;
        }
    }
}

void tsk_motion(INT stacd, void *exinf)
{
    speed_params_t sp;
    odometry_t     odo = { 0 };
    int32_t        prev_r = hal_enc_count(HAL_WHEEL_RIGHT);
    int32_t        prev_l = hal_enc_count(HAL_WHEEL_LEFT);
    uint64_t       t_prev = hal_time_us();
    float          v_prev = 0.0f;
    float          a_filt = 0.0f;
    uint32_t       no_tick_ms[2] = { 0u, 0u };

    (void)stacd;
    (void)exinf;
    wheel_reset(&g_w[0]);
    wheel_reset(&g_w[1]);
    g_prim.b_active = false;

    for (;;)
    {
        app_msg_t *p_msg;
        uint64_t   t_now;
        float      dt;
        int32_t    enc_r;
        int32_t    enc_l;
        int32_t    d_r;
        int32_t    d_l;
        float      mmpc = params_mm_per_count();
        uint32_t   now;
        bool       b_stop;

        (void)tk_slp_tsk(TMO_FEVR);          /* released every 20 ms by the cyclic tick */
        g_hb[HB_MOTION]++;

        t_now  = hal_time_us();
        dt     = (float)(t_now - t_prev) / 1.0e6f;
        t_prev = t_now;
        if ((dt < 0.005f) || (dt > 0.2f))
        {
            dt = 0.02f;
        }
        now   = (uint32_t)(t_now / 1000u);
        enc_r = hal_enc_count(HAL_WHEEL_RIGHT);
        enc_l = hal_enc_count(HAL_WHEEL_LEFT);
        d_r   = enc_r - prev_r;
        d_l   = enc_l - prev_l;
        prev_r = enc_r;
        prev_l = enc_l;

        /* 1. odometry */
        odo.enc_r      = enc_r;
        odo.enc_l      = enc_l;
        odo.dist_r_mm += (float)d_r * mmpc;
        odo.dist_l_mm += (float)d_l * mmpc;
        if (g_params.track_width_mm > 0.0f)
        {
            float dh = (((float)(d_l - d_r) * mmpc) / g_params.track_width_mm) * RAD_TO_DEG;

            odo.heading_deg += dh;
            odo.turn_dps     = dh / dt;
        }
        odo.v_mm_s  = 0.5f * (g_w[0].speed + g_w[1].speed);
        a_filt      = (0.7f * a_filt) + (0.3f * (((odo.v_mm_s - v_prev) / dt) / G_MM_S2));
        v_prev      = odo.v_mm_s;
        odo.a_fwd_g = a_filt;
        odo.odo_um  = (uint32_t)(((float)hal_odometer() * mmpc * 1000.0f) * 0.5f);
        odometry_put(&odo);

        /* 2. commands */
        while ((p_msg = msg_recv(g_mbx_motion, TMO_POL)) != NULL)
        {
            handle_cmd(&p_msg->body.motion, enc_r, enc_l, now);
            msg_free(p_msg);
        }

        /* 3. what to drive */
        b_stop = ((flag_peek() & EVF_ESTOP) != 0u) || hal_motor_killed() || (mmpc <= 0.0f);
        if (b_stop)
        {
            finish(MOTION_STOPPED);
            g_line_t = 0u;
            wheel_reset(&g_w[0]);
            wheel_reset(&g_w[1]);
            hal_motor_set(HAL_MOTOR_RIGHT, 0.0f);
            hal_motor_set(HAL_MOTOR_LEFT, 0.0f);
        }
        else
        {
            if (g_prim.b_active)
            {
                run_primitive(enc_r, enc_l, mmpc, now);
            }
            else if ((g_line_t != 0u) && ((now - g_line_t) <= LINE_STALE_MS))
            {
                g_w[1].target = clamp_speed(g_line_l * g_scale);
                g_w[0].target = clamp_speed(g_line_r * g_scale);
            }
            else
            {
                g_w[0].target = 0.0f;
                g_w[1].target = 0.0f;
            }

            /* 4. speed control */
            sp.kp              = g_params.speed_kp;
            sp.ki              = g_params.speed_ki;
            sp.ff_deadband_pct = g_params.ff_deadband_pct;
            sp.ff_mm_s_per_pct = g_params.ff_mm_s_per_pct;
            sp.max_duty        = 1.0f;
            hal_motor_set(HAL_MOTOR_RIGHT, wheel_update(&g_w[0], &sp, d_r, mmpc, dt));
            hal_motor_set(HAL_MOTOR_LEFT, wheel_update(&g_w[1], &sp, d_l, mmpc, dt));

            /* encoder loss: driven hard but not one tick (DR Section 5) */
            for (uint32_t w = 0u; w < 2u; w++)
            {
                float   duty = (g_w[w].duty < 0.0f) ? -g_w[w].duty : g_w[w].duty;
                int32_t d = (w == 0u) ? d_r : d_l;

                no_tick_ms[w] = ((duty > ENC_LOSS_DUTY) && (d == 0)) ? (no_tick_ms[w] + 20u) : 0u;
                if (no_tick_ms[w] >= ENC_LOSS_MS)
                {
                    no_tick_ms[w] = 0u;
                    fault_set(FAULT_ENCODER_LOSS);
                    flag_set(EVF_ERROR_REQ);
                }
            }
        }

        /* telemetry snapshot */
        g_status.spd_r       = g_w[0].speed;
        g_status.spd_l       = g_w[1].speed;
        g_status.duty_r      = g_w[0].duty;
        g_status.duty_l      = g_w[1].duty;
        g_status.enc_r       = enc_r;
        g_status.enc_l       = enc_l;
        g_status.dist_mm     = (float)odo.odo_um / 1000.0f;
        g_status.speed_scale = g_scale;
    }
}
