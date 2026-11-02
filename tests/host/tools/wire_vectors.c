/**
 * @file    wire_vectors.c
 * @brief   Runs shared test vectors through the C Wire_Codec (spec task 14.5,
 *          C half). Reads one vector per line on stdin, writes one JSON
 *          object per line on stdout. The Python and JavaScript codecs run
 *          the same lines and the laptop test (Property 28) compares.
 *
 * Input lines (numbers are decimal, hex strings have no prefix; '#' starts
 * a comment line):
 *   telemetry <36 field codes in design D8 order: seq t state spd_l spd_r
 *             duty_l duty_r enc_l enc_r dist line_l line_r ir ir_min ir_max
 *             thr_valid thr_b thr_w bc pitch hump_h hump_peak hump_max us
 *             scan0..scan4 side imu_ev turn faults vbat rssi buf>
 *   heartbeat <seq t state rssi buf uptime reconnects dropped sat faults
 *             n_params param_crc ip0 ip1 ip2 ip3>                  (D9 order)
 *   event     <t> <code> <arg_bits> <args hex, MSB first>             (D10)
 *   ack       <t> <cmd> <reason> <arguments by command, D11 order:
 *             ESTOP pwm_off_us | BARCODE letter | MAP/MAP_CLEAR/GET_MAP
 *             26 actions then n | PING number | SET id |
 *             GET_PARAMS part count then count x (id width code) |
 *             CONFIG applied n_rej then n_rej ids | others nothing>
 *   cmd       <code> [BARCODE letter | PING number | SET id value_bits |
 *             MAP p0 l0 p1 l1 p2 l2 p3 l3]                       (D12)
 *   map       <p0 l0 p1 l1 p2 l2 p3 l3>  (forward, left, right, U-turn; D13)
 *   config    <n> then n x (<id> <value_bits>)                        (D14)
 *   decode    <telemetry|heartbeat|event|ack|cmd|map|config> <hex>
 *
 * Output: encode lines give {"line": n, "kind": k, "hex": "..."} ("" when
 * the encoder refused the fields); decode lines give {"line": n, "kind": k,
 * "status": s, "fields": [...]} with the fields in the same order as the
 * matching encode line (status: wire_status_t, 0 = ok).
 *
 *   make -C tests/host tools && build/wire_vectors < vectors.txt
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_params.h"
#include "wire.h"
#include "wire_ids.h"

#define MAXF 400

static long  g_f[MAXF];
static int   g_n;

static void hex_out(const uint8_t *p, uint32_t n)
{
    putchar('"');
    for (uint32_t i = 0u; i < n; i++)
    {
        printf("%02x", p[i]);
    }
    putchar('"');
}

static uint32_t hex_in(const char *s, uint8_t *p, uint32_t cap)
{
    uint32_t n = 0u;

    while ((s[0] != '\0') && (s[1] != '\0') && (n < cap))
    {
        char b[3] = { s[0], s[1], '\0' };

        p[n++] = (uint8_t)strtoul(b, NULL, 16);
        s += 2;
    }
    return n;
}

static void fields_out(int status, const long *f, int n)
{
    printf(", \"status\": %d, \"fields\": [", status);
    for (int i = 0; i < n; i++)
    {
        printf("%s%ld", i ? ", " : "", f[i]);
    }
    printf("]}\n");
}

static uint32_t pbits(uint32_t id)
{
    return params_store_width(id);
}

/* ---- telemetry / heartbeat <-> field lists --------------------------------- */

static void tlm_from(wire_tlm_t *t, const long *f)
{
    memset(t, 0, sizeof(*t));
    t->seq = (uint16_t)f[0]; t->t_ms = (uint32_t)f[1]; t->state = (uint8_t)f[2];
    t->spd[0] = (int16_t)f[3]; t->spd[1] = (int16_t)f[4]; t->duty[0] = (int16_t)f[5]; t->duty[1] = (int16_t)f[6];
    t->enc[0] = (int32_t)f[7]; t->enc[1] = (int32_t)f[8]; t->dist = (uint32_t)f[9];
    t->line_l = (int8_t)f[10]; t->line_r = (int8_t)f[11]; t->ir = (uint16_t)f[12];
    t->ir_min = (uint16_t)f[13]; t->ir_max = (uint16_t)f[14]; t->thr_valid = (uint8_t)f[15];
    t->thr_b = (uint16_t)f[16]; t->thr_w = (uint16_t)f[17]; t->bc = (uint8_t)f[18];
    t->pitch = (int16_t)f[19]; t->hump_h = (int16_t)f[20]; t->hump_peak = (uint16_t)f[21];
    t->hump_max = (uint16_t)f[22]; t->us = (uint16_t)f[23];
    for (int i = 0; i < 5; i++) { t->scan[i] = (uint16_t)f[24 + i]; }
    t->side = (uint8_t)f[29]; t->imu_ev = (uint8_t)f[30]; t->turn = (int16_t)f[31];
    t->faults = (uint16_t)f[32]; t->vbat = (uint16_t)f[33]; t->rssi = (uint8_t)f[34]; t->buf = (uint8_t)f[35];
}

static int tlm_to(const wire_tlm_t *t, long *f)
{
    long v[36] = { t->seq, (long)t->t_ms, t->state, t->spd[0], t->spd[1], t->duty[0], t->duty[1],
                   t->enc[0], t->enc[1], (long)t->dist, t->line_l, t->line_r, t->ir, t->ir_min, t->ir_max,
                   t->thr_valid, t->thr_b, t->thr_w, t->bc, t->pitch, t->hump_h, t->hump_peak, t->hump_max,
                   t->us, t->scan[0], t->scan[1], t->scan[2], t->scan[3], t->scan[4], t->side, t->imu_ev,
                   t->turn, t->faults, t->vbat, t->rssi, t->buf };
    memcpy(f, v, sizeof(v));
    return 36;
}

static void hb_from(wire_hb_t *h, const long *f)
{
    memset(h, 0, sizeof(*h));
    h->seq = (uint16_t)f[0]; h->t_ms = (uint32_t)f[1]; h->state = (uint8_t)f[2]; h->rssi = (uint8_t)f[3];
    h->buf = (uint8_t)f[4]; h->uptime_s = (uint32_t)f[5]; h->reconnects = (uint16_t)f[6];
    h->dropped = (uint16_t)f[7]; h->sat = (uint16_t)f[8]; h->faults = (uint16_t)f[9];
    h->n_params = (uint8_t)f[10]; h->param_crc = (uint16_t)f[11];
    for (int i = 0; i < 4; i++) { h->ip[i] = (uint8_t)f[12 + i]; }
}

static int hb_to(const wire_hb_t *h, long *f)
{
    long v[16] = { h->seq, (long)h->t_ms, h->state, h->rssi, h->buf, (long)h->uptime_s, h->reconnects,
                   h->dropped, h->sat, h->faults, h->n_params, h->param_crc, h->ip[0], h->ip[1], h->ip[2],
                   h->ip[3] };
    memcpy(f, v, sizeof(v));
    return 16;
}

/* ---- ack <-> field list --------------------------------------------------- */

static int ack_from(wire_ack_t *a, const long *f, int n)
{
    int k = 3;

    memset(a, 0, sizeof(*a));
    a->t_ms = (uint32_t)f[0]; a->cmd = (uint8_t)f[1]; a->reason = (uint8_t)f[2];
    switch (a->cmd)
    {
        case WCMD_ESTOP: a->a.pwm_off_us = (uint16_t)f[k++]; break;
        case WCMD_BARCODE: a->a.letter = (uint8_t)f[k++]; break;
        case WCMD_MAP: case WCMD_MAP_CLEAR: case WCMD_GET_MAP:
            for (int i = 0; i < 26; i++) { a->a.map.act[i] = (uint8_t)f[k++]; }
            a->a.map.n = (uint8_t)f[k++];
            break;
        case WCMD_PING: a->a.ping = (uint16_t)f[k++]; break;
        case WCMD_SET: a->a.param_id = (uint8_t)f[k++]; break;
        case WCMD_GET_PARAMS:
            a->a.getp.part = (uint8_t)f[k++];
            a->a.getp.count = (uint8_t)f[k++];
            for (int i = 0; (i < a->a.getp.count) && (i < (int)WIRE_GETP_PER_PART); i++)
            {
                a->a.getp.v[i].id = (uint8_t)f[k++];
                a->a.getp.v[i].w = (uint8_t)f[k++];
                a->a.getp.v[i].code = (uint32_t)f[k++];
            }
            break;
        case WCMD_CONFIG:
            a->a.cfg.applied = (uint8_t)f[k++];
            a->a.cfg.n_rej = (uint8_t)f[k++];
            for (int i = 0; (i < a->a.cfg.n_rej) && (i < (int)WIRE_CFG_REJ_MAX); i++)
            {
                a->a.cfg.rej[i] = (uint8_t)f[k++];
            }
            break;
        default: break;
    }
    return (k <= n);
}

static int ack_to(const wire_ack_t *a, long *f)
{
    int k = 0;

    f[k++] = (long)a->t_ms; f[k++] = a->cmd; f[k++] = a->reason;
    switch (a->cmd)
    {
        case WCMD_ESTOP: f[k++] = a->a.pwm_off_us; break;
        case WCMD_BARCODE: f[k++] = a->a.letter; break;
        case WCMD_MAP: case WCMD_MAP_CLEAR: case WCMD_GET_MAP:
            for (int i = 0; i < 26; i++) { f[k++] = a->a.map.act[i]; }
            f[k++] = a->a.map.n;
            break;
        case WCMD_PING: f[k++] = a->a.ping; break;
        case WCMD_SET: f[k++] = a->a.param_id; break;
        case WCMD_GET_PARAMS:
            f[k++] = a->a.getp.part; f[k++] = a->a.getp.count;
            for (int i = 0; i < a->a.getp.count; i++)
            {
                f[k++] = a->a.getp.v[i].id; f[k++] = a->a.getp.v[i].w; f[k++] = (long)a->a.getp.v[i].code;
            }
            break;
        case WCMD_CONFIG:
            f[k++] = a->a.cfg.applied; f[k++] = a->a.cfg.n_rej;
            for (int i = 0; i < a->a.cfg.n_rej; i++) { f[k++] = a->a.cfg.rej[i]; }
            break;
        default: break;
    }
    return k;
}

/* ---- command <-> field list ---------------------------------------------- */

static void cmd_from(wire_cmd_t *c, const long *f, int n)
{
    memset(c, 0, sizeof(*c));
    c->cmd = (uint8_t)f[0];
    if ((c->cmd == WCMD_BARCODE) && (n > 1)) { c->letter = (uint8_t)f[1]; }
    if ((c->cmd == WCMD_PING) && (n > 1)) { c->ping = (uint16_t)f[1]; }
    if ((c->cmd == WCMD_SET) && (n > 2)) { c->param_id = (uint8_t)f[1]; c->value_bits = (uint32_t)f[2]; }
    if ((c->cmd == WCMD_MAP) && (n > 8))
    {
        for (int i = 0; i < 4; i++) { c->map_present[i] = (uint8_t)f[1 + 2 * i]; c->map_letter[i] = (uint8_t)f[2 + 2 * i]; }
    }
}

static int cmd_to(const wire_cmd_t *c, long *f, int b_map_only)
{
    int k = 0;

    if (!b_map_only)
    {
        f[k++] = c->cmd;
        if (c->cmd == WCMD_BARCODE) { f[k++] = c->letter; }
        if (c->cmd == WCMD_PING) { f[k++] = c->ping; }
        if (c->cmd == WCMD_SET) { f[k++] = c->param_id; f[k++] = (long)c->value_bits; }
    }
    if (b_map_only || (c->cmd == WCMD_MAP))
    {
        for (int i = 0; i < 4; i++) { f[k++] = c->map_present[i]; f[k++] = c->map_letter[i]; }
    }
    return k;
}

typedef struct { long f[1 + 2 * WIRE_PARAMS_MAX]; int n; } cfg_ctx_t;

static void cfg_cb(uint32_t id, uint32_t bits, void *p)
{
    cfg_ctx_t *c = (cfg_ctx_t *)p;

    c->f[c->n++] = (long)id;
    c->f[c->n++] = (long)bits;
}

/* ---- main ------------------------------------------------------------------ */

int main(void)
{
    char    line[8192];
    int     ln = 0;
    uint8_t out[300];
    long    f[MAXF];

    params_init();
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        char *kind;
        char *tok;

        ln++;
        kind = strtok(line, " \t\r\n");
        if ((kind == NULL) || (kind[0] == '#'))
        {
            continue;
        }
        printf("{\"line\": %d, \"kind\": \"%s\"", ln, kind);
        if (strcmp(kind, "decode") == 0)
        {
            char    *what = strtok(NULL, " \t\r\n");
            char    *hex = strtok(NULL, " \t\r\n");
            uint8_t  in[300];
            uint32_t len = (hex != NULL) ? hex_in(hex, in, sizeof(in)) : 0u;
            int      st = WIRE_BAD_VALUE;
            int      n = 0;

            what = (what != NULL) ? what : "";
            printf(", \"of\": \"%s\"", what);
            if (strcmp(what, "telemetry") == 0)
            {
                wire_tlm_t t;
                st = wire_dec_telemetry(in, len, &t);
                n = (st == WIRE_OK) ? tlm_to(&t, f) : 0;
            }
            else if (strcmp(what, "heartbeat") == 0)
            {
                wire_hb_t h;
                st = wire_dec_heartbeat(in, len, &h);
                n = (st == WIRE_OK) ? hb_to(&h, f) : 0;
            }
            else if (strcmp(what, "event") == 0)
            {
                wire_evt_t e;
                uint32_t   t;
                st = wire_dec_event(in, len, &t, &e);
                if (st == WIRE_OK)
                {
                    f[n++] = (long)t; f[n++] = e.code; f[n++] = e.nbits;
                    printf(", \"args\": ");
                    hex_out(e.args, (e.nbits + 7u) / 8u);
                }
            }
            else if (strcmp(what, "ack") == 0)
            {
                wire_ack_t a;
                st = wire_dec_ack(in, len, pbits, &a);
                n = (st == WIRE_OK) ? ack_to(&a, f) : 0;
            }
            else if (strcmp(what, "cmd") == 0)
            {
                wire_cmd_t c;
                st = wire_dec_cmd(in, len, &c);
                n = (st == WIRE_OK) ? cmd_to(&c, f, 0) : 0;
            }
            else if (strcmp(what, "map") == 0)
            {
                wire_cmd_t c;
                st = wire_dec_map(in, len, &c);
                n = (st == WIRE_OK) ? cmd_to(&c, f, 1) : 0;
            }
            else if (strcmp(what, "config") == 0)
            {
                cfg_ctx_t c;
                c.n = 1;
                st = wire_dec_config(in, len, cfg_cb, &c);
                c.f[0] = (c.n - 1) / 2;
                n = (st == WIRE_OK) ? c.n : 0;
                memcpy(f, c.f, sizeof(long) * (size_t)n);
            }
            fields_out(st, f, n);
            continue;
        }
        g_n = 0;
        if (strcmp(kind, "event") == 0)
        {
            char      *t = strtok(NULL, " \t\r\n");
            char      *c = strtok(NULL, " \t\r\n");
            char      *b = strtok(NULL, " \t\r\n");
            char      *h = strtok(NULL, " \t\r\n");
            wire_evt_t e;
            uint32_t   n = 0u;

            memset(&e, 0, sizeof(e));
            if ((t != NULL) && (c != NULL) && (b != NULL))
            {
                e.code = (uint8_t)strtoul(c, NULL, 10);
                e.nbits = (uint8_t)strtoul(b, NULL, 10);
                if (h != NULL)
                {
                    (void)hex_in(h, e.args, sizeof(e.args));
                }
                n = wire_enc_event((uint32_t)strtoul(t, NULL, 10), &e, out, sizeof(out));
            }
            printf(", \"hex\": ");
            hex_out(out, n);
            printf("}\n");
            continue;
        }
        while (((tok = strtok(NULL, " \t\r\n")) != NULL) && (g_n < MAXF))
        {
            g_f[g_n++] = strtol(tok, NULL, 10);
        }
        {
            uint32_t n = 0u;

            if ((strcmp(kind, "telemetry") == 0) && (g_n == 36))
            {
                wire_tlm_t t;
                tlm_from(&t, g_f);
                n = wire_enc_telemetry(&t, out);
            }
            else if ((strcmp(kind, "heartbeat") == 0) && (g_n == 16))
            {
                wire_hb_t h;
                hb_from(&h, g_f);
                n = wire_enc_heartbeat(&h, out);
            }
            else if ((strcmp(kind, "ack") == 0) && (g_n >= 3))
            {
                wire_ack_t a;
                if (ack_from(&a, g_f, g_n))
                {
                    n = wire_enc_ack(&a, out, sizeof(out));
                }
            }
            else if ((strcmp(kind, "cmd") == 0) && (g_n >= 1))
            {
                wire_cmd_t c;
                cmd_from(&c, g_f, g_n);
                n = wire_enc_cmd(&c, out, sizeof(out));
            }
            else if ((strcmp(kind, "map") == 0) && (g_n == 8))
            {
                wire_cmd_t c;
                long       tmp[9];
                tmp[0] = WCMD_MAP;
                memcpy(&tmp[1], g_f, sizeof(long) * 8u);
                cmd_from(&c, tmp, 9);
                n = wire_enc_map(&c, out, sizeof(out));
            }
            else if ((strcmp(kind, "config") == 0) && (g_n >= 1) && (g_n == (1 + 2 * (int)g_f[0])) &&
                     (g_f[0] <= (long)WIRE_PARAMS_MAX))
            {
                uint8_t  ids[WIRE_PARAMS_MAX];
                uint32_t bits[WIRE_PARAMS_MAX];
                for (int i = 0; i < (int)g_f[0]; i++)
                {
                    ids[i] = (uint8_t)g_f[1 + 2 * i];
                    bits[i] = (uint32_t)g_f[2 + 2 * i];
                }
                n = wire_enc_config((uint32_t)g_f[0], ids, bits, out, sizeof(out));
            }
            printf(", \"hex\": ");
            hex_out(out, n);
            printf("}\n");
        }
    }
    return 0;
}
