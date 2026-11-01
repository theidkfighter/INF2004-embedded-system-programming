/**
 * @file    wire_dump.c
 * @brief   Prints the C side of the binary protocol as JSON (spec task 3.11):
 *          Format_Version, every code table of wire_ids.h, the parameter
 *          table (D15) and every frame layout (field, offset, width, kind,
 *          resolution, sentinel). The laptop tests compare the Python and
 *          JavaScript tables with this output (INTEGRATION.md 1.4).
 *
 *   make -C tests/host tools && build/wire_dump > build/wire_dump.json
 *
 * The frame layouts below are checked against the encoder output by
 * tests/host/py/test_wire_tools.py, so they cannot drift from wire.c.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "app_params.h"
#include "wire.h"
#include "wire_ids.h"

typedef struct
{
    const char *name;
    unsigned    off;
    unsigned    width;
    const char *kind;      /* U, S, E, F, M, F32, PAD */
    const char *res;       /* resolution as text, "" for codes */
    long        sentinel;  /* -1 = none */
} field_t;

static const field_t TLM[] =
{
    { "version", 0, 4, "U", "", -1 }, { "seq", 4, 15, "M", "", -1 }, { "t", 19, 24, "M", "1 ms", -1 },
    { "state", 43, 3, "E", "", -1 }, { "spd_l", 46, 15, "S", "0.1 mm/s", -1 },
    { "spd_r", 61, 15, "S", "0.1 mm/s", -1 }, { "duty_l", 76, 11, "S", "0.1 %", -1 },
    { "duty_r", 87, 11, "S", "0.1 %", -1 }, { "enc_l", 98, 24, "S", "1 count", -1 },
    { "enc_r", 122, 24, "S", "1 count", -1 }, { "dist", 146, 20, "U", "0.1 mm", -1 },
    { "line_l", 166, 2, "S", "", -1 }, { "line_r", 168, 2, "S", "", -1 }, { "ir", 170, 12, "U", "1", -1 },
    { "ir_min", 182, 12, "U", "1", -1 }, { "ir_max", 194, 12, "U", "1", -1 },
    { "thr_valid", 206, 1, "F", "", -1 }, { "thr_black", 207, 12, "U", "1", -1 },
    { "thr_white", 219, 12, "U", "1", -1 }, { "bc", 231, 6, "U", "", 63 },
    { "pitch", 237, 11, "S", "0.1 deg", -1 }, { "hump_h", 248, 15, "S", "0.1 mm", -1 },
    { "hump_peak", 263, 14, "U", "0.1 mm", -1 }, { "hump_max", 277, 14, "U", "0.1 mm", -1 },
    { "us", 291, 13, "U", "0.1 cm", 8191 }, { "scan0", 304, 13, "U", "0.1 cm", 8191 },
    { "scan1", 317, 13, "U", "0.1 cm", 8191 }, { "scan2", 330, 13, "U", "0.1 cm", 8191 },
    { "scan3", 343, 13, "U", "0.1 cm", 8191 }, { "scan4", 356, 13, "U", "0.1 cm", 8191 },
    { "side", 369, 2, "E", "", -1 }, { "imu_ev", 371, 3, "E", "", -1 },
    { "turn", 374, 14, "S", "0.1 deg/s", -1 }, { "faults", 388, 16, "F", "", -1 },
    { "vbat", 404, 13, "U", "1 mV", -1 }, { "rssi", 417, 8, "U", "-1 dBm", -1 },
    { "buf", 425, 7, "U", "1", -1 },
};

static const field_t HB[] =
{
    { "version", 0, 4, "U", "", -1 }, { "seq", 4, 14, "M", "", -1 }, { "t", 18, 24, "M", "1 ms", -1 },
    { "state", 42, 3, "E", "", -1 }, { "rssi", 45, 8, "U", "-1 dBm", -1 }, { "buf", 53, 7, "U", "1", -1 },
    { "uptime", 60, 17, "U", "1 s", -1 }, { "reconnects", 77, 16, "U", "1", -1 },
    { "dropped", 93, 16, "U", "1", -1 }, { "sat", 109, 16, "U", "1", -1 },
    { "faults", 125, 16, "F", "", -1 }, { "n_params", 141, 6, "U", "1", -1 },
    { "param_crc", 147, 16, "U", "", -1 }, { "ip0", 163, 8, "U", "", -1 }, { "ip1", 171, 8, "U", "", -1 },
    { "ip2", 179, 8, "U", "", -1 }, { "ip3", 187, 8, "U", "", -1 }, { "padding", 195, 5, "PAD", "", -1 },
};

static const field_t EVT_HDR[] =
{
    { "version", 0, 4, "U", "", -1 }, { "t", 4, 24, "M", "1 ms", -1 }, { "code", 28, 6, "E", "", -1 },
};

static const field_t ACK_HDR[] =
{
    { "version", 0, 4, "U", "", -1 }, { "t", 4, 24, "M", "1 ms", -1 }, { "cmd", 28, 4, "E", "", -1 },
    { "reason", 32, 5, "E", "", -1 },
};

static const field_t CMD_HDR[] = { { "version", 0, 4, "U", "", -1 }, { "cmd", 4, 4, "E", "", -1 } };

static const field_t MAP[] =
{
    { "version", 0, 4, "U", "", -1 },
    { "fwd_present", 4, 1, "F", "", -1 }, { "fwd_letter", 5, 5, "U", "", -1 },
    { "left_present", 10, 1, "F", "", -1 }, { "left_letter", 11, 5, "U", "", -1 },
    { "right_present", 16, 1, "F", "", -1 }, { "right_letter", 17, 5, "U", "", -1 },
    { "uturn_present", 22, 1, "F", "", -1 }, { "uturn_letter", 23, 5, "U", "", -1 },
    { "padding", 28, 4, "PAD", "", -1 },
};

static const field_t CFG[] =
{
    { "version", 0, 4, "U", "", -1 }, { "count", 4, 6, "U", "", -1 },
    { "pair_id", 10, 6, "U", "", -1 }, { "pair_value", 16, 32, "F32", "", -1 },
};

static void str(const char *s)
{
    putchar('"');
    for (; *s != '\0'; s++)
    {
        if ((*s == '"') || (*s == '\\'))
        {
            putchar('\\');
        }
        putchar(*s);
    }
    putchar('"');
}

static void layout(const char *name, const field_t *f, unsigned n, unsigned bits, unsigned len, int last)
{
    printf("    ");
    str(name);
    printf(": {\"bits\": %u, \"len\": %u, \"fields\": [\n", bits, len);
    for (unsigned i = 0; i < n; i++)
    {
        printf("      {\"name\": ");
        str(f[i].name);
        printf(", \"offset\": %u, \"width\": %u, \"kind\": ", f[i].off, f[i].width);
        str(f[i].kind);
        printf(", \"resolution\": ");
        str(f[i].res);
        if (f[i].sentinel >= 0)
        {
            printf(", \"sentinel\": %ld", f[i].sentinel);
        }
        printf("}%s\n", (i + 1 < n) ? "," : "");
    }
    printf("    ]}%s\n", last ? "" : ",");
}

int main(void)
{
    int first;

    params_init();
    printf("{\n  \"format_version\": %u,\n", WIRE_FORMAT_VERSION);

    printf("  \"events\": [\n");
    first = 1;
#define X(code, NAME, bits, var) \
    printf("%s    {\"code\": %u, \"name\": \"%s\", \"arg_bits\": %u, \"variable\": %s}", first ? "" : ",\n", \
           (unsigned)(code), #NAME, (unsigned)(bits), (var) ? "true" : "false"); first = 0;
    WIRE_EVENT_TABLE(X)
#undef X
    printf("\n  ],\n  \"reasons\": [\n");
    first = 1;
#define X(code, NAME, text) \
    printf("%s    {\"code\": %u, \"name\": \"%s\", \"text\": ", first ? "" : ",\n", (unsigned)(code), #NAME); \
    str(text); printf("}"); first = 0;
    WIRE_REASON_TABLE(X)
#undef X
    printf("\n  ],\n  \"commands\": [\n");
    first = 1;
#define X(code, NAME, echo, clen, alen) \
    printf("%s    {\"code\": %u, \"name\": \"%s\", \"echo\": \"%s\", \"cmd_len\": %u, \"ack_len\": %u}", \
           first ? "" : ",\n", (unsigned)(code), #NAME, echo, (unsigned)(clen), (unsigned)(alen)); first = 0;
    WIRE_CMD_TABLE(X)
#undef X
    printf("\n  ],\n  \"stop_reasons\": [\n");
    first = 1;
#define X(code, NAME, text) \
    printf("%s    {\"code\": %u, \"name\": \"%s\", \"text\": ", first ? "" : ",\n", (unsigned)(code), #NAME); \
    str(text); printf("}"); first = 0;
    WIRE_STOP_TABLE(X)
#undef X
    printf("\n  ],\n  \"abort_reasons\": [\n");
    first = 1;
#define X(code, NAME, text) \
    printf("%s    {\"code\": %u, \"name\": \"%s\", \"text\": ", first ? "" : ",\n", (unsigned)(code), #NAME); \
    str(text); printf("}"); first = 0;
    WIRE_ABORT_TABLE(X)
#undef X
    printf("\n  ],\n  \"params\": [\n");
    for (uint32_t i = 0u; i < params_count(); i++)
    {
        printf("    {\"id\": %u, \"name\": ", (unsigned)i);
        str(params_name(i));
        printf(", \"store_width\": %u}%s\n", (unsigned)params_store_width(i), (i + 1u < params_count()) ? "," : "");
    }
    printf("  ],\n  \"param_table_crc\": %u,\n", (unsigned)params_table_crc());

    printf("  \"frames\": {\n");
    layout("telemetry", TLM, sizeof(TLM) / sizeof(TLM[0]), WIRE_TLM_BITS, WIRE_TLM_LEN, 0);
    layout("heartbeat", HB, sizeof(HB) / sizeof(HB[0]), WIRE_HB_BITS, WIRE_HB_LEN, 0);
    layout("event_header", EVT_HDR, 3, WIRE_EVT_HDR_BITS, 0, 0);
    layout("ack_header", ACK_HDR, 4, 37, 0, 0);
    layout("command_header", CMD_HDR, 2, 8, 1, 0);
    layout("map", MAP, sizeof(MAP) / sizeof(MAP[0]), WIRE_MAP_BITS, WIRE_MAP_LEN, 0);
    layout("config", CFG, 4, WIRE_CFG_HDR_BITS, 0, 1);
    printf("  }\n}\n");
    return 0;
}
