# Inter_Task_Message sizes (target)

| Record | Value |
|---|---|
| Headers | `app_program/` at commit `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` |
| Build | Baseline_Build (no msg_codec.h) |
| Compiler (`arm-none-eabi-gcc --version`) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Flags | `-mcpu=cortex-m0plus -mthumb -std=gnu11 -O2 -ffreestanding -fno-common -DAPP_LEGACY_TEXT_CMD=0 -DAPP_PERF=0 -DAPP_SELFTEST=0 -I /tmp/type_sizes.jXAHoR/app_program` |
| Size source | `arm-none-eabi-nm -S` on `type_sizes.o` |

## Message body types (Requirement 23.1)

| `app_msg_body_t` member | Body type | sizeof (bytes) |
|---|---|---|
| `motion` | `motion_cmd_t` | 32 |
| `nav` | `nav_event_t` | 2 |
| `obstacle` | `obstacle_profile_t` | 64 |
| `event` | `tlm_event_t` | 92 |
| (union) | `app_msg_body_t` | 92 |

## Message_Pool block

| Item | Bytes |
|---|---|
| `T_MSG` header | 4 |
| Block size: `(sizeof(T_MSG) + sizeof(app_msg_body_t) + 3) / 4 * 4` (`MSG_BLK_WORDS * 4` in os_glue.c) | 96 |
| `g_mpf_buf` in `is02_robot_smp0.elf` (`arm-none-eabi-nm -S`) | 4000 |
| MSG_BLOCKS | 40 |
| Block size from the ELF: `g_mpf_buf` bytes / MSG_BLOCKS - 4 (buffer is MSG_BLOCKS x (MSG_BLK_WORDS + 1) words) | 96 |
