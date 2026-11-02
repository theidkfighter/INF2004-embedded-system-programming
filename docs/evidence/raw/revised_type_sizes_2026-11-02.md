# Inter_Task_Message sizes (target)

| Record | Value |
|---|---|
| Headers | `app_program/` at commit `36a380bc3d8daf0f945e6c6853db1ab53914633e` |
| Build | Revised_Build (msg_codec.h present) |
| Compiler (`arm-none-eabi-gcc --version`) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Flags | `-mcpu=cortex-m0plus -mthumb -std=gnu11 -O2 -ffreestanding -fno-common -DAPP_LEGACY_TEXT_CMD=0 -DAPP_PERF=0 -DAPP_SELFTEST=0 -I /tmp/type_sizes.1jC12H/app_program` |
| Size source | `arm-none-eabi-nm -S` on `type_sizes.o` |

## Message body types (Requirement 23.1)

| `app_msg_body_t` member | Body type | sizeof (bytes) |
|---|---|---|
| `motion` | `motion_msg_t` | 12 |
| `nav` | `nav_msg_t` | 1 |
| `obstacle` | `obst_msg_t` | 24 |
| `event` | `evt_msg_t` | 32 |
| (union) | `app_msg_body_t` | 32 |

## Message_Pool block

| Item | Bytes |
|---|---|
| `T_MSG` header | 4 |
| Block size: `(sizeof(T_MSG) + sizeof(app_msg_body_t) + 3) / 4 * 4` (`MSG_BLK_WORDS * 4` in os_glue.c) | 36 |
| `g_mpf_buf` in `is02_robot_smp0.elf` (`arm-none-eabi-nm -S`) | 1600 |
| MSG_BLOCKS | 40 |
| Block size from the ELF: `g_mpf_buf` bytes / MSG_BLOCKS - 4 (buffer is MSG_BLOCKS x (MSG_BLK_WORDS + 1) words) | 36 |
