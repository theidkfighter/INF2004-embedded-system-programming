# Static RAM report: is02_robot_smp0.elf

| Record | Value |
|---|---|
| ELF | `build_make/is02_robot_smp0.elf` |
| Map file | `build_make/is02_robot_smp0.map` |
| Git commit | `456beae1d7d0fef249cd2ab050807a54c5f29fe1` |
| Working tree | clean |
| make command line | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` |
| Robot options (`.robot_opts`) | `perf0-self0` |
| Compiler (`arm-none-eabi-gcc --version`) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Report date (UTC) | 2026-11-19T07:59:34Z |

## `arm-none-eabi-size` output (unedited)

```text
   text	   data	    bss	    dec	    hex	filename
 414908	    708	  60284	 475900	  742fc	build_make/is02_robot_smp0.elf
```

## `arm-none-eabi-size -A` output (unedited)

```text
build_make/is02_robot_smp0.elf  :
section                     size        addr
boot2                        256   268435456
.text                     414168   268435712
.time_critical.tinyusb       476   268849880
.ARM.exidx                     8   268850356
vector                       192   536870912
.data                        708   536871104
.bss                       60092   536871812
.debug_info               736114           0
.debug_abbrev             125615           0
.debug_loclists           383373           0
.debug_aranges             15328           0
.debug_rnglists            47836           0
.debug_macro              275849           0
.debug_line               532860           0
.debug_str               1246771           0
.comment                      38           0
.ARM.attributes               42           0
.debug_frame               40948           0
.debug_line_str              382           0
Total                    3881056
```

## Static RAM (Requirement 22.1)

| Section | Bytes |
|---|---|
| .data | 708 |
| .bss | 60284 |
| .data + .bss | 60992 |

## RAM items (Requirement 22.3)

Sizes in bytes. "nm -S" is the symbol size from `arm-none-eabi-nm -S`; "Map file"
is the size of the item's `.bss.SYM` / `.data.SYM` input section in the
linker map file.

| Item | Symbols (bytes each) | nm -S bytes | Map file bytes | Notes |
|---|---|---|---|---|
| Status_Snapshot (`g_status`) | `g_status` 60 | 60 | 60 | `g_status` in app_status.o. |
| Message_Pool buffer (`g_mpf_buf`) | `g_mpf_buf` 1600 | 1600 | 1600 | `g_mpf_buf` in os_glue.o. |
| Frame_Ring (`g_frames`) | `g_frames` 3520 | 3520 | 3520 | `g_frames` in app_net.o. |
| message slots (`g_msgs`) | `g_msgs` 528 | 528 | 528 | `g_msgs` in app_net.o. |
| barcode edge ring (`g_bc_ring`) | `g_bc_ring` 256 | 256 | 256 | `g_bc_ring` in hal_isr.o. |
| raw sample ring (`g_raw`) | `g_raw` 1024 | 1024 | 1024 | replacement symbols (Requirement 22.6). `g_raw` in hal_isr.o. |
| command slots (`g_cmds`) | `g_cmds` 192 | 192 | 192 | `g_cmds` in tsk_telemetry.o. |
| Parameter_Store (`g_params`) | `g_params` 176 | 176 | 176 | `g_params` in app_params.o. |
