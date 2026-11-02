# Static RAM report: is02_robot_smp0.elf

| Record | Value |
|---|---|
| ELF | `is02_robot_smp0.elf` |
| Map file | `is02_robot_smp0.map` |
| Git commit | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` |
| Working tree | clean |
| make command line | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` |
| Robot options (`.robot_opts`) | `not found` |
| Compiler (`arm-none-eabi-gcc --version`) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Report date (UTC) | 2026-11-02T07:08:45Z |

## `arm-none-eabi-size` output (unedited)

```text
   text	   data	    bss	    dec	    hex	filename
 398788	   2808	 107972	 509568	  7c680	is02_robot_smp0.elf
```

## `arm-none-eabi-size -A` output (unedited)

```text
is02_robot_smp0.elf  :
section                     size        addr
boot2                        256   268435456
.text                     398048   268435712
.time_critical.tinyusb       476   268833760
.ARM.exidx                     8   268834236
vector                       192   536870912
.data                       2808   536871104
.bss                      107780   536873912
.debug_info               629400           0
.debug_abbrev             113767           0
.debug_loclists           309600           0
.debug_aranges             13616           0
.debug_rnglists            37763           0
.debug_macro              270725           0
.debug_line               470830           0
.debug_str               1225707           0
.comment                      38           0
.ARM.attributes               42           0
.debug_frame               36416           0
.debug_line_str              384           0
Total                    3617856
```

## Static RAM (Requirement 22.1)

| Section | Bytes |
|---|---|
| .data | 2808 |
| .bss | 107972 |
| .data + .bss | 110780 |

## RAM items (Requirement 22.3)

Sizes in bytes. "nm -S" is the symbol size from `arm-none-eabi-nm -S`; "Map file"
is the size of the item's `.bss.SYM` / `.data.SYM` input section in the
linker map file.

| Item | Symbols (bytes each) | nm -S bytes | Map file bytes | Notes |
|---|---|---|---|---|
| Status_Snapshot (`g_status`) | `g_status` 140 | 140 | 140 | `g_status` in app_status.o. |
| Message_Pool buffer (`g_mpf_buf`) | `g_mpf_buf` 4000 | 4000 | 4000 | `g_mpf_buf` in os_glue.o. |
| Frame_Ring (`g_frames`) | `g_frames` 32896 | 32896 | 32896 | `g_frames` in app_net.o. |
| message slots (`g_msgs`) | `g_msgs` 6432 | 6432 | 6432 | `g_msgs` in app_net.o. |
| barcode edge ring (`g_bc_ring`) | `g_bc_ring` 768 | 768 | 768 | `g_bc_ring` in hal_isr.o. |
| raw sample ring (`g_raw`) | `g_raw_er` 1024 `g_raw_el` 1024 `g_raw_adc` 512 | 2560 | 2560 | `g_raw_er` in hal_isr.o. `g_raw_el` in hal_isr.o. `g_raw_adc` in hal_isr.o. |
| command slots (`g_cmds`) | `g_cmds` 768 | 768 | 768 | `g_cmds` in tsk_telemetry.o. |
| Parameter_Store (`g_params`) | `g_params` 220 | 220 | 220 | `g_params` in app_params.o. |
