# Static RAM report: is02_robot_smp0.elf

| Record | Value |
|---|---|
| ELF | `is02_robot_smp0.elf` |
| Map file | `is02_robot_smp0.map` |
| Git commit | `36a380bc3d8daf0f945e6c6853db1ab53914633e` |
| Working tree | clean |
| make command line | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` |
| Robot options (`.robot_opts`) | `legacy0-perf0-self0` |
| Compiler (`arm-none-eabi-gcc --version`) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Report date (UTC) | 2026-11-02T07:08:45Z |

## `arm-none-eabi-size` output (unedited)

```text
   text	   data	    bss	    dec	    hex	filename
 409668	   2788	  60160	 472616	  73628	is02_robot_smp0.elf
```

## `arm-none-eabi-size -A` output (unedited)

```text
is02_robot_smp0.elf  :
section                     size        addr
boot2                        256   268435456
.text                     408928   268435712
.time_critical.tinyusb       476   268844640
.ARM.exidx                     8   268845116
vector                       192   536870912
.data                       2788   536871104
.bss                       59968   536873892
.debug_info               723694           0
.debug_abbrev             123878           0
.debug_loclists           375969           0
.debug_aranges             15056           0
.debug_rnglists            47371           0
.debug_macro              275562           0
.debug_line               524214           0
.debug_str               1245890           0
.comment                      38           0
.ARM.attributes               42           0
.debug_frame               40168           0
.debug_line_str              383           0
Total                    3844881
```

## Static RAM (Requirement 22.1)

| Section | Bytes |
|---|---|
| .data | 2788 |
| .bss | 60160 |
| .data + .bss | 62948 |

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
| Parameter_Store (`g_params`) | `g_params` 160 | 160 | 160 | `g_params` in app_params.o. |
