# RAM per symbol: Baseline_Build bca7964 vs cr1/min-bit-continue

Generated with `arm-none-eabi-nm -S` on is02_robot_smp0.elf (same toolchain, same config). Only symbols whose size changed are listed.

| totals | text | data | bss | RAM (data+bss) |
|---|---|---|---|---|
| baseline | 398,752 | 2,808 | 107,972 | 110,780 |
| revised | 408,628 | 2,796 | 60,268 | 63,064 |
| change | +9,876 | -12 | -47,704 | -47,716 (-43.1 %) |

| symbol | baseline B | revised B | change |
|---|---|---|---|
| `g_frames` | 32,896 | 3,520 | -29,376 |
| `g_msgs` | 6,432 | 528 | -5,904 |
| `g_mpf_buf` | 4,000 | 1,600 | -2,400 |
| `g_bcm` | 3,732 | 1,848 | -1,884 |
| `g_raw_er` | 1,024 | 0 | -1,024 |
| `g_raw_el` | 1,024 | 0 | -1,024 |
| `g_buf` | 800 | 0 | -800 |
| `g_in` | 1,024 | 263 | -761 |
| `g_config` | 1,024 | 263 | -761 |
| `s_extra.3` | 680 | 0 | -680 |
| `s_params.4` | 600 | 0 | -600 |
| `g_cmds` | 768 | 192 | -576 |
| `g_raw_adc` | 512 | 0 | -512 |
| `g_bc_ring` | 768 | 256 | -512 |
| `rw.1` | 512 | 0 | -512 |
| `s_frame.2` | 512 | 54 | -458 |
| `g_tx_frame` | 512 | 54 | -458 |
| `g_bc` | 512 | 212 | -300 |
| `s_bad.5` | 200 | 0 | -200 |
| `rb.0` | 128 | 0 | -128 |
| `s_map_cmd.0` | 100 | 0 | -100 |
| `g_status` | 140 | 60 | -80 |
| `g_odo` | 36 | 4 | -32 |
| `g_echo_us` | 4 | 0 | -4 |
| `g_echo_rise_us` | 8 | 4 | -4 |
| `g_stop_reason` | 4 | 0 | -4 |
| `g_why` | 4 | 1 | -3 |
| `g_adc_max` | 4 | 2 | -2 |
| `g_bc_thr_white` | 4 | 2 | -2 |
| `g_adc_min` | 4 | 2 | -2 |
| `g_hb_seq` | 4 | 2 | -2 |
| `g_seq` | 4 | 2 | -2 |
| `g_bc_thr_black` | 4 | 2 | -2 |
| `g_adc_last` | 4 | 2 | -2 |
| `g_echo_done` | 1 | 0 | -1 |
| `g_raw_gap` | 0 | 1 | +1 |
| `g_sat_count` | 0 | 2 | +2 |
| `g_raw_ref_l` | 0 | 4 | +4 |
| `g_echo` | 0 | 4 | +4 |
| `g_raw_full_l` | 0 | 4 | +4 |
| `g_raw_ref_r` | 0 | 4 | +4 |
| `g_config_len` | 0 | 4 | +4 |
| `g_raw_full_r` | 0 | 4 | +4 |
| `g_odo_rec` | 0 | 20 | +20 |
| `g_out` | 0 | 64 | +64 |
| `rw.0` | 0 | 256 | +256 |
| `g_raw` | 0 | 1,024 | +1,024 |
