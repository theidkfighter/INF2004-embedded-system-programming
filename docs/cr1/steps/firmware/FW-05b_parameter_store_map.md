# FW-05b: Parameter_Store and barcode map (spec Step 5, second half)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 10.8–10.12, checkpoint 11 |
| Needs first | FW-05a done |
| Model | **Opus** (55 parameters, value equivalence with the baseline) |
| Budget | about 1 Claude window / 120–200 Kiro credits |
| Car | no |
| Unblocks | merge into `Poh-Loon` = sync point **S2** (LT-04 task 14.10 needs `params_table_crc`) |

## Read

- `spec/design.md`: "C11. Parameter_Store", "D15. Parameter_IDs and Parameter_Store layout", Properties 22 and 24 (the C side of the CRC).
- `spec/requirements.md`: Requirements 14.5, 15 (all), 2.6.
- `REVIEW_FLAGS.md` R4 (map). It doesn't affect this step's internal map.
- The code: `app_params.h/.c` (`PARAMS[]`, `params_set`, `params_apply`), every `g_params.<discrete>` read (`grep -n "g_params\.\(enc_._reversed\|line_do_black_high\|barcode_levels\|turn_snap\|barcode_at_junction\|barcode_detect\|barcode_debug\|obst_legacy\|obst_trust_miss\|servo_low_angle_is_right\|servo_calibrated\|motor_mapping_confirmed\|enc_direction_confirmed\|imu_forward_axis\|imu_forward_sign\)" app_program/*.c`), `nav_map.c/.h`.

## Work

1. **10.8 Parameter_Store:**
   - `param_id_t` follows the `PARAMS[]` order (55 IDs), plus `PRM_ID_UNKNOWN`.
   - The 16 discrete parameters move into `g_params.bits` (D15 bit table); the 39 floats keep their members.
   - Descriptors gain kind, shift, width and encoding.
   - Add `params_get`, `prm_flag`, `prm_tri`, `prm_int`, and `params_set_id`. `params_set_id` returns 0 ok, −1 unknown, −2 NaN/inf, −3 out of range or not whole, and stores with one composed 32-bit store.
   - Add `params_put_wire`.
   - Add `params_table_crc`: CRC-16/CCITT-FALSE over the NAMEs, each followed by `'\n'`, computed in `params_init`.
   - **Text `params_set` and the JSON SET / GET:PARAMS replies keep working.**
2. **10.9:** replace the discrete reads at every call site (`prm_flag`, `prm_tri`, `prm_int`).
3. **10.10 Property 22:** the value model vs the Baseline_Build float table (`libbase.so`), including −1 (CAL_UNSET) for the tri-states.
4. **10.11 `g_map_w[3]`:** letter `i` in word `i/10`, bits `3·(i mod 10)`, composed single-writer stores. `nav_map_describe()` output stays the same.
5. **10.12:** the map round-trip and describe-text tests.
6. **Checkpoint 11:** gate, with Properties 13, 14, 15 and 22 passing.

## Watch out

- **Rejecting non-whole values:** a discrete parameter given a non-whole value (for example `TURN_SNAP=0.5`) must be **rejected** like an out-of-range value (Requirement 15.6). The baseline accepted it as a float, so this is the one intended change; note it in the session log.
- **`nav_map_parse` keeps accepting the pair form** (several letters per movement) internally. Flag R4 only concerns the binary Map_Frame.
- **The CRC check value** for "123456789" is `0x29B1`; test it.

## Gate

The firmware gate in `CONTEXT.md` section 5.

## Hand-over

- **Commit:** `CR1 FW-05b: Parameter_Store, packed barcode map (Step 5 complete)`.
- **`PROGRESS_LT.md`:** add "S2 ready when merged".
