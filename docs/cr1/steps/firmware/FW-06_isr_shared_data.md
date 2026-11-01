# FW-06: ISR_Shared_Data (spec Step 6)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 12.1–12.6, checkpoint 13 |
| Needs first | FW-05b done, and FW-04's car check done |
| Model | **Opus** (ISR concurrency, 1 kHz timer ISR) |
| Budget | about 1 Claude window / 120–200 Kiro credits |
| Car | **yes**: checkpoint 13 (ISR timing vs FW-00, encoder course, barcode passes, ultrasonic medians) |

## Read

- `spec/design.md`: "C12. HAL changes", "D4. ISR_Shared_Data", "Critical-section strategy", Property 12.
- `spec/requirements.md`: Requirements 4.7, 5.4–5.8, 6.1, 6.2, 10 (all).
- The code: `hal_isr.c` (`g_bc_ring`, `g_raw_*`, `g_bc_*` flags, `g_adc_*`, `g_echo_*`, `g_vbat_mv`, `g_enc_rev`), `hal_board.c` (`g_motor_killed`, `g_servo_min_us/max_us`), `hal_imu.c` (config flags), and the consumer in `tsk_line.c` (`hal_barcode_pop`, `hal_barcode_pop_sample`).

## Work

1. **12.1 edge ring:**
   - `g_bc_ring[64]` holds one `uint32_t` per edge: `odo` mod 2³¹ in bits [0..30], black in bit 31.
   - Remove `t_ms` and its 64-bit division from the 1 kHz ISR.
   - The 8-bit head and tail use masked arithmetic. When the ring is full, drop the new entry and increment `g_bc_overruns`.
   - The pack and index helpers go in a plain header, shared with the tests. Update `tsk_line.c`.
2. **12.2:**
   - `g_raw_adc[256]` as U12 in `uint16_t`; 16-bit raw head and tail.
   - `g_bc_flags`: 8-bit, with the task read-modify-writes inside `crit_irq_*`.
   - `g_bc_thr`: 32-bit, stored while `enabled = 0`.
3. **12.3:**
   - `g_adc_last`: 16-bit.
   - `g_adc_rng`: read-and-reset inside `crit_irq`. Return the last value as both min and max when nothing was sampled.
   - `g_echo` (echo_us U15, done, rise_seen) with `g_echo_rise_us`. `hal_us_arm` stores inside `crit_irq`. `hal_us_read_cm` reads with one load and returns −1 for no echo or more than 25,000 µs.
   - `g_vbat_mv`: 16-bit; `g_vbat_div`: 8-bit.
4. **12.4:**
   - `g_enc_rev`: 8-bit, one store.
   - `g_motor_flags`, `g_servo_rng` (32-bit) and `g_imu_cfg` (16-bit).
   - **Keep** `g_enc[]`, `g_odo`, `g_enc_err` and `g_bc_overruns` in their own 32-bit units, with a comment giving the reason (Requirement 10.5/10.6).
5. **Tests:** 12.5 Property 12 (ring FIFO model, drop-new rule, overrun count, and the command-slot index arithmetic); 12.6 `test_adc_range.py`.
6. **Checkpoint 13:** gate. Write the car checks below into "Car checks waiting".

## Watch out

- **The raw sample ring:** the spec keeps `g_raw_er` / `g_raw_el` as they are (D4 says `g_raw_er + g_raw_el + g_raw_adc` stays 2,560 bytes). Don't invent a 16-bit encoder packing that the spec doesn't list.
- **Every interrupt-masked section must stay ≤ 10 µs** (Requirement 10.2): no loops or divisions inside.
- **ISR worst case must not grow by more than 10% or 1 µs** (Requirement 20.3). Removing the `t_ms` division should make the timer ISR faster.
- **`hal_us_read_cm` must return exactly `us / 58.0f`**, so the 8 cm proximity stop is unchanged.

## Gate

The firmware gate in `CONTEXT.md` section 5.

## Car check (user, before FW-08b starts)

- **`APP_PERF=1` build:** drive the track and compare the `[perf]` ISR max values with FW-00 (≤ 110% or +1 µs).
- **Encoder course:** 1,000 mm × 5 runs (within 1% of the baseline).
- **Barcode:** 5 passes, the same letters as the baseline.
- **Ultrasonic medians:** 20 cm and 100 cm, within ±1 cm.

The procedures are in `docs/evidence/correctness.md`.

## Hand-over

- **Commit:** `CR1 FW-06: packed ISR shared data`.
- Log the size line. `g_bc_ring` should be 768 → 256 bytes.
