# Review flags (spec review, 1 Nov 2026)

These are points found when the spec (`spec/requirements.md`, `spec/design.md`, `spec/tasks.md`) was checked against Change Requirement 1 and the code on `Poh-Loon` (`542368f`).

**They are flagged only, not applied.** Every session follows the spec as written unless an item's **Decision** below says `accepted`. Only the team changes a decision.

- **Changing a decision:** edit the Decision line, and add the date and who decided.
- **New flags:** append them at the end (R9, R10, ...). Never renumber.

---

## R1. Motor stall shares one bit with encoder loss, and two fault bits are duplicates

- **Spec:** Req 3.1–3.3, design D1, tasks 5.1, 5.2, 5.4.
- **Issue:**
  - **Motor stall shares a bit.** Change Requirement 1 lists "motor stall" and "encoder fault" as separate example fault conditions, each to be an individual bit. The spec deliberately makes `FAULT_ENCODER_LOSS` "the only motor stall and encoder fault bit".
  - **Two duplicate bits.** The 16-bit mask also keeps two bits that are defined but never set, while adding new bits with the same meaning:
    - bit 4 `FAULT_ULTRASONIC` (never set) vs new bit 15 `FAULT_US_NO_ECHO`;
    - bit 12 `FAULT_TELEMETRY` (never set) vs new bit 14 `FAULT_MQTT_LOSS`.

    That conflicts with Req 3.6 ("replace each variable that duplicates a Fault_Mask condition").
- **Suggested change (if accepted):**
  - Use bit 4 for ultrasonic no-echo and bit 12 for MQTT session loss.
  - Keep bit 13 for Wi-Fi loss.
  - Add `FAULT_MOTOR_STALL` at bit 14, set when *both* wheels have duty above `ENC_LOSS_DUTY` and no ticks for `ENC_LOSS_MS`. Keep `FAULT_ENCODER_LOSS` for one wheel only. Both still request ERROR as today.
  - Bit 15 is free.
  - The mask stays 16 bits.
  - Touches Req 3.1–3.3/3.8, D1, D8/D9 (only the fault names), tasks 5.1/5.2/5.4, the codecs' fault name tables (LT track) and the dashboard fault list.
- **When it must be decided:** before FW-02 starts. Cheap now, expensive after FW-02.
- **Decision:** open

## R2. Requirement 2.8 (every `bool` packed) isn't fully covered by the tasks

- **Spec:** Req 2.8; design D5; tasks 10.1, 10.3, 8.2, 18.8 (optional).
- **Issue:** Req 2.8 requires *every* `bool` member of a controller context and *every* file-scope `bool` of a task to be a 1-bit Packed_Field. Two groups aren't covered by a required task:
  - **Barcode contexts** (only in optional task 18.8):
    - `bc_ctx_t`: `b_open`, `b_black`, `bar[96]`;
    - `det_t` / `adt_t`: `b_black`, `b_inv`, `b_started`, `b_have_prev`;
    - `bcm_t`: `rev[6]`, `b_pass`, `b_settle`, `b_conflict`.
  - **File-scope flags in no task:**
    - `app_net.c`: `g_led`, `g_led_applied`, `g_connecting`, `g_ever_connected`, `g_join_pending`, `g_broker_set`;
    - `tsk_safety.c`: `g_estop_reported`. Task 8.2 may mean it as part of `g_net_flags`, but doesn't name it.
- **Options:**
  - **(a)** Make 18.8 required and add a task for the `app_net.c` and `tsk_safety.c` flags in FW-04.
  - **(b)** Narrow Req 2.8 to the listed contexts, and give the reason for the others in the data table.
- **When:** before FW-04 (the `tsk_safety.c` / `app_net.c` flags) and before FW-09 (18.8).
- **Decision:** open

## R3. The message-pool RAM saving would be measured on an unused buffer

- **Spec:** design D6, D17; Req 22.3; tasks 8.4, 16.1.
- **Issue:**
  - The spec counts `g_mpf_buf` shrinking from 4,000 to 1,440 bytes as a RAM saving.
  - But `os_glue.c` creates the pool **without `TA_USERBUF`**: `.mpfatr = TA_TFIFO | TA_RNG3`. With `USE_IMALLOC = 1` the kernel then allocates its own pool memory outside `.bss` (`kernel/tkernel/mempfix.c`), and `g_mpf_buf` is never used.
  - So the reported saving is on a dead buffer, and the real pool shrink (40 × 96 = 3,840 → 40 × 32 = 1,280 bytes) never appears in `arm-none-eabi-size`.
- **Suggested change (if accepted):**
  - In task 8.4 (or 16.1), add `TA_USERBUF` to `mpfatr`.
  - Size `g_mpf_buf` exactly `MSG_BLOCKS × block size`. The block size must be a multiple of `sizeof(FREEL)`, which 32 bytes is (V2 in the old plan; `mempfix.c:89-92`).
  - Then the measured buffer is the real pool. Note it in `docs/evidence/ram_usage.md`.
- **When:** before FW-04.
- **Decision:** open

## R4. The binary map frame can't carry two letters for the same movement

- **Spec:** Req 14.1, 14.6; design D13.
- **Issue:** today `MAP:A=FWD,B=FWD,...` (pair form) can map several letters to one movement (`nav_map.c`). The 4-slot Map_Frame holds one letter per movement. The demo needs only one letter per movement, but this is a functional difference that Req 19 doesn't list.
- **Options:**
  - **(a)** Record it as an allowed difference, in the data table and `docs/evidence/correctness.md`.
  - **(b)** Use a 26 × 3-bit map body (78 bits; the Map_Frame becomes 11 bytes).
- **When:** before LT-01 and FW-09.
- **Decision:** open

## R5. `bp_quant` doesn't round exactly like `jb_fixed`

- **Spec:** design D0 rule 1 ("the same rule `jb_fixed()` uses"); Req 1.8, 17.3.
- **Issue:**
  - **Rounding:** `bp_quant` rounds `(double)x * scale` exactly. `jb_fixed` (today's JSON text) rounds `|x| * 10^d + 0.5f` in **float**. In rare boundary cases, the decoded binary value is therefore one step away from the value the Baseline_Build JSON showed. Strictly, that fails Req 17.3 ("within half a resolution of the value the Baseline_Build JSON carried").
  - **Cost:** double arithmetic is done in software on the Cortex-M0+, which adds time to every status write.
- **Options:**
  - **(a)** Round in float exactly like `jb_fixed` (then the binary and baseline text agree bit for bit).
  - **(b)** Keep double, make test 3.13 (`test_resolution.py`) list the boundary cases, and state the difference in `correctness.md`.
- **When:** before FW-03 (first heavy use of `bp_quant`).
- **Decision:** open

## R6. No task is marked optional, so all 129 look required

- **Spec:** tasks.md "Notes" says test tasks marked `*` are optional, but no task carries a `*`.
- **Issue:** the property tests (about 30, many run in C, Python and JavaScript) are roughly a third of the remaining work and of the Kiro credits.
- **Suggestion:** mark as optional the cross-language duplicates that a C or Python test already covers (for example, 14.13 Property 27 in JS once Python passes). Keep 14.11/14.12 (round trips), 14.14 (cross-codec vectors), the differential Properties 11, 13, 14, 15, 22, and Property 18.
- **Decision:** open

## R7. Step 7 tools: commands in legacy mode must stay text

- **Spec:** task 14.6 ("commands sent with `encode_command_text`"), 14.7, 14.8; design C14/C15; Step 7 checkpoint ("tools still work against the Step 6 JSON firmware with `--legacy-json`").
- **Issue:** `--legacy-json` is described only for **reading** JSON. If the Step 7 tools send binary Command_Frames, a robot that hasn't reached FW-09 (text commands only) rejects every command, including ESTOP from the dashboard.
- **How the guides handle it (no spec change):**
  - LT-03 sends text commands while `--legacy-json` is on, and binary only when it's off.
  - LT-05 switches the default after FW-09.
  - This follows the Step 7 checkpoint's intent.
- **Decision:** open (guides follow the interpretation above unless the team says otherwise)

## R8. Later obstacle refinements will fail Property 14 against the Baseline_Build

- **Spec:** Req 6.7, task 10.5 (Property 14: obstacle planning identical to the Baseline_Build `bca7964`).
- **Issue:** the Buddy 5 refinements still planned (rejoin check, reverse-cap fallback, side-ping proximity, deep-obstacle `p_ended` fix; see `REVIEW_ultrasonic-refinement_2026-11-01.md` on the user's laptop) change `tsk_obstacle.c` / `obstacle_logic.c` on purpose.
  - The differential test compares with the frozen Baseline_Build, so it would fail after such a change.
  - A behaviour change in the middle of CR1 also muddies the "no behaviour change" evidence (Req 19.7).
- **Suggested rule (if accepted):**
  - Do the obstacle refinements **after** the CR1 Week 13 measurements, as separate commits outside CR1.
  - If one is needed earlier, make it a separate commit labelled `REFINE:`. Re-run the obstacle bench check on both builds. Update the Property 14 reference to the refined source in the same commit, and say so in `correctness.md`.
- **Decision:** open
