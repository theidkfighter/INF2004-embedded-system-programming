# Change Requirement 1: step guides (start here)

These files let a **new Claude chat** or a **Kiro session** carry out one step of Change Requirement 1 (minimum-bit data representation) without reading the whole history. Each guide lists the tasks, what to read, the checks that must pass, and how to hand over.

The work is split into two tracks that can run in parallel: **firmware** (robot side) and **laptop** (codecs and tools). They meet only at the binary protocol, which is already fixed in writing (`INTEGRATION.md`).

Written 1 Nov 2026 against branch `Poh-Loon` at commit `542368f`.

## Files

| File | What it is |
|---|---|
| `README.md` | This page |
| `CONTEXT.md` | Project facts, commits, environment set-up (verified), build and test commands (the gate), rules for every session |
| `INTEGRATION.md` | How the two tracks stay compatible: the contract, file ownership, sync points, branches and merging |
| `PROGRESS_FW.md`, `PROGRESS_LT.md` | **The record of what's done**, one per track: task ledger, car checks, session log. Read first, update last |
| `REVIEW_FLAGS.md` | Issues found in the spec review (R1–R8). **Flagged only, not applied**, until the team marks one accepted |
| `spec/requirements.md`, `spec/design.md`, `spec/tasks.md` | Copies of the Kiro spec (snapshot of 1 Nov). The originals live in the gitignored `.kiro/specs/`. Requirements and design are the contract. The ticks in `spec/tasks.md` aren't maintained: use the PROGRESS files |
| `steps/firmware/FW-*.md` | Firmware track guides, one per session |
| `steps/laptop/LT-*.md` | Laptop track guides, one per session |
| `steps/INT-01_final.md` | Final integration session |

## The sessions

**Firmware track:**

| Guide | Tasks (spec/tasks.md) | Needs first | Car check after | Claude Pro | Kiro credits (rough) |
|---|---|---|---|---|---|
| `FW-00_baseline_measurement` | checkpoint 2 | — | **is** the car session | user | — |
| `FW-01a_repair_and_bitpack` | repair, 3.1–3.5, 3.7 | — | no | ½ window | 40–80 (Auto) |
| `FW-01b_wire_codec` | 3.9–3.13, 14.5 (C half) | FW-01a | no | 1 window | 150–250 (Opus) |
| `FW-01c_msg_codec_and_events` | 3.14–3.16, cp 4 | FW-01b | no | 1 window | 100–180 (Opus) |
| `FW-02_fault_mask` | 5.1–5.4 | FW-01c, **flag R1 decided** | with FW-03 | ½ window | 50–90 (Sonnet) |
| `FW-03_status_snapshot` | 6.1–6.8, cp 7 | FW-02 | **yes** (bench) | 1 window | 150–250 (Opus) |
| `FW-04_shared_scalars_messages` | 8.1–8.8, cp 9 | FW-03, flags R2/R3 decided | **yes** (turn, bypass) | 1 window | 150–250 (Opus) |
| `FW-05a_controller_contexts` | 10.1–10.7 | FW-04 | no | 1 window | 100–180 (Sonnet) |
| `FW-05b_parameter_store_map` | 10.8–10.12, cp 11 | FW-05a | no | 1 window | 120–200 (Opus) |
| `FW-06_isr_shared_data` | 12.1–12.6, cp 13 | FW-05b | **yes** (ISR timing) | 1 window | 120–200 (Opus) |
| `FW-08a_binary_events` | 16.1–16.3 | FW-06 | no | 1 window | 120–200 (Sonnet) |
| `FW-08b_acks_heartbeat` | 16.5–16.7 | FW-08a | no | ½–1 window | 80–150 (Sonnet) |
| `FW-08c_telemetry_frame` | 16.8–16.11, cp 17 | FW-08b; **S3** before flashing | **yes** (dashboard) | 1 window | 100–180 (Opus) |
| `FW-09_binary_commands` | 18.1–18.3, 18.5–18.8, cp 19 | FW-08c; **S5** before flashing | **yes** (commands, ESTOP) | 1 window | 150–250 (Opus) |
| `FW-10_docs_evidence` | 20.1–20.7 | FW-09 | measurements | 1 window | 80–150 (Auto/Sonnet) |

**Laptop track:**

| Guide | Tasks | Needs first | Check after | Claude Pro | Kiro credits (rough) |
|---|---|---|---|---|---|
| `LT-01_python_codec` | 14.1, 14.2 | — (can start now) | no | 1 window | 120–200 (Opus) |
| `LT-02_js_codec` | 14.3, 14.4 | LT-01 | no | 1 window | 100–160 (Sonnet) |
| `LT-03_tools` | 14.6–14.9, 14.18, cp 15 | LT-02 | tools vs robot (JSON) | 1 window | 120–200 (Sonnet) |
| `LT-04_cross_codec_tests` | 14.5 (Py half), 14.10–14.17, 16.4 | LT-03, **S1**; 14.10 also **S2** | no | 1 window | 120–220 (Sonnet) |
| `LT-05_binary_default` | 18.4, cp 19 laptop part | **S4** | **yes** (with FW-09) | ½ window | 30–60 (Auto) |

**Final:**

| Guide | Tasks | Needs first | Claude Pro | Kiro credits |
|---|---|---|---|---|
| `INT-01_final` | 20.8, final checkpoint 21 | **S6** (everything merged) | ½ window | 40–80 (Sonnet) |

**About the budget estimates:**
- **Claude Pro:** "window" means one 5-hour usage window of the Pro plan, estimated. Measure the first session and scale.
- **Kiro credits:** ±50%, estimated with the model in brackets. Opus 5.5 costs 2× Auto; Sonnet 1.3×. The student plan has 1,000 credits a month and no add-on credits. Both tracks on Kiro would need roughly 2,000–3,300 credits; splitting the tracks between Kiro and Claude Pro is the cheapest way.

**Order:**
- Inside a track, run the guides in the order listed.
- Across tracks, only the sync points S1–S6 in `INTEGRATION.md` matter.
- A step that needs the car can be coded without one, but its car check should be done, and logged in the PROGRESS file, before the step after next. That way, problems are found while they're still easy to trace.

## How to run a step with Claude (new chat)

1. Start a **new chat**. Long chats use more of the plan's usage.
2. Paste this as the first message, filling in the guide name:

   > Repository `theidkfighter/INF2004-embedded-system-programming`, branch `Poh-Loon`. Read `docs/cr1/README.md`, `docs/cr1/CONTEXT.md`, `docs/cr1/INTEGRATION.md` and the PROGRESS file of my track, then `docs/cr1/steps/<track>/<GUIDE>.md`, and carry out that step only. Follow the session rules in CONTEXT.md. Do not apply any item from REVIEW_FLAGS.md unless its decision says accepted.

   If the chat can't see the repo yet, attach `docs/cr1/` as a zip, or give it the GitHub link.
3. The chat clones the repo, sets up the tools (CONTEXT.md section 4), merges `Poh-Loon` into the track branch, does the tasks, runs the gate, updates the PROGRESS file and commits. Without push access it gives you a patch file instead (CONTEXT.md section 3).
4. Apply or merge the result into `Poh-Loon` (INTEGRATION.md section 4), and read the hand-over entry in the PROGRESS file before the next step.

## How to run a step with Kiro

1. Open the spec in Kiro and run only the tasks the guide lists, with the model the table suggests. Use Opus only where it says Opus.
2. Give Kiro the guide file as context. It contains the review notes and pitfalls that `tasks.md` doesn't have.
3. After the session, also tick the tasks in the track's PROGRESS file and commit it. Claude chats and Kiro sessions then see the same state.

## Never

- **Never commit secrets:** `config/wifi_credentials.h`, `config/mqtt_config.h` or any `.uf2` (they hold the hotspot password).
- **Never edit `spec/requirements.md` or `spec/design.md` inside a step session.** A needed change goes to `REVIEW_FLAGS.md` and waits for a team decision.
- **Never run the same step in two sessions** (Kiro and a chat) at the same time. Never edit the other track's files.
