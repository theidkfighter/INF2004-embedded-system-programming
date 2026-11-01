# LT-01: Python Laptop_Codec (`MQTT/is02_codec.py`)

| | |
|---|---|
| Track | Laptop (branch `cr1/laptop`) |
| Tasks | 14.1, 14.2 |
| Needs first | nothing. Can start now, in parallel with the firmware track |
| Model | **Opus** (bit-exact decoding of all seven payload types, 45 event text formatters) |
| Budget | about 1 Claude window / 120–200 Kiro credits |
| Car | no |

## Read

- `CONTEXT.md`, `INTEGRATION.md` (sections 1–3: the contract and file ownership), `PROGRESS_LT.md`.
- **`spec/design.md`:**
  - "C14. Laptop_Codec";
  - "D7. Wire conventions" through "D15. Parameter_IDs" (all layouts and code tables);
  - "D10. Event_Frame" (the per-event detail-text rules) and "D11. Ack_Frame" (Reason_Code texts);
  - "Error Handling".
- **`spec/requirements.md`:** Requirements 16.1, 17.1, 17.2, 17.6, 17.7, 17.9, 17.10, 14.5 (the CRC).
- **The contract in code:** `app_program/wire_ids.h`, read only. Copy its tables; don't edit it.
- **Today's text formats,** so the decoded shapes match: `MQTT/telemetry_monitor.py` (`CSV_COLS`, how it reads the JSON keys), `MQTT/dashboard.html` (`onMessage`), and the baseline event texts (`grep -n "post_event(\|event2(" app_program/*.c` on `bca7964`: `git show bca7964:app_program/<file>`).

## Work

1. **14.1 field layer:**
   - `FORMAT_VERSION`; the event, reason, command, stop and abort tables; the Parameter_ID table (NAME, kind, min, max, in `PARAMS[]` order: read `app_program/app_params.c`); the frame lengths; and the CRC-16/CCITT-FALSE parameter table version.
   - `decode_fields(kind, payload)` and `encode_fields(kind, record)` for all seven payload kinds, over an MSB-first bit reader and writer.
   - Version and length checks raise `CodecError`.
   - A record holds raw codes, so decode followed by encode returns the input bytes.
2. **14.2 presentation layer:**
   - `decode(topic, payload)` returns the **baseline JSON shape** per topic (C14 lists every key), with baseline units, enum names, and the sentinels as `-1.0`, `""` and `[0,0]`.
   - The per-event detail-text formatters rebuild the baseline text (D10). Ack extras come back too.
   - **Bad payloads:** an unknown code shows as its number and is logged. A bad version or length is logged with topic, version and length, then dropped.
   - `encode_command_text(text)` covers every baseline command text (`ESTOP`, `PING:7`, `SET:LINE_STEER=0.35`, `MAP:...`, `BARCODE:A`, `START`, `STOP`, `RESUME`, `MAP:CLEAR`, `GET:MAP`, `GET:PARAMS`).
   - `encode_map(pairs)` and `encode_config(lines)` raise `CodecError` naming the line or argument and the NAME.
   - With `legacy_json=True`, a payload starting with `{` is parsed as baseline JSON.
3. **Quick self-tests in this session,** so the step can be verified before LT-04:
   - `tests/host/py/test_codec_basic.py` (an LT-owned file): encode/decode round trip on hand-made records for each payload kind;
   - the CRC check value `0x29B1` for "123456789";
   - `decode` of a hand-built telemetry frame gives the right JSON keys;
   - the legacy JSON pass-through.
   - The test finds the codec through `tests/host/py/codec_helpers.py` (`sys.path` → `MQTT/`).

## Watch out

- **Only `paho-mqtt` is allowed as an outside dependency for the tools.** The codec itself must use the standard library only.
- **The detail-text rounding must match `jb_fixed`** (half away from zero at the printed decimals). Flag R5 is on the C side, but your formatter must print exactly what the baseline printed for the same value.
- **Flag R4:** the map is D13's 4 movements. `encode_map` given two letters for one movement raises `CodecError` with a clear message, unless R4 is accepted.
- **Flag R1:** the fault names come from D1 as written, unless R1 is accepted. If it is, check `PROGRESS_LT.md` for the new names.
- **Don't edit** `app_program/`, `tests/host/conftest.py`, `run_all.sh` or the `Makefile` (`INTEGRATION.md` ownership).

## Gate

The laptop gate in `CONTEXT.md` section 5.

## Hand-over

- **Commit:** `CR1 LT-01: Python Laptop_Codec`.
- **Session log:** any place where the design was unclear and how you read it. LT-02 must make the same choice in JavaScript.
