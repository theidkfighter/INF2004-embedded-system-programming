# LT-02: JavaScript Laptop_Codec (`MQTT/is02_codec.js`)

| | |
|---|---|
| Track | Laptop (branch `cr1/laptop`) |
| Tasks | 14.3, 14.4 |
| Needs first | LT-01 done |
| Model | Sonnet (a port of the Python codec) |
| Budget | about 1 Claude window / 100–160 Kiro credits |
| Car | no |

## Read

- `PROGRESS_LT.md`: especially LT-01's notes on how unclear design points were read.
- `spec/design.md`: "C14. Laptop_Codec", D7–D15.
- `spec/requirements.md`: 17.1–17.3, 17.7, 17.9, 17.10.
- `MQTT/is02_codec.py` (from LT-01): the reference to port.

## Work

1. **14.3 field layer:**
   - Use a UMD wrapper: `<script src="is02_codec.js">` must work from `file://`, and `require()` in Node.
   - The same tables, CRC, `decodeFields` / `encodeFields`, and version and length checks, which throw `Error`.
   - Read bits MSB first. Use `DataView.getFloat32(.., false)` for F32 fields (SET values and the config pairs).
2. **14.4 presentation layer:**
   - `IS02.decode(topic, bytes)`, `IS02.encodeCommandText`, `IS02.encodeMap` and `IS02.encodeConfig`.
   - Legacy JSON mode.
   - Unknown-code and bad-payload handling must match the Python codec exactly, including the texts.
3. **Quick self-test,** `tests/host/js/codec_basic.test.js` (LT-owned):
   - the same hand-made records as `test_codec_basic.py`;
   - the CRC check value;
   - one telemetry decode.

   `node --test` picks it up through `run_all.sh` automatically.

## Watch out

- **No build step and no npm packages in `MQTT/`.** The dashboard loads the file directly. `fast-check` is only a test dependency in `tests/host/js/`.
- **Integers wider than 32 bits don't occur in the frames.** Keep the bit reader in plain Numbers, with care for fields of 24–32 bits: use `>>> 0`.
- **The Python and JS codecs must give identical bytes and identical decoded objects.** LT-04's Property 28 will check this against the C codec.

## Gate

The laptop gate in `CONTEXT.md` section 5, including `node -e "require('./MQTT/is02_codec.js')"`.

## Hand-over

**Commit:** `CR1 LT-02: JavaScript Laptop_Codec`.
