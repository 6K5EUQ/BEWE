# BEAE/uav — ExpressLRS 2.4 GHz link analysis (offline, receive-only)

Reverse-engineer and decode an ExpressLRS 2.4 GHz control link from a
wideband IQ recording, **without the bind phrase and without touching the
transmitter**. Pure offline analysis; no BEWE C++ core dependency, no IPC.

Nothing here is encrypted in ExpressLRS — the only "key" is the 6-byte UID
derived from the bind phrase, and everything on air (FHSS hop order, CRC,
sync/model-match) is derived from it. Recover the UID and the link is open.

## What it does

| Phase | Input | Output | Status |
|---|---|---|---|
| **A** | hop events `{index: channel}` | FHSS LCG seed → UID[2..5] | exact, source-ported |
| **B** | wideband IQ (e.g. 61.44 MSPS) | per-channel bursts → hop events | STFT channelizer |
| **C** | one SYNC burst's IQ | UID[4], UID[5] (CRC-validated) + payload | generic LoRa demod |

The realistic flow: **C** demodulates one SYNC packet → gives UID[4], UID[5]
(pins the CRC key and the low 16 bits of the seed) plus `fhss_index` (pins the
hop phase). **B** turns the snapshot into observed hops. **A** then needs only
a 2^16 search over UID[2], UID[3] — seconds — to recover the whole UID.

`python end_to_end.py` runs all three against a self-generated example.

## Hard limits (found, not assumed)

- **UID[0], UID[1] are never transmitted** in any form → unknowable by RF.
  They are not needed to follow or decode the link (only UID[2..5] are).
- **UID[2] bit 7 is a free bit in LoRa mode.** The LCG is `mod 2^31`, so seed
  bit 31 (= UID[2] MSB) changes no hop, and the CRC init never uses UID[2].
  UID[2] and UID[2]^0x80 are operationally identical. (FLRC mode would pin it
  via its 32-bit sync word; the 2.4 GHz LoRa rates have no sync word.)
- **Coverage.** 61.44 MSPS covers ~50–60 of the 80 × 1 MHz channels (band is
  79 MHz). That is plenty: `test_solve_sim` shows unique UID recovery from
  ≤20 observed hops even at 40/80 coverage, once UID[4], UID[5] are known.
- **SX1280 "LI" coding rate.** The ELRS 2.4 GHz LoRa rates use the chip's
  long-interleaved coding, which is **not in the ExpressLRS source** (it is in
  the SX1280 silicon). The chirp/FFT **symbol** demod here is generic and
  correct; the symbol→byte interleaver is the *classic* LoRa mapping, proven
  by round-trip. The exact SX1280 LI byte mapping must be calibrated against a
  real SYNC burst (known-plaintext) — `lora.solve_interleaver_from_known()` is
  the hook. Until then Phase C is validated on self-generated frames.

- **LoRa frame sync (Phase C, real captures).** `demodulate_symbols` is exact
  when given an explicit symbol `start` (all pipeline tests do). `find_preamble`
  does coarse frame detection + **integer-chip STO** correction (verified for
  sample-aligned and whole-chip offsets, os = 1–4, both IQ polarities). What it
  does **not** yet do — and what needs real IQ to calibrate — is carrier
  frequency offset (CFO) derotation and sub-chip/fractional-STO resampling.
  On a real capture, feed the demod an external coarse frame position (Phase B
  already timestamps bursts) so the residual STO stays within a chip.

## Layout

```
uav_elrs/
  fhss.py         LCG PRNG + hop-sequence build (1:1 with lib/FHSS)
  uid.py          bind phrase -> UID (MD5), UID <-> seed, seed recovery
  crc.py          ELRS CRC14/CRC16 (1:1 with lib/CRC) + packet validation
  packet.py       OTA packet layout, SYNC fields, 4x10 RC channel unpack
  detect.py       Phase B: IQ -> STFT channelizer -> hop events (+ test IQ gen)
  solve.py        Phase A/B: hop events -> seed match -> UID
  lora.py         Phase C: SX1280 LoRa modulate/demodulate (chirp/FFT/gray)
  sync_packet.py  Phase C: demod bytes -> SYNC -> validated UID[4], UID[5]
tests/            unit tests + a C cross-check of the numeric ports
end_to_end.py     full pipeline demo (no hardware)
PLAN.md           design + phase gates
```

## Verification

- `tests/*.py` — 23 unit tests. Symbol/packet/frame round-trips, solve
  uniqueness sweep, channelizer detection.
- `tests/xcheck.c` + `compare_xcheck.py` — transcribes the exact ExpressLRS C
  (LCG, sequence build, Crc2Byte) and asserts **byte-identical** output vs the
  Python port for several seeds and CRCs. This is the decisive check that the
  numeric ports are exact, not just self-consistent.

An adversarial audit (independent agents comparing each module to the source,
each finding then re-checked by a skeptic) surfaced and drove fixes for: a
`base_chirp` oversampling bug (chirp widened with os; real captures oversample
— fixed, now exact os = 1–4), the missing STO handling in `find_preamble`
(integer STO added; CFO/fractional flagged), a `channel_hz` register-vs-Hz
truncation (< 327 Hz, documented; `exact_channel_hz` added), the missing
numeric-UID shortcut in `bind_phrase_to_uid` (added), and a docstring line ref.

```bash
python -m venv .venv && . .venv/bin/activate && pip install -r requirements.txt
python -m pytest tests/ -q
gcc -O2 tests/xcheck.c -o /tmp/xcheck && /tmp/xcheck | python tests/compare_xcheck.py
python end_to_end.py
```

## When real IQ arrives (next, not done here)

1. Confirm which SDR actually sustains 61.44 MSPS (BladeRF likely; RTL can't
   reach 2.4 GHz; Pluto USB may bottleneck).
2. Feed the recording to `detect.Channelizer` → `solve.solve_from_sync`.
3. Calibrate the SX1280 LI interleaver from one captured SYNC burst
   (`lora.solve_interleaver_from_known`) to unlock full payload decode.

Source of truth: `~/ExpressLRS`, HEAD `5909f771`, `OTA_VERSION_ID = 4`.
