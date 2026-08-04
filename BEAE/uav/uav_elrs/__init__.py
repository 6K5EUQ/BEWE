"""BEAE/uav — ExpressLRS 2.4 GHz link analysis (offline, receive-only).

Ports the ExpressLRS over-the-air layer (FHSS, UID seeding, CRC, packet
structure) 1:1 from the firmware source at ~/ExpressLRS so that a wideband
IQ recording can be turned back into hop sequence -> UID -> payload without
knowing the bind phrase.

Modules
  fhss        LCG PRNG + FHSS sequence build (1:1 with lib/FHSS)
  uid         bind-phrase -> UID (MD5), UID <-> FHSS seed, seed recovery
  crc         ELRS custom CRC14/CRC16 (1:1 with lib/CRC) + packet validation
  packet      OTA packet layout (type, SYNC fields, RC channels)
  detect      Phase B: wideband IQ -> per-channel energy -> hop events
  solve       Phase A/B core: hop events -> LCG seed match -> UID
  lora        Phase C: SX1280 LoRa modulate/demodulate (dechirp/FFT/gray)
  sync_packet Phase C: demodulated bytes -> SYNC packet -> UID[4:5]

Source of truth: ~/ExpressLRS, HEAD 5909f771, OTA_VERSION_ID = 4.
"""

OTA_VERSION_ID = 4  # include/targets.h:8
UID_LEN = 6         # lib/OTA/OTA.h:11
