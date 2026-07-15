---
name: mazak-decoder
description: Use when working on Mazak SmoothX/SmoothAI emulator behavior, pcap analysis, timeline, work search, DO101, DI101, DO102, cycle start, or door signals. Points to src/machinist/devices/machines/mazak_eip_decode.py CLI for decoding real machine captures.
---

# Mazak EtherNet/IP decoder

Use `src/machinist/devices/machines/mazak_eip_decode.py` (run via
`python3 -m machinist.devices.machines.mazak_eip_decode`) to decode Mazak
SmoothX/SmoothAI EtherNet/IP captures from pcap files.

## Quick reference

```bash
# Summary (connection profile, first/last frame)
python3 -m machinist.devices.machines.mazak_eip_decode <file.pcap>

# Timeline — bit-level change log, filtered to signals of interest
python3 -m machinist.devices.machines.mazak_eip_decode --timeline <file.pcap>

# Filter to specific signals (auto-resolves byte offsets)
python3 -m machinist.devices.machines.mazak_eip_decode --timeline --no-heartbeat --signal DO101,DI101,DO102 <file.pcap>

# Filter to specific byte offsets
python3 -m machinist.devices.machines.mazak_eip_decode --timeline --bytes 0,12,13 <file.pcap>

# Compare two captures
python3 -m machinist.devices.machines.mazak_eip_decode --diff <file_a.pcap> <file_b.pcap>

# Guide to output format
python3 -m machinist.devices.machines.mazak_eip_decode --guide
```

## Signal mapping notes

- Assembly bytes 12-13 are the control word:
  - DO101/DI101 at byte 12 bit 0
  - DO102/DI102 at byte 12 bit 1
  - DO103 at byte 12 bit 2
  - DO104 at byte 12 bit 3
  - DI109 (robot clear) at byte 13 bit 0
- Work number / program is a 32B ASCII field at byte 44
- Heartbeat is at byte 0 bit 0 (both DO000 and DI000)
