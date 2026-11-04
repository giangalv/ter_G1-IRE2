# 02 — Inspire RH56E2 hands

## Identification

Labels on the hands: **`RH56E2-2R-T1`** (right) and **`RH56E2-2L-T1`** (left).
`E2` = second-generation RH56 hand, `T1` = variant with resistive tactile
sensing. Each hand has 6 actuators (5 fingers + thumb rotation).

| Hand | IP | How it was established |
|---|---|---|
| **Right** | `192.168.123.211` | the user pulled the right little finger: only `.211` reacted |
| **Left** | `192.168.123.210` | by exclusion (it never reacted) |

Do not assume the opposite: the Unitree convention would have suggested it.

## Interface: Modbus TCP, port 6000

* TCP **6000** is open on both hands, **502 is closed**. Unit id **1**.
* Function **0x03** (read holding registers) verified. Function **0x10**
  (write multiple registers) is the expected write path but is **not verified**
  on hardware.
* The `HAND_ID` register reads `1` on both hands (it is the Modbus slave id, it
  does *not* tell the hand model).

### Addressing: BYTE offsets

This is the one thing that bites. The Modbus "register address" is a **byte
offset**, and a read of `N` registers at an **even** address `A` returns the
`N` little-endian 16-bit words stored at bytes `A, A+2, A+4, …`.

* Documented register addresses (1486, 1546, 1582, …) are all even and work as written.
* Reads at an **odd** address return byte-shifted mixtures (e.g. `1280` shows
  up as `5` with its bytes swapped). Never use them.
* Reads whose address is **≥ 6000 drop the TCP connection**.
* Throughput: ~20 Hz per hand for a sweep of 3000 words in 100-word chunks
  (measured with both hands read in parallel).

### Register map (verified by reading, byte addresses)

| Name | Address | Count | Meaning / observed |
|---|---|---|---|
| `HAND_ID` | 1000 | 1 | `1` |
| `REDU_RATIO` | 1002 | 1 | `0` |
| `ANGLE_SET` | 1486 | 6 | commanded angle; at rest read `0, 0, -1, -1, -1, 117` on the right hand (`-1` probably "no target"; meaning not confirmed) |
| `FORCE_SET` | 1498 | 6 | force limit; factory value **2000** on all six |
| `SPEED_SET` | 1522 | 6 | speed; **1000** on all six |
| `ANGLE_ACT` | 1546 | 6 | measured angle, **0 = closed … 1000 = open** |
| `FORCE_ACT` | 1582 | 6 | measured force per **actuator** (signed 16 bit), idle noise ~100–140, swings to ~800 when pulled; units assumed grams (**unverified**) |
| `CURRENT`, `ERROR`, `STATUS`, `TEMP` | 1594, 1606, 1612, 1618 | 6/3/3/3 | answer with plausible values, not studied |

Channel order for every 6-value block: **little, ring, middle, index,
thumb-bend, thumb-rotation**.

At the time of the measurements both hands were in a closed fist
(`ANGLE_ACT` fingers ≈ 0, thumb rotation ≈ 984).

### Tactile arrays (`T1`)

`FORCE_ACT` has only 6 values per hand (one per actuator). The `T1` tactile data
is a separate block of **1062 cells in 17 regions**, byte addresses
**3000 … 5123**, one 16-bit register per cell, **0 = no contact … 4095**.
The layout is the one of Inspire's **RH56DFTP** manual (section 2.6.20, as
transcribed by the third-party `inspire-rh56dftp` Python driver) and it fits
this hand:

| Region (per finger) | Cells | Start address (little finger) |
|---|---|---|
| `tip` | 3×3 = 9 | 3000 |
| `nail` | 12×8 = 96 | 3018 |
| `pad` | 10×8 = 80 | 3210 |

Order in memory: **little, ring, middle, index** (`tip`, `nail`, `pad` each), then
the **thumb** (`tip` 4480 3×3, `nail` 4498 12×8, `middle` 4690 3×3, `pad` 4708
12×8), then the **palm** (4900, 8×14 = 112 cells). Each finger block is 185
cells = 370 bytes (little 3000, ring 3370, middle 3740, index 4110). All region
addresses are even and the whole block is contiguous, so it can be read in 11
requests of ≤ 125 registers (the Modbus limit). Nothing needs to be written.

**How it was checked on the real hands:** in the pull test of the right little
finger (2026-11-04) only `little_nail` (92 of 96 cells > 100 counts, peak 1438)
and `little_pad` (79 of 80, peak 1392) moved; every other region of both hands
stayed at ~0, and the left hand was silent. The idle baseline is 0 counts except
a constant load on the thumb tip (`thumb_tip` ≈ 90 on the left hand, ≈ 380 on the
right: the thumb rests against something in the closed fist).

**Cost:** a full read takes ~19 ms per hand (≈ 53 Hz at best). The hand accepts
**two simultaneous Modbus clients**, so a second connection can read the
tactile block without slowing the position loop.

**Not verified:**

* which physical part of the finger each region covers (the names are the
  manual's; "nail" and "pad" may not be where the words suggest);
* the force behind a count. There is **no usable calibration**: the sum of the
  little finger's cells did not correlate with its actuator force
  (`FORCE_ACT`, correlation 0.2), because the two measure different things.
  Press a known weight on a known region to calibrate;
* the palm and the thumb regions were never touched on purpose.

Tool: `tools/tactile_probe.py HOST [SECONDS] [--map]` prints the active regions
every second and draws a heat map of the most active one.

## What is known to be safe

* Reads (0x03) at even addresses < 6000: always; used extensively.
* Nothing was ever written. Do not write before checking `FORCE_SET`
  (factory 2000) and the pose, and keep a hand on the robot's stand/e-stop.

## Tools

* `tools/modbus_probe.py HOST` — read the main registers of one hand.
* `tools/modbus_capture.py SECONDS PREFIX` — wide capture of both hands to `.npz`
  (word `k` of a sample = byte address `2k`).
