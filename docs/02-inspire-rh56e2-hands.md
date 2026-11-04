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

### Tactile (`T1`)

`FORCE_ACT` has only **6 values per hand** — one per actuator — *not* the 15
per-phalanx values of the older RH56DFQ. The `T1` resistive tactile data is
somewhere else: a block of ~174 registers around byte addresses **3018 … 3368**
moved by up to ~1400 counts on the right hand while the little finger was
pulled, so it probably is the tactile array. **Layout and sensor count are not
decoded.** Getting a clean map needs the hand open and a controlled press on
known spots (a write command to open the hand is required first).

## What is known to be safe

* Reads (0x03) at even addresses < 6000: always; used extensively.
* Nothing was ever written. Do not write before checking `FORCE_SET`
  (factory 2000) and the pose, and keep a hand on the robot's stand/e-stop.

## Tools

* `tools/modbus_probe.py HOST` — read the main registers of one hand.
* `tools/modbus_capture.py SECONDS PREFIX` — wide capture of both hands to `.npz`
  (word `k` of a sample = byte address `2k`).
