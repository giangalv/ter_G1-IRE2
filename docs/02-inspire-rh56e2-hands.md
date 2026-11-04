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
  (write multiple registers) verified on the **right hand** on 2026-11-04: one
  write of `SPEED_SET` (300) and one of `ANGLE_SET` (open), both echoed
  `(address, count)` and read back exactly (`tools/open_hand.py`). Nothing else
  was ever written, and nothing at all on the left hand.
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

**Where the regions are (right hand, measured 2026-11-04 with the hand open):**
voice-guided presses with `tools/tactile_session.py`, one finger and one spot per
step, read-only. The rubber skin has two grip textures (user's description):
**dots** on the palm, the fingertip, the first phalanx and the third phalanx, and
**lines** on the first phalanx too, but on the side that faces the neighbouring
finger, which cannot be touched with the fingers open. The second phalanx is
not described. The thumb looks different (not mapped yet).
First phalanx = the one nearest the hand, third = the one with the fingerprint.

| Region | Where it is | Evidence |
|---|---|---|
| `tip` (3×3) | the **extreme end** of the finger | press with a nail on the very end: index +2396, ring +1099, middle +929, little +1204, thumb +1104; all 9 cells react, peak near the centre cell |
| `nail` (12×8) | **palm side of the third phalanx** (the fingerprint side), not the back | pressing the fingerprint side lights it on all 5 fingers (+420 … +1730, 55-96 of 96 cells) |
| `pad` (10×8) | **palm side of the first phalanx** (the part nearest the hand) | index +963 (80 cells), middle +826, ring +1451…1596, little +1717 |
| `thumb_tip`, `thumb_nail` (12×8), `thumb_pad` (12×8) | same roles as on the fingers: extreme end, then the fingerprint side, then the part nearest the hand, along the **dotted** side | slide scan along the thumb: `thumb_tip` +315 → `thumb_nail` +763 → **nothing for about 30 % of the stroke** → `thumb_pad` +1167 |
| `thumb_middle` (3×3) | **not located**: it did not light on the slide along the dotted side, nor on a second slide along the inner side (that one showed only the standing +100 on `thumb_tip`, and it is not certain that it was pressed) | |
| `palm` (8×14) | palm side; the 14 columns run along the hand, **column 0 at the base of the fingers, column 13 towards the wrist**; the 8 rows across, row 7 probably on the thumb side (one press) | "near the wrist" → (3,13); "near the finger base" → (4,0); centre → (2…4, 8…12) |

* **There are no sensors on the back** of the fingers: pressing the dorsum of
  index, ring, middle and thumb produced nothing.
* **Not the whole finger is sensitive.** A slow slide of a fingertip along the index
  (palm side) hit, in order, `index_tip` → `index_nail` → **nothing for about
  a fifth of the stroke** → `index_pad`. Row 0 of `nail` and of `pad` is on the
  fingertip side, the last row on the side of the hand. The gap is where the second
  phalanx is: a press there is invisible to the array. Pressing the palm side of
  the second phalanx gave nothing on the index and the middle finger; on the ring
  and the little finger the same cue lit `pad`, probably because the press landed
  on the edge of the first phalanx.
* The sensors **keep a small offset after a hard press** (+20 … +60 counts, once
  +390 on `thumb_tip`), so take a baseline before every measurement.

**Not verified:**

* whether the sensitive areas are exactly the **dotted** zones (the data fit that
  and not the whole sheath), and whether the **lined** side of the first phalanx
  has sensors (not reachable with the fingers open; the `pad` region may cover
  it). A test with the second phalanx pressed directly, one finger at a time,
  is still to be done; the earlier try on four fingers is not usable because
  the lined grip is not where it was asked to press;
* where `thumb_middle` (3×3) is: the thumb's skin is laid out differently from the
  other fingers, and its lined grip is probably the place to look;
* the force behind a count. There is **no usable calibration**: the sum of the
  little finger's cells did not correlate with its actuator force
  (`FORCE_ACT`, correlation 0.2), because the two measure different things.
  Press a known weight on a known region to calibrate;
* the **left hand** (same layout assumed, only the right was mapped);
* the left/right orientation of the columns of `nail`/`pad` and of the palm rows.

Tool: `tools/tactile_probe.py HOST [SECONDS] [--map]` prints the active regions
every second and draws a heat map of the most active one.

## What is known to be safe

* Reads (0x03) at even addresses < 6000: always; used extensively.
* The only writes so far are the ones of `tools/open_hand.py` on the right hand:
  `SPEED_SET` 300 then `ANGLE_SET` 1000 (thumb rotation kept at its current
  value). All fingers reached 996-998 in about 2 s with `ERROR` = 0 and the forces
  falling to ~0; `SPEED_SET` was restored to 1000 afterwards. Before writing,
  check `ERROR`, `FORCE_SET` (factory 2000) and the pose, keep the fingers of the
  people clear of the hand, and remember that `SPEED_SET` 300 is not slow (the
  whole stroke took about 2 s).

## Tools

* `tools/modbus_probe.py HOST` — read the main registers of one hand.
* `tools/modbus_capture.py SECONDS PREFIX` — wide capture of both hands to `.npz`
  (word `k` of a sample = byte address `2k`).
