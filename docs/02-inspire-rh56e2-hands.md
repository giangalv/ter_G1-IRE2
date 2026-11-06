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
  (write multiple registers) verified on **both hands** on 2026-11-04: per hand one
  write of `SPEED_SET` (300) and one of `ANGLE_SET` (open), both echoed
  `(address, count)` and read back exactly (`tools/open_hand.py`). Nothing else
  was ever written (`FORCE_SET` never).
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

**Where the regions are (measured 2026-11-04 with the hand open; fully on the right hand, a quick 19-step check gave the same result on the left):**
voice-guided presses with `tools/tactile_session.py`, one finger and one spot per
step, read-only. The rubber skin has two grip textures (user's description):
**dots** on the palm, the fingertip and the first, second and third phalanx, and
**lines** on the first phalanx too, but on the side that faces the neighbouring
finger, which cannot be touched with the fingers open. The thumb's skin looks
different.
First phalanx = the one nearest the hand, third = the one with the fingerprint.

| Region | Where it is | Evidence |
|---|---|---|
| `tip` (3×3) | the **extreme end** of the finger | press with a nail on the very end: index +2396, ring +1099, middle +929, little +1204, thumb +1104; all 9 cells react, peak near the centre cell |
| `nail` (12×8) | **palm side of the third phalanx** (the fingerprint side), not the back | pressing the fingerprint side lights it on all 5 fingers (+420 … +1730, 55-96 of 96 cells) |
| `pad` (10×8) | **palm side of the first phalanx** (the part nearest the hand) | index +963 (80 cells), middle +826, ring +1451…1596, little +1717 |
| `thumb_tip`, `thumb_nail` (12×8), `thumb_pad` (12×8) | same roles as on the fingers: extreme end, then the fingerprint side, then the part nearest the hand, along the **dotted** side | slide scan along the thumb: `thumb_tip` +315 → `thumb_nail` +763 → **nothing for about 30 % of the stroke** → `thumb_pad` +1167 |
| `thumb_middle` (3×3) | the **middle of the thumb**, between the tip and the base, dotted side (a small patch: a slide along the thumb missed it) | pressed directly: +592, 5 of 9 cells, peak at the centre cell (1,1) |
| `palm` (8×14) | palm side; the 14 columns run along the hand, **column 0 at the base of the fingers, column 13 towards the wrist**; the 8 rows across, row 7 probably on the thumb side (one press) | "near the wrist" → (3,13); "near the finger base" → (4,0); centre → (2…4, 8…12) |

* **There are no sensors on the back** of the fingers: pressing the dorsum of
  index, ring, middle and thumb produced nothing.
* **Not the whole finger is sensitive.** A slow slide of a fingertip along the index
  (palm side) hit, in order, `index_tip` → `index_nail` → **nothing for about
  a fifth of the stroke** → `index_pad`. Row 0 of `nail` and of `pad` is on the
  fingertip side, the last row on the side of the hand. The gap is where the second
  phalanx is: a press there is invisible to the array. The second phalanx has
  the dotted skin like the others, but pressed directly (middle of the dotted
  side) it gave nothing on the index, middle and ring finger, and +131 on two
  cells of the last row of `nail` on the little finger (the edge next to the
  second phalanx, coupling): **the second phalanx has no sensors.**
* The sensors **keep a small offset after a hard press** (+20 … +60 counts, once
  +390 on `thumb_tip`), so take a baseline before every measurement.

**Not verified:**

* whether the sensitive areas are exactly the **dotted** zones (the data fit that
  and not the whole sheath). The **lined** side of the first phalanx was pressed
  on the little finger (nothing above +20) and on the index (+35 on two edge
  cells, column 7 of `index_pad`, probably mechanical coupling), while the
  dotted side of the same phalanx gave +1810 and +1333 in the same run: the
  lined grip has **no sensors of its own**. The dotted second phalanx has none
  either, so the sensing surface is `tip`, the third and the first phalanx, and
  the palm (plus `thumb_middle` on the thumb);
* the thumb's lined grip and the position of its 12×8 regions along the thumb
  (only the order tip → nail → gap → pad is known);
* the force behind a count. There is **no usable calibration**: the sum of the
  little finger's cells did not correlate with its actuator force
  (`FORCE_ACT`, correlation 0.2), because the two measure different things.
  Press a known weight on a known region to calibrate;
* the finer details on the **left hand** (it was checked once per region, with the same result as the right: every region lit where expected, `thumb_middle` +796 at (1,1); the palm columns/rows were not compared);
* the left/right orientation of the columns of `nail`/`pad` and of the palm rows.

Tool: `tools/tactile_probe.py HOST [SECONDS] [--map]` prints the active regions
every second and draws a heat map of the most active one.

## What is known to be safe

* Reads (0x03) at even addresses < 6000: always; used extensively.
* The only writes so far are the ones of `tools/open_hand.py` on each hand (right first, then left, same result):
  `SPEED_SET` 300 then `ANGLE_SET` 1000 (thumb rotation kept at its current
  value). All fingers reached 996-998 in about 2 s with `ERROR` = 0 and the forces
  falling to ~0; `SPEED_SET` was restored to 1000 afterwards. Before writing,
  check `ERROR`, `FORCE_SET` (factory 2000) and the pose, keep the fingers of the
  people clear of the hand, and remember that `SPEED_SET` 300 is not slow (the
  whole stroke took about 2 s).
* The same two registers were then written by `tools/close_hand.py` (2026-11-06, user-approved, right hand first, then left):
  `SPEED_SET` 300, `ANGLE_SET` `[100, 100, 100, 100, 200, <thumb rotation as read>]`, i.e. a relaxed fist (fingers 100, thumb
  bend 200, nothing squeezed). Both hands got there in about 2 s: fingers 99-100, thumb bend 198, `ERROR` = 0, `FORCE_ACT` at most
  202 (the abort limit is 700); `SPEED_SET` was restored to 1000. `FORCE_SET` (factory 2000) was still never written. The tool does
  nothing without `--go`. Close the hands before moving the arms (fingers do not snag); the fist covers the palm cameras, open
  again with `open_hand.py` before manipulation work.
* Thumb (2026-11-06, right hand, user-approved, `tools/thumb_test.py --go`): the thumb has two actuators, **bend** (channel 4,
  moved 1000 -> 198 by `close_hand.py`) and **rotation** (channel 5). `close_hand.py` and `open_hand.py` leave the rotation where it
  is, so the first close did not rotate it. The rotation test (`SPEED_SET` 300, set value 956 -> 850 -> 700 -> 956, the other five
  channels held) was reached at every step: measured 828, 683, back to 942, `ERROR` = 0, `FORCE_ACT` unchanged (thumb rotation
  about -90 the whole time, no obstacle); every step finished within the first 0.4 s sample, so the rotation is fast even at
  `SPEED_SET` 300. `open_hand.py HOST destra --keep-thumb` opens only the four fingers.
* **Full stroke of the thumb, right hand, same day** (`thumb_test.py --go --bend=1000 --steps=700,500,300,150,0 --poll=0.1 --stall=0.5`,
  then `--channel=bend --steps=100,0`): the rotation went through 700, 500, 300, 150 and **0** (measured 729, 517, 326, 137, 9) and
  came back to 941 (set 956) in 1.8 s; the bend went from 198 to 1000 (measured 997) and, with the rotation at rest, down to **0**
  (measured 2) and back to 198. No contact, no stall, `ERROR` = 0, `FORCE_ACT` of the moving channel stayed within about
  +-40 of its idle value (it shifts by up to ~120 while the actuator moves, with the sign of the direction), nothing came near the
  abort limit 700. So both thumb channels use the **whole 0 ... 1000 range** and 700 was not an end stop. The rotation covers about
  1000 counts in 1.8 s and the bend about 800 in 1.6 s at `SPEED_SET` 300. The right hand was left with the four fingers open
  (~998), thumb bend 198 and rotation 941 (set 956); the left hand is still a closed fist and was not tested.
* `thumb_test.py` stops a step on contact (force of the moving channel more than 250 over its idle value) or when it has not moved
  for `--stall` seconds, backs the set value off 40 counts and goes straight to the way back; it did not trigger in these runs, so
  that path is untested on the hardware.
* **The actuators rest ~15 counts below their set value** (set 969 -> measured 953-956; set 956 -> 941; set 850 -> 828; 700 -> 683).
  Writing the measured value back as the new set value therefore makes a held channel creep by ~15 per run; the tools now write
  back the last set value instead (`hold()`), unless it is far from the measured one (the register reads garbage after a
  power-up: 117 / 110 on the thumb rotation while it measured 969).

## Tools

* `tools/modbus_probe.py HOST` — read the main registers of one hand.
* `tools/close_hand.py HOST [destra|sinistra] [--go]` — close one hand slowly to a relaxed fist; without `--go` it only reads and
  prints the plan. `tools/open_hand.py` is the way back.
* `tools/modbus_capture.py SECONDS PREFIX` — wide capture of both hands to `.npz`
  (word `k` of a sample = byte address `2k`).
