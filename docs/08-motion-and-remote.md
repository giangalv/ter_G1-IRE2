# 08 — Moving the robot: FSM states, the remote's keys and the sport API

How the G1's locomotion is driven: the **FSM id** the robot reports, which **remote
keys** change it, which **`/api/sport` requests** change it, and what each one
did. All of it was measured on 2026-11-12 on the robot **hanging on its support
stand with its feet on the floor**, the remote in a person's hand and the laptop on
the cable (DDS environment of [01](01-network-and-computers.md)). The robot was
never walked on the floor. Tools: `tools/sport_call.py` (talks to the service) and
`tools/remote_events.py` (listens, sends nothing).

## FSM states

`GetFsmId` (api 7001) answers `{"data": n}`. A poll of it by somebody on the robot
runs at ~2 Hz all the time, so `/api/sport/response` always carries the current id.

| id | State | Remote | API (`tools/sport_call.py`) |
|---|---|---|---|
| 0 | Zero torque (the state after boot) | **L2+Y**, from damping | teleop key `z` (`SetFsmId 0` from damping: worked 2026-11-13 15:00:52) |
| 1 | Damping | **L2+B** | `damp` / `fsm 1`; teleop key `p` (also gets the robot out of 0, zero torque, 2026-11-13 15:01:09) |
| 4 | Locked stand: the legs hold the weight | **L2+Up**, from damping | `fsm 4` from damping worked 2026-11-13 15:01:15; teleop key `l` |
| **501** | **Regular mode**: slower walking | **R1+Y**, from 4 | `fsm 501`, from 4; teleop key `g` |
| 802 | **Running mode**: faster, more agile | **R2+A**, from 4 | — |
| 550, 702, 1001 | Seen only as transients/side effects on 2026-11-06 (550 for ~5 s after R1+X in 802; 702 after L2+X; 1001 flickered with R1+R2+X); not understood | | |

This robot has **29 motors** (3-DoF waist), and that decides the regular-mode key:

* **R1+Y enters the regular mode** (FSM **501**, both from the remote and, as `SetFsmId 501`, from the API).
* **R1+X does nothing**: held for 4 s from FSM 4 it changed neither the FSM nor the joints (on 2026-11-06 it had also been pressed many times, from FSM 1 and 4, without ever changing the FSM). R1+X is the regular-mode key of the G1 with a **1-DoF** waist.
* **Do not send `SetFsmId 500`.** The SDK helper `Start()` is `SetFsmId(500)`; a community wiki says 500 is the regular mode of the 1-DoF-waist version and that it jitters. `sport_call.py` only accepts 1, 4 and 501.
* The 6-Nov attempts used R1+X and R1+R2+X and never got a regular-mode state; R1+Y had not been tried.

Sources for the regular/running distinction are **not official Unitree documentation**: a
[community wiki](https://vat.pages.dev/RoboticsKnowledgebase/roboticsknowledgebase.github.io/blob/e25f04f00ad0e76d908d3c07dc21e95907be312e/wiki/common-platforms/unitree-g1.md)
(regular = 501 on the 3-DoF waist, running = 801, 500 = 1-DoF regular) and Unitree's
[xr_teleoperate README](https://raw.githubusercontent.com/unitreerobotics/xr_teleoperate/main/README.md)
(regular mode = R1+X, running = R2+A, only regular supported). Running was **802**, not 801, on this robot.
Claimed there and **not checked here**: in running mode arm control and velocity commands cannot be used
together (regular has no such limit); running balances slightly better. What was seen: regular is slower
(the person driving it), and in running mode the API velocity did not make steps (below).

## Sequence

```
zero torque (0) ──L2+B──▶ damping (1) ──L2+Up──▶ locked stand (4) ──R1+Y──▶ regular mode (501)
                                                       │  └──SetFsmId 501 (API)──▶ 501
                                                       └─R2+A──▶ running mode (802)
```

Leaving a walking state is **slow**: from the first L2+B to FSM 1 took ~5 s in 501 (three times) and in 802.
In 802 short taps did nothing, or only a 550 transient (2026-11-06); taps were not tried in 501. In 0, 1 and 4 the
keys act in ~0.3 s. L2+Up in 501 does nothing; from 501 or 802 go L2+B first. L2+Y then gives zero torque (it needs
damping first).

## The remote reaches the robot, not the API

The remote's keys and sticks arrive in **`/lf/lowstate.wireless_remote`** (40 bytes: 2 header,
`uint16` keys, five `float32`: lx, rx, ry, L2, ly). Bit order from the LSB: R1, L1, START,
SELECT, R2, L2, F1, F2, A, B, X, Y, Up, Right, Down, Left. A remote command shows
up only as a changed FSM id: no request appears on `/api/sport/request`.
`tools/remote_events.py` decodes this. START (bit 2) does reach the robot and did nothing in 802.

## `/api/sport` (ids from the official `g1_loco_client.hpp`)

Request/response shape: [05](05-unitree-dds.md). `header.identity.id` must be unique
(the answer carries it); QoS reliable, depth 1.

| api | Meaning | Measured |
|---|---|---|
| 7001 / 7002 | GetFsmId / GetFsmMode | answer in ~1 ms; 7002 always `{"data":0}` |
| 7005 | GetStandHeight | code **7301**, `{"data":0.0}` in every state tried |
| 7101 | SetFsmId `{"data":n}` | **1** from 4 and from 802: ack in 1 ms, FSM 1 within the second, leg mean \|tau_est\| 4 → 0.2 in 1–2 s (the remote's key needs ~5 s). **501** from 4: FSM 501 in ~1 s, entry transient \|tau\| ≈ 10 for a second, then settled (torso pitch +0.22 in 4 went to +0.04) |
| 7105 | SetVelocity `{"velocity":[vx,vy,omega],"duration":s}` | accepted (code 0) in 802: zero → nothing; 0.1 m/s for 0.5 s → no reaction; 0.2 m/s for 1.0 s → a small posture shift only (max \|dq\| 0.3 rad/s, "it moved but did not step"). The sticks in 802 and 501 do walk it (\|dq\| 1–3 rad/s). In **501**: 0.2 m/s for ~1 s walks, short taps only rock the torso, see the section above |

## Safety of the tools

* `sport_call.py` sends only getters (`get`, `watch`) unless `G1_ALLOW_MOTION=1`; velocity is capped at 0.2 m/s forward, 0.1 m/s sideways, 0.3 rad/s turning, 1 s; FSM ids are limited to 1, 4, 501; it aborts without sending if nobody answers.
* The software stop is `damp`: the robot goes limp at once (it does not brake), so use `stop` (zero velocity) first and `damp` as the emergency. The remote's L2+B is the person's backup but takes ~5 s in a walking state.
* Everything above was done on the support stand with the feet touching the floor; a walk on the floor needs its own risk assessment.

## Keyboard teleop (`tools/keyboard_teleop.py`)

```bash
source tools/g1_env.sh cable                       # or wifi, docs/07
G1_ALLOW_MOTION=1 python3 tools/keyboard_teleop.py # --dry: shows everything, sends only the FSM poll
```

It starts **disarmed**. `r` arms, `w`/`s` forward/back, `a`/`d` step left/right, `q`/`e` turn, `1 2 3` speed 25/50/100 %
of the caps (0.2 m/s, 0.1 m/s, 0.3 rad/s; hard ceiling 0.4 / 0.2 / 0.6), `Enter` (or space) zero velocity + disarm,
`Ctrl-C` zero velocity and quit. The status line says `ARMED  Enter/Space=STOP` while armed.

**The robot's state, from the keyboard (no remote needed to go up or down the ladder): a key HELD for 3 s**, the same
order as the remote's keys in the table above:

| Key | State | `SetFsmId` | Only from | Remote equivalent |
|---|---|---|---|---|
| `z` | zero torque (motors off) | 0 | damping (1) | L2+Y |
| `p` | damping (limp at once) | 1 | any state (it is the emergency) | L2+B |
| `l` | locked standing | 4 | damping (1) | L2+Up |
| `g` | regular mode (walking-ready) | 501 | locked standing (4) | R1+Y |

So the way up is `p`, `l`, `g` (then `r` to arm) and the way down is `p`. These are deliberate gestures: a tap, a stray
key, another key in between, or releasing early does nothing; the status line counts the seconds (`LOCKED STANDING l
1.2/3 s`); one request per hold (damping is repeated until FSM 1 is seen, the others are sent once and the program
waits 6 s for the new id, then says it did not change); a key that does not fit the robot's present state says so when
pressed (`g: REGULAR MODE starts from FSM 4 (locked standing): hold l first ...`) and does not count. It relies on the
terminal's key repeat, so a terminal that repeats slower than ~5 per second never reaches 3 s and the remote stays the
way out. `x` no longer does anything (it was the damping key until 2026-11-13). **All four were
used on the robot on 2026-11-13 (support stand, cable, remote in hand): `z` 15:00:52 (1 -> 0), `p` 15:01:09 (0 -> 1), `l`
15:01:15 (1 -> 4), `g` 15:01:53 (4 -> 501, with the usual entry transient of the legs), each seen as the new FSM id
within the ~0.5 s resolution of the recorder.**

* **Dead-man**: the first press of a key keeps the command alive for `--first-hold` s (0.6: it covers the terminal's
  key-repeat delay of ~0.5 s, and the gait needs ~1 s of command to start, see below); once the repeats arrive each
  one keeps it alive only `--hold` s (0.2), so on release the robot stops within ~0.2 s. A single tap is a 0.6 s
  command. A request is sent every 0.1 s with `"duration": 0.3`, so if the program, the cable or the Wi-Fi dies the
  robot's own timeout stops the command; on release a zero is sent at once.
* **Guards** (a trip disarms; press `r` again): FSM id must be **501** and fresh (the program polls 7001 at 2 Hz itself,
  the answers do not reliably carry `api_id`), `/lf/lowstate` not older than 0.5 s, |roll| and |pitch| < 0.35 rad, every
  velocity request answered with code 0 (three lost in a row trip it).
* **The Unitree remote must be ON** (it is the backup, L2+B damps the robot): at start the program waits 4 s for a
  non-zero `wireless_remote` payload in `/lf/lowstate` and, if there is none, refuses to run with `SAFETY: ... SWITCH ON
  THE UNITREE REMOTE first` (exit status 1, nothing sent; with `--dry` it only warns). The same check is a guard: arming
  needs a payload seen in the last 2 s, and if the remote goes off while armed the program stops the robot and disarms.
  What was seen: remote on and idle = `55 51` followed by zeros, constant for 8+ minutes (2026-11-13); remote not linked
  to the robot = all zero (2026-11-06); remote switched OFF = all zero too (2026-11-13: payload all zero from 14:52:30,
  `55 51` again at 14:52:52 after switching it on, about 22 s later; the exact delay after the switch-off was not
  timed). The teleop calls the remote lost when 2 s pass without a non-zero payload.
* Tested offline (a fake `/api/sport` service and a pty that presses the keys): refusal without `G1_ALLOW_MOTION`,
  no arming in FSM 4, speed levels, damp, stale `lowstate`, tilt, `--dry` sending nothing, `Ctrl-C` mid-walk sending
  three zeros, a held key with a 0.5 s repeat delay giving one continuous command (23 requests, largest gap 0.11 s) and
  a zero 20 ms after the last repeat, a tap giving 0.6 s. The state keys (2026-11-13, 21 scenarios): each of `z p l g`
  from the right state sends one `SetFsmId` and the new id is confirmed; from the wrong state it refuses at the press and
  sends nothing; `l` then `g` in a row reaches 501; a tap, 2 s, a hold interrupted by another key, alternating keys and
  the old `x` send nothing; an FSM that does not change, an error answer and `--dry` are reported; the remote missing at
  start refuses, and a remote that goes off mid-session disarms. On the robot: see the next section.
* The remote remains the person's backup (L2+B, ~5 s in a walking state); how the robot combines the remote's sticks
  with an API velocity has not been tested, so do not use both at once.

## What an API velocity does in regular mode (2026-11-13)

Robot on the support stand, feet on the floor, FSM 501 entered with R1+Y, laptop on the cable. Logs: the commands seen
on `/api/sport/request`, the IMU gyro and the leg joint speeds of `/lf/lowstate`.

| Command | Result |
|---|---|
| Taps of 0.3 s (3 requests) at 0.05-0.25 (forward 0.15 m/s, turn 0.25 rad/s, sideways 0.05 m/s) | the torso rocks; the person saw the **same movement for forward, left turn and right turn**; no net yaw (e.g. -0.66 -> -0.66 rad); the turn taps began with a yaw twitch of -0.9 and -1.0 rad/s whatever the sign (two samples) |
| `vx` +0.20 m/s for ~1.2 s (12 requests) | **real steps** (person's report); legs max speed 5.6 rad/s, gyro z within ±0.26 rad/s (straight) |
| `omega` +0.5 rad/s for ~1.2 s | turns left, integrated yaw +21° |
| `omega` -0.5 rad/s for ~2 s | turns right, integrated yaw -39° (person: "q and e work correctly") |

So, with this firmware and these calls: a velocity of 0.2 m/s (0.5 rad/s) held for about a second and more **walks**; the
short, small taps only make the robot rock. The minimum speed that steps, the distance covered, backward, sideways and the
sign of `vy` are **not measured yet**. The turns reach about 60-70 % of the commanded angle (0.6 and 1.0 rad
commanded, 0.37 and 0.67 measured), because of the start delay.

The remote's sticks, for comparison (the person moved the left stick forward, back, sideways, then the right stick to turn,
the same day): they step 0.3-0.9 s after a stick passes ~0.2; a backward stick held at 0.09-0.18 for 2.4 s did not step and
0.19+ did (one episode: a dead zone near 0.2 is likely, not proven); sideways did not step at 0.4-0.86 for 2 s but did at 1.0
(also one episode); full right stick turns at up to ±0.9-1.2 rad/s; positive `lx` is the stick to the right, positive gyro z a
turn to the left. The remote's values are mirrored on the DDS topic **`/wirelesscontroller`** (`unitree_go/WirelessController`:
`lx, ly, rx, ry, keys`; 2 publishers and 3 subscribers on the robot): it is silent while the sticks are idle and publishes at
20 Hz while they are used, with the same stick values as `/lf/lowstate.wireless_remote` (median difference 0). Who reads it
inside the robot is unknown. A third-party report ([unitree_sdk2_python issue 170](https://github.com/unitreerobotics/unitree_sdk2_python/issues/170),
one user, firmware 1.5.3, unverified) says that after `SetFsmId(501)` 0.1 m/s for 2 s did not walk and 0.2 m/s did.

## Reproduce

```bash
source <DDS environment of docs/01>
python3 tools/remote_events.py 600 --quiet &       # keys + FSM changes, in another terminal
python3 tools/sport_call.py get                    # read-only
# a person: L2+B, then L2+Up (locked stand), then
G1_ALLOW_MOTION=1 python3 tools/sport_call.py fsm 501
```

## Open

* Sideways: `a` / `d` at `vy` = +/-0.10 m/s (speed level 3 with the default `--vy 0.1`), 2-5 s each, on the stand (2026-11-13 15:03:49-15:04:02): **no sideways steps**, the torso only tilts (the person's report; legs max|dq| <= 0.8 rad/s against 1.8-2.8 rad/s when walking forward or backward). The sign of `vy` is therefore still unverified. Next: `--vy 0.2` (the hard ceiling), and if that does not step either, a higher ceiling for stand tests. Also open: the distance covered, `damp` from a walk, and the first step on the floor (the tests above were on the support stand).
* The minimum speed that steps in **501**: 0.2 m/s steps, on the cable and over the Wi-Fi (2026-11-13 14:34-14:36, laptop on the robot's AP, RViz following the robot: real steps, 96 velocity requests, one answer slower than 50 ms, no guard tripped). A 0.1 m/s command for ~2 s over the Wi-Fi (14:35:50): the person reported steps only at speed level 3 (0.2 m/s), so 0.1 m/s most likely does not step; nothing between 0.1 and 0.2 has been tried.
* Whether `SetFsmId 501` also works straight from damping (the keys only allow it from 4). (Done 2026-11-13: `SetFsmId 0` and `4` work from damping, and `p` gets the robot out of 0.)
* 550/702/1001, `GetStandHeight` code 7301, and what the remote's other keys (F1, F2, R2+Up/Down, START) do.
