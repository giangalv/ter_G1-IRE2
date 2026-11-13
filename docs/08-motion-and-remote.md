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
| 0 | Zero torque (the state after boot) | **L2+Y**, from damping | — |
| 1 | Damping | **L2+B** | `damp` / `fsm 1` |
| 4 | Locked stand: the legs hold the weight | **L2+Up**, from damping | `fsm 4` is allowed but has never been sent |
| **501** | **Regular mode**: slower walking | **R1+Y**, from 4 | `fsm 501`, from 4 |
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
| 7105 | SetVelocity `{"velocity":[vx,vy,omega],"duration":s}` | accepted (code 0) in 802: zero → nothing; 0.1 m/s for 0.5 s → no reaction; 0.2 m/s for 1.0 s → a small posture shift only (max \|dq\| 0.3 rad/s, "it moved but did not step"). The sticks in 802 and 501 do walk it (\|dq\| 1–3 rad/s). **Not yet tried in 501** |

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
of the caps (0.2 m/s, 0.1 m/s, 0.3 rad/s; hard ceiling 0.4 / 0.2 / 0.6), `space` zero velocity + disarm, `x` damp
(`SetFsmId 1`, repeated until FSM 1 is seen), `Ctrl-C` zero velocity and quit.

* **Dead-man**: a motion key acts for `--hold` s (0.2) after its last key event, so it needs the terminal's key repeat:
  a tap is a short step, a held key walks. A request is sent every 0.1 s with `"duration": 0.3`, so if the program, the
  cable or the Wi-Fi dies the robot's own timeout stops the command; on release a zero is sent at once.
* **Guards** (a trip disarms; press `r` again): FSM id must be **501** and fresh (the program polls 7001 at 2 Hz itself,
  the answers do not reliably carry `api_id`), `/lf/lowstate` not older than 0.5 s, |roll| and |pitch| < 0.35 rad, every
  velocity request answered with code 0 (three lost in a row trip it).
* Tested **offline only** (a fake `/api/sport` service and a pty that presses the keys): refusal without
  `G1_ALLOW_MOTION`, no arming in FSM 4, 0.1 m/s while held, zero within 0.1 s of the release, speed levels, damp, stale
  `lowstate`, tilt, `--dry` sending nothing, `Ctrl-C` mid-walk sending three zeros. Not yet run on the robot.
* The remote remains the person's backup (L2+B, ~5 s in a walking state); how the robot combines the remote's sticks
  with an API velocity has not been tested, so do not use both at once.

## Reproduce

```bash
source <DDS environment of docs/01>
python3 tools/remote_events.py 600 --quiet &       # keys + FSM changes, in another terminal
python3 tools/sport_call.py get                    # read-only
# a person: L2+B, then L2+Up (locked stand), then
G1_ALLOW_MOTION=1 python3 tools/sport_call.py fsm 501
```

## Open

* `SetVelocity` and `damp` in **501**; the first step on the floor.
* Whether `SetFsmId 4` and `SetFsmId 501` from damping (without the remote) work, and a software way out of 0 (zero torque).
* 550/702/1001, `GetStandHeight` code 7301, and what the remote's other keys (F1, F2, R2+Up/Down, START) do.
