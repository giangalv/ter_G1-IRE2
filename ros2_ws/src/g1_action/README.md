# g1_action

Safety-critical realtime execution package for the G1 U6 hybrid VLA workspace.
All names, joint maps and QoS follow `CONVENTIONS.md` (authoritative).

## Nodes

### `action_executor_node` (regular Node, realtime CPU, 50 Hz)

Executes `g1_interfaces/ActionChunk` with a receding horizon. ONLY writer of
`/g1/arm_sdk`, `/g1/cmd_vel` and `/g1/hand/{left,right}/cmd` on the VLA path.

* **ChunkBuffer** (`executor_core.py`): decodes chunks to `[T,53]` / `[T,10]`,
  pops one row per tick; a new chunk is cross-faded with the remainder of the
  old one over `replan_overlap * T` steps; chunk exhaustion (replan gap
  ~100-200 ms) and `chunk_invalid` produce an impedance HOLD — last safe body
  targets, zero velocity, grip force retained (never released).
* **Dual-path blending** (`blending.py`): on FAST <-> PRECISE mode switches,
  `alpha` ramps 0 -> 1 linearly over `blend_window_steps` ticks and the output
  is `(1 - alpha) * fast + alpha * precise` (passthrough when only one path is
  available).
* **SafetyPipeline** (`safety.py`), exact stage order:
  1. dual-encoder fault (`|primary - secondary| > dual_encoder_fault_threshold`)
     -> latched ESTOP;
  2. per-joint velocity clamp to `max_joint_velocity_rad_s / rate`;
  3. torque clamp — demand estimated as `stiffness_nm_per_rad * tracking error`,
     limited to `max_joint_torque_nm` (120 N·m knee spec);
  4. workspace boundary check on approximate wrist FK -> BLOCK (hold);
  5. self-collision capsule check (both forearms + torso, clamped
     closest-point segment distance) -> BLOCK (hold);
  6. execute.
* **Hand execution**: HAND-12 slice -> normalized `HandCmd.position` via the
  `hand_joint_{min,max}_rad` limit map; `force_limit` from
  `ActionChunk.hand_force`. During `grasp_precision` subtasks a 100 Hz
  force-position hybrid PI loop (`hand_force_kp/ki`) runs on the tactile
  arrays; a mean-pressure drop > 40 % mid-grasp raises TACTILE_SLIP handling:
  position freezes and the force limit is bumped 20 % (recovery is planned
  elsewhere).
* **Navigation**: while the subtask starts with `navigate_to`, a P-controller
  drives `/g1/cmd_vel` toward the DynaMem object pose in the odom frame
  (caps 0.4 m/s / 0.5 rad/s); otherwise a zero Twist is published.
* **ESTOP**: on a safety ESTOP the node latches, publishes `/g1/estop` and
  `/g1/diagnostics/encoder_fault` (latched), ramps `ArmCmd.weight` to 0 over
  0.5 s (smooth release), zeroes `cmd_vel` and freezes the hand commands
  (grip held). An external `/g1/estop` True stops output the same way.

Diagnostics: `/g1/diagnostics/encoder_delta` (`Float64MultiArray`, 41 values)
at 10 Hz.

### `risk_estimator_node` (regular Node, realtime CPU, 10 Hz)

Publishes `g1_interfaces/RiskScore` on `/g1/risk/score`:

* `collision_proximity = sigmoid((d0 - min_dist)/scale)`, `min_dist` = minimum
  of subsampled depth pixels and LiDAR points inside the arm-workspace box
  (depth box membership approximated by forward range — no extrinsics here);
* `grasp_uncertainty = 1 - ObjectPose.confidence` (top-1 SigLIP similarity);
* `action_entropy` mirrored from the latest ActionChunk;
* `tactile_anomaly`: z-score spike detection of the mean tactile pressure
  over a 1 s window (z > 3), floored at 0.5 while ContactState reports
  contacted-but-unstable fingers.

Decision (exact): `trigger_precise = (collision > 0.7) OR (uncertainty > 0.6)
OR (entropy > 0.4) OR (anomaly > 0.3 AND phase == grasp)`; overrides:
`slip_detected` True forces precise; `ContactState.grasp_stable` with
collision/uncertainty/entropy below thresholds forces fast.

### `inspire_hand_bridge_node` (regular Node, realtime CPU)

Bridges the Inspire hands (`inspire_protocol.py`) over one of two transports
(`transport` parameter):

* `modbus_tcp` (default, **RH56E2-T1**): Modbus TCP on the LAN, right
  `192.168.123.211` / left `192.168.123.210`, port 6000, unit 1. Register
  addresses are byte offsets (even addresses only). One 24-word read per tick
  returns `ANGLE_ACT` + `FORCE_ACT`: `joint_states.position` (normalized
  0 open..1 closed) and `joint_states.effort` (6 per-actuator forces, N,
  assuming gram-force raw units). The 1062-cell T1 tactile block is read by a
  worker thread per hand (own Modbus connection, `e2_tactile_rate_hz` = 30 Hz,
  `e2_tactile:=false` disables it) and published as `/tactile_raw` (all cells,
  `UInt16MultiArray`, region order of `E2_TACTILE_REGIONS`) and as TACTILE-15
  `/tactile` (sum per region, `e2_tactile_n_per_count` = UNCALIBRATED
  placeholder). Writes (`ANGLE_SET`, `FORCE_SET`, function 0x10) are
  unverified on hardware, so the node starts with `read_only: true` and force
  limits are hard-capped at the factory default raw value (2000).
* `serial` (**RH56DFQ**): 0xEB 0x90 frames, register-level encode/decode,
  checksum = low byte of the frame-body sum. Per hand at `tactile_rate_hz`
  (100 Hz) it reads per-phalanx forces + actual positions and publishes
  `/g1/hand/{side}/tactile` (15 pressures, placeholder normals `[0,0,1]`).

`/g1/hand/{side}/cmd` is clamped to `force_limit_n` and written to the
position/force registers (when `read_only` is false).

Hot-disconnect: serial failures close the port, publish an open-hand-safe
state, latch `/g1/hand/{side}/available = False` and retry every 2 s with
exponential backoff (cap 30 s); reconnect re-applies the last command and
republishes `available = True`. `/g1/estop` True holds the grip (commands
ignored, nothing released).

Mock mode (`mock:=true`, and required when pyserial is absent):
`MockInspireHand` simulates first-order tracking and synthetic tactile
pressure against a virtual object at 60 % closure. With `mock:=false` and no
pyserial the node logs fatal and raises at startup.

## Layout

```
g1_action/
  executor_core.py     # rclpy-free: ChunkBuffer, hand hybrid PI, nav P-control
  safety.py            # rclpy-free: five-stage SafetyPipeline, capsule distance
  blending.py          # rclpy-free: DualPathBlender
  risk_core.py         # rclpy-free: risk signals + dispatch decision
  inspire_protocol.py  # rclpy-free: RH56 serial frames, RH56E2 Modbus TCP, mock
  qos.py               # shared QoS profiles (sensor / command / latched)
  action_executor_node.py
  risk_estimator_node.py
  inspire_hand_bridge_node.py
test/                  # pytest, numpy only (no ROS required)
```

## Tests

```
cd src/g1_action
python -m pytest test -q
```

Covers: dual-encoder fault injection with ESTOP latching, velocity/torque
clamps, workspace block, capsule-distance correctness (parallel / crossing /
degenerate) and self-collision block; blend ramp length/midpoint/passthrough;
receding-horizon ordering, replan-overlap blend and HOLD-on-invalidation
(grip force retained); every risk trigger individually plus phase gating,
slip override and stable-grasp downgrade; Inspire frame round-trips, checksum
rejection, force clamping and mock-hand contact pressures.

## Kinematic / model approximations (documented)

* Wrist FK is a 3-segment planar chain (torso offset, upper arm, forearm)
  using shoulder_pitch, shoulder_roll and elbow_pitch only; shoulder yaw and
  wrist joints are ignored. Segment lengths and shoulder offsets are
  parameters (`upper_arm_m=0.28`, `forearm_m=0.26`,
  `shoulder_offset_y_m=0.15`, `shoulder_offset_z_m=1.1` above the base
  footprint so the default `workspace_bounds` z-range [0, 1.6] is meaningful).
* Self-collision uses three capsules (forearms radius 0.035 m, torso radius
  0.07 m spanning z 0.7..1.1 m) with a 0.01 m margin.
* Torque demand is a stiffness model (`stiffness_nm_per_rad=80`), not a full
  dynamics model; it bounds the commanded tracking error to
  `120 / 80 = 1.5 rad`.
