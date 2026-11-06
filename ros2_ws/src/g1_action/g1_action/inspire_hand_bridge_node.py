"""``inspire_hand_bridge_node``: Inspire RH56 hand <-> ROS 2 bridge.

Thin ROS 2 shell around :mod:`g1_action.inspire_protocol`. Two transports
(``transport`` parameter): ``modbus_tcp`` (RH56E2 on the G1 LAN, default) and
``serial`` (RH56DFQ). Per hand:

* 100 Hz poll: read actual positions (+ per-phalanx forces on the serial
  transport, per-actuator forces on Modbus), publish
  ``/g1/hand/{side}/joint_states`` (normalized [0..1] positions; ``effort`` =
  per-actuator force in N when the transport reports it) and
  ``/g1/hand/{side}/tactile`` (TactileArray, 15 pressures + placeholder
  normals ``[0, 0, 1]``). On the RH56E2-T1 the 1062-cell tactile block is read
  by one worker thread per hand on its own Modbus connection (the hand accepts
  two clients) at ``e2_tactile_rate_hz`` (a full read is ~19 ms): the cells go
  out unchanged as ``/g1/hand/{side}/tactile_raw`` (UInt16MultiArray, region
  order of ``inspire_protocol.E2_TACTILE_REGIONS``) and aggregated into the 15
  TACTILE-15 pressures with the UNCALIBRATED scale ``e2_tactile_n_per_count``.
* Subscribes ``/g1/hand/{side}/cmd``: clamps force limits to
  ``force_limit_n`` and writes position + force registers. With
  ``read_only:=true`` (default on real hardware) commands are logged and
  dropped; the write path of the RH56E2 is not verified on hardware yet.
* Hot-disconnect handling: on a serial exception the port is closed, an
  open-hand-safe state is published, ``/g1/hand/{side}/available`` (latched
  Bool) goes False and reconnection is retried every 2 s with exponential
  backoff; on reconnect the last command is re-applied and available=True.
* ``/g1/estop``: on True the hands hold their current grip (incoming
  commands are ignored, the last position is re-asserted; grip is NOT
  released).

Mock mode (``mock=True``): :class:`MockInspireHand` simulates first-order
position tracking and synthetic tactile pressure (a virtual object at 60 %
closure). If ``mock`` is False and pyserial is missing, startup fails fatally
(CONVENTIONS.md mock rules).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, MultiArrayDimension, UInt16MultiArray

from g1_interfaces.msg import HandCmd, TactileArray

from g1_action.inspire_protocol import (
    HAS_SERIAL,
    HandSample,
    InspireConnectionError,
    InspireModbusHand,
    InspireSerialHand,
    E2_TACTILE_N_PER_COUNT_DEFAULT,
    E2_TACTILE_NUM_CELLS,
    MODBUS_DEFAULT_PORT,
    MockInspireHand,
    NUM_DOF,
    NUM_FINGERS,
    NUM_TACTILE,
    ProtocolError,
    clamp_force_limits,
    e2_tactile_phalanx_pressures,
)
from g1_action.qos import COMMAND_QOS, LATCHED_QOS, SENSOR_QOS

_SIDES = ("left", "right")
_JOINT_NAMES = (
    "little_flex",
    "ring_flex",
    "middle_flex",
    "index_flex",
    "thumb_flex",
    "thumb_rotation",
)
_RECONNECT_INITIAL_S = 2.0
_RECONNECT_MAX_S = 30.0
_TRANSPORTS = ("modbus_tcp", "serial")


class _E2TactileWorker(threading.Thread):
    """Tactile reader of one RH56E2 hand on its own Modbus connection.

    Keeps the 100 Hz position loop free of the 11 requests a full tactile read
    needs. A failed read closes the link and retries with exponential backoff
    (2 s -> 30 s), independently of the main link.
    """

    def __init__(
        self,
        node: "InspireHandBridgeNode",
        side: str,
        host: str,
        port: int,
        unit_id: int,
        timeout_s: float,
        rate_hz: float,
        n_per_count: float,
    ) -> None:
        super().__init__(name=f"e2_tactile_{side}", daemon=True)
        self._node = node
        self._side = side
        self._host = host
        self._port = port
        self._unit_id = unit_id
        self._timeout_s = timeout_s
        self._period_s = 1.0 / max(rate_hz, 1e-3)
        self._n_per_count = n_per_count
        self.stop_event = threading.Event()

    def run(self) -> None:
        log = self._node.get_logger()
        device: Optional[InspireModbusHand] = None
        backoff = _RECONNECT_INITIAL_S
        while not self.stop_event.is_set():
            started = time.monotonic()
            if device is None:
                try:
                    device = InspireModbusHand(
                        host=self._host,
                        port=self._port,
                        unit_id=self._unit_id,
                        timeout_s=self._timeout_s,
                    )
                    device.connect()
                    backoff = _RECONNECT_INITIAL_S
                except InspireConnectionError as exc:
                    device = None
                    log.warning(
                        f"{self._side} tactile link: {exc}; retry in {backoff:.0f} s",
                        throttle_duration_sec=10.0,
                    )
                    self.stop_event.wait(backoff)
                    backoff = min(backoff * 2.0, _RECONNECT_MAX_S)
                    continue
            try:
                raw = device.read_tactile_raw()
            except (InspireConnectionError, OSError) as exc:
                device.close()
                device = None
                log.warning(
                    f"{self._side} tactile link failure: {exc}; retry in {backoff:.0f} s",
                    throttle_duration_sec=10.0,
                )
                self.stop_event.wait(backoff)
                backoff = min(backoff * 2.0, _RECONNECT_MAX_S)
                continue
            except ProtocolError as exc:
                log.warning(
                    f"{self._side} tactile: dropped corrupt read ({exc})",
                    throttle_duration_sec=1.0,
                )
                continue
            if not self.stop_event.is_set():
                try:
                    self._node.publish_e2_tactile(self._side, raw, self._n_per_count)
                except Exception as exc:  # publisher dies when the context shuts down
                    if not rclpy.ok():
                        break
                    log.error(f"{self._side} tactile publish failed: {exc}")
            self.stop_event.wait(max(self._period_s - (time.monotonic() - started), 0.0))
        if device is not None:
            device.close()


@dataclass
class _HandState:
    """Bridge-side bookkeeping for one hand."""

    side: str
    endpoint: str  # serial device path or "host:port"
    device: Optional[object] = None
    last_position: Optional[np.ndarray] = None
    last_force: Optional[np.ndarray] = None
    backoff_s: float = _RECONNECT_INITIAL_S
    next_attempt_s: float = 0.0
    normals: list = field(default_factory=list)


class InspireHandBridgeNode(Node):
    """Dual Inspire RH56 bridge: Modbus TCP (E2) or serial (DFQ)."""

    def __init__(self) -> None:
        super().__init__("inspire_hand_bridge_node")

        # -- parameters ------------------------------------------------------
        self.declare_parameter("transport", "modbus_tcp")
        self.declare_parameter("host_left", "192.168.123.210")
        self.declare_parameter("host_right", "192.168.123.211")
        self.declare_parameter("tcp_port", MODBUS_DEFAULT_PORT)
        self.declare_parameter("tcp_timeout_s", 0.02)
        self.declare_parameter("read_only", True)
        self.declare_parameter("e2_tactile", True)
        self.declare_parameter("e2_tactile_rate_hz", 30.0)
        self.declare_parameter("e2_tactile_n_per_count", E2_TACTILE_N_PER_COUNT_DEFAULT)
        self.declare_parameter("serial_port_left", "/dev/inspire_left")
        self.declare_parameter("serial_port_right", "/dev/inspire_right")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("tactile_rate_hz", 100.0)
        self.declare_parameter("force_limit_n", 15.0)
        self.declare_parameter("mock", False)
        self.declare_parameter("hand_id", 1)

        gp = self.get_parameter
        self._baudrate = int(gp("baudrate").value)
        self._rate_hz = float(gp("tactile_rate_hz").value)
        self._dt = 1.0 / max(self._rate_hz, 1e-6)
        self._force_limit_n = float(gp("force_limit_n").value)
        self._hand_id = int(gp("hand_id").value)
        self._mock = bool(gp("mock").value)
        self._transport = str(gp("transport").value)
        self._tcp_port = int(gp("tcp_port").value)
        self._tcp_timeout_s = float(gp("tcp_timeout_s").value)
        self._read_only = bool(gp("read_only").value)
        self._estop_hold = False
        self._e2_tactile = (
            self._transport == "modbus_tcp"
            and not self._mock
            and bool(gp("e2_tactile").value)
        )
        self._e2_tactile_rate_hz = float(gp("e2_tactile_rate_hz").value)
        self._e2_n_per_count = float(gp("e2_tactile_n_per_count").value)
        self._tactile_workers: Dict[str, _E2TactileWorker] = {}

        if self._transport not in _TRANSPORTS:
            self.get_logger().fatal(
                f"unknown transport '{self._transport}' (expected one of {_TRANSPORTS})"
            )
            raise ValueError(f"unknown transport '{self._transport}'")
        if not self._mock and self._transport == "serial" and not HAS_SERIAL:
            self.get_logger().fatal(
                "pyserial is not installed and transport=serial with mock=False; "
                "refusing to start (set mock:=true for sim mode)"
            )
            raise RuntimeError("pyserial missing and mock=False")

        # Tactile comes from the mock, the serial DFQ and, on the RH56E2-T1,
        # from the dedicated per-hand workers (``e2_tactile``).
        self._has_tactile = (
            self._mock or self._transport == "serial" or self._e2_tactile
        )

        if self._transport == "modbus_tcp":
            endpoints = {
                "left": f"{gp('host_left').value}:{self._tcp_port}",
                "right": f"{gp('host_right').value}:{self._tcp_port}",
            }
        else:
            endpoints = {
                "left": str(gp("serial_port_left").value),
                "right": str(gp("serial_port_right").value),
            }
        placeholder_normals = [0.0, 0.0, 1.0] * NUM_TACTILE
        self._state: Dict[str, _HandState] = {
            side: _HandState(
                side=side, endpoint=endpoints[side], normals=placeholder_normals
            )
            for side in _SIDES
        }

        # -- publishers ------------------------------------------------------
        self._tactile_pubs = {
            side: self.create_publisher(
                TactileArray, f"/g1/hand/{side}/tactile", SENSOR_QOS
            )
            for side in _SIDES
        }
        self._tactile_raw_pubs = {}
        if self._e2_tactile:
            self._tactile_raw_pubs = {
                side: self.create_publisher(
                    UInt16MultiArray, f"/g1/hand/{side}/tactile_raw", SENSOR_QOS
                )
                for side in _SIDES
            }
        self._joint_pubs = {
            side: self.create_publisher(
                JointState, f"/g1/hand/{side}/joint_states", SENSOR_QOS
            )
            for side in _SIDES
        }
        self._available_pubs = {
            side: self.create_publisher(
                Bool, f"/g1/hand/{side}/available", LATCHED_QOS
            )
            for side in _SIDES
        }

        # -- subscriptions -----------------------------------------------------
        for side in _SIDES:
            self.create_subscription(
                HandCmd,
                f"/g1/hand/{side}/cmd",
                lambda msg, s=side: self._on_cmd(s, msg),
                COMMAND_QOS,
            )
        self.create_subscription(Bool, "/g1/estop", self._on_estop, LATCHED_QOS)

        # -- devices ------------------------------------------------------------
        for side in _SIDES:
            self._open_device(side, initial=True)

        # -- timers ------------------------------------------------------------
        for side in _SIDES:
            self.create_timer(self._dt, lambda s=side: self._poll(s))

        # -- RH56E2 tactile workers ---------------------------------------------
        if self._e2_tactile:
            for side in _SIDES:
                host = self._state[side].endpoint.rpartition(":")[0]
                worker = _E2TactileWorker(
                    node=self,
                    side=side,
                    host=host,
                    port=self._tcp_port,
                    unit_id=self._hand_id,
                    timeout_s=max(self._tcp_timeout_s, 0.1),
                    rate_hz=self._e2_tactile_rate_hz,
                    n_per_count=self._e2_n_per_count,
                )
                self._tactile_workers[side] = worker
                worker.start()

        self.get_logger().info(
            f"inspire_hand_bridge_node up: mock={self._mock}, "
            f"transport={self._transport}, "
            f"left={self._state['left'].endpoint}, "
            f"right={self._state['right'].endpoint}, "
            f"{self._rate_hz:.0f} Hz, force limit {self._force_limit_n} N, "
            f"read_only={self._read_only}, tactile={self._has_tactile}"
            + (
                f" (RH56E2-T1 cells at {self._e2_tactile_rate_hz:.0f} Hz, "
                f"{self._e2_n_per_count:g} N/count UNCALIBRATED)"
                if self._e2_tactile
                else ""
            )
        )
        if not self._mock and self._read_only:
            self.get_logger().warning(
                "read_only:=true -> hand commands are dropped; the bridge only "
                "reads. Set read_only:=false to allow writes to the hands."
            )
        if not self._mock and not self._has_tactile:
            self.get_logger().warning(
                "tactile is disabled (e2_tactile:=false): "
                "/g1/hand/*/tactile is NOT published (executor falls back to "
                "position control)"
            )

    # ------------------------------------------------------------------
    # Device lifecycle
    # ------------------------------------------------------------------

    def _open_device(self, side: str, initial: bool = False) -> bool:
        """Create and connect the transport for one hand."""
        st = self._state[side]
        if self._mock:
            st.device = MockInspireHand(
                side=side, force_limit_n=self._force_limit_n
            )
            self._publish_available(side, True)
            self.get_logger().info(f"{side} hand: mock device ready")
            return True
        try:
            if self._transport == "modbus_tcp":
                host, _, port = st.endpoint.rpartition(":")
                device = InspireModbusHand(
                    host=host,
                    port=int(port),
                    unit_id=self._hand_id,
                    timeout_s=self._tcp_timeout_s,
                    force_limit_n=self._force_limit_n,
                )
            else:
                device = InspireSerialHand(
                    port=st.endpoint, baudrate=self._baudrate, hand_id=self._hand_id
                )
            device.connect()
        except InspireConnectionError as exc:
            st.device = None
            self._publish_available(side, False)
            level = self.get_logger().error if initial else self.get_logger().warning
            level(f"{side} hand: cannot open {st.endpoint}: {exc}")
            st.next_attempt_s = time.monotonic() + st.backoff_s
            return False
        st.device = device
        st.backoff_s = _RECONNECT_INITIAL_S
        self._publish_available(side, True)
        self.get_logger().info(f"{side} hand connected on {st.endpoint}")
        self._reapply_command(side)
        return True

    def _handle_disconnect(self, side: str, error: Exception) -> None:
        """Hot-disconnect: close, publish safe state, schedule reconnect."""
        st = self._state[side]
        self.get_logger().error(
            f"{side} hand link failure: {error}; closing the link, will retry "
            f"in {st.backoff_s:.0f} s"
        )
        if st.device is not None:
            try:
                st.device.close()
            except Exception:  # pragma: no cover - best-effort close
                pass
        st.device = None
        self._publish_available(side, False)
        # Open-hand-safe reported state while the hand is gone.
        self._publish_state(
            side,
            HandSample(
                position=np.zeros(NUM_DOF, dtype=np.float64),
                tactile=(
                    np.zeros(NUM_TACTILE, dtype=np.float64)
                    if self._has_tactile
                    else None
                ),
            ),
        )
        st.next_attempt_s = time.monotonic() + st.backoff_s

    def _maybe_reconnect(self, side: str) -> None:
        st = self._state[side]
        if time.monotonic() < st.next_attempt_s:
            return
        if not self._open_device(side):
            st.backoff_s = min(st.backoff_s * 2.0, _RECONNECT_MAX_S)
            st.next_attempt_s = time.monotonic() + st.backoff_s

    def _reapply_command(self, side: str) -> None:
        """Re-send the last command after a reconnect."""
        st = self._state[side]
        if st.device is None:
            return
        try:
            if st.last_force is not None:
                st.device.set_force_limit(st.last_force)
            if st.last_position is not None:
                st.device.set_position(st.last_position)
        except (InspireConnectionError, ProtocolError, OSError) as exc:
            self._handle_disconnect(side, exc)

    # ------------------------------------------------------------------
    # 100 Hz poll
    # ------------------------------------------------------------------

    def _poll(self, side: str) -> None:
        st = self._state[side]
        if st.device is None:
            self._maybe_reconnect(side)
            return
        try:
            if self._mock:
                st.device.step(self._dt)  # type: ignore[union-attr]
            sample = st.device.read_sample()  # type: ignore[union-attr]
        except (InspireConnectionError, OSError) as exc:
            self._handle_disconnect(side, exc)
            return
        except ProtocolError as exc:
            # Corrupted frame: drop this cycle, keep the link up.
            self.get_logger().warning(
                f"{side} hand: dropped corrupt frame ({exc})",
                throttle_duration_sec=1.0,
            )
            return
        self._publish_state(side, sample)

    def _publish_state(self, side: str, sample: HandSample) -> None:
        now = self.get_clock().now().to_msg()

        joints = JointState()
        joints.header.stamp = now
        joints.name = [f"{side}_{name}" for name in _JOINT_NAMES]
        joints.position = [
            float(v) for v in np.asarray(sample.position).reshape(NUM_DOF)
        ]
        if sample.actuator_force is not None:
            joints.effort = [
                float(v) for v in np.asarray(sample.actuator_force).reshape(NUM_DOF)
            ]
        self._joint_pubs[side].publish(joints)

        if sample.tactile is None:
            return  # this poll carries no tactile data: publish nothing, not zeros
        tact = TactileArray()
        tact.header.stamp = now
        tact.hand_side = side
        tact.pressure = [
            float(v) for v in np.asarray(sample.tactile).reshape(NUM_TACTILE)
        ]
        tact.contact_normal = list(self._state[side].normals)
        self._tactile_pubs[side].publish(tact)

    def publish_e2_tactile(self, side: str, raw: np.ndarray, n_per_count: float) -> None:
        """Publish one RH56E2-T1 frame (called from the tactile worker thread)."""
        now = self.get_clock().now().to_msg()
        tact = TactileArray()
        tact.header.stamp = now
        tact.hand_side = side
        tact.pressure = [
            float(v) for v in e2_tactile_phalanx_pressures(raw, n_per_count)
        ]
        tact.contact_normal = list(self._state[side].normals)
        self._tactile_pubs[side].publish(tact)

        cells = UInt16MultiArray()
        cells.layout.dim = [
            MultiArrayDimension(
                label="cells", size=E2_TACTILE_NUM_CELLS, stride=E2_TACTILE_NUM_CELLS
            )
        ]
        cells.data = [int(v) for v in raw]
        self._tactile_raw_pubs[side].publish(cells)

    def destroy_node(self) -> None:
        """Stop the tactile workers before tearing the node down."""
        for worker in self._tactile_workers.values():
            worker.stop_event.set()
        for worker in self._tactile_workers.values():
            worker.join(timeout=1.0)
        super().destroy_node()

    def _publish_available(self, side: str, available: bool) -> None:
        msg = Bool()
        msg.data = bool(available)
        self._available_pubs[side].publish(msg)

    # ------------------------------------------------------------------
    # Commands / estop
    # ------------------------------------------------------------------

    def _on_cmd(self, side: str, msg: HandCmd) -> None:
        if self._read_only and not self._mock:
            self.get_logger().warning(
                f"{side} hand: command dropped (read_only:=true)",
                throttle_duration_sec=5.0,
            )
            return
        if self._estop_hold:
            self.get_logger().warning(
                f"{side} hand: command ignored while ESTOP hold is active",
                throttle_duration_sec=1.0,
            )
            return
        if len(msg.position) != NUM_DOF or len(msg.force_limit) != NUM_FINGERS:
            self.get_logger().error(
                f"{side} hand: malformed HandCmd "
                f"({len(msg.position)} positions, {len(msg.force_limit)} forces)"
            )
            return
        position = np.clip(
            np.asarray(msg.position, dtype=np.float64), 0.0, 1.0
        )
        force = clamp_force_limits(msg.force_limit, self._force_limit_n)
        st = self._state[side]
        st.last_position = position
        st.last_force = force
        if st.device is None:
            return  # applied on reconnect
        try:
            st.device.set_force_limit(force)  # type: ignore[union-attr]
            st.device.set_position(position)  # type: ignore[union-attr]
        except (InspireConnectionError, ProtocolError, OSError) as exc:
            self._handle_disconnect(side, exc)

    def _on_estop(self, msg: Bool) -> None:
        """On ESTOP: hold the current grip (do NOT release)."""
        if msg.data and not self._estop_hold:
            self._estop_hold = True
            self.get_logger().warning(
                "ESTOP: holding hand positions, ignoring new commands"
            )
            for side in _SIDES:
                st = self._state[side]
                if st.device is None or st.last_position is None:
                    continue
                try:
                    st.device.set_position(st.last_position)  # type: ignore[union-attr]
                except (InspireConnectionError, ProtocolError, OSError) as exc:
                    self._handle_disconnect(side, exc)
        elif not msg.data and self._estop_hold:
            self._estop_hold = False
            self.get_logger().info("ESTOP released: accepting hand commands")


def main(args: Optional[list] = None) -> None:
    """Entry point."""
    rclpy.init(args=args)
    node = InspireHandBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # the SIGINT/SIGTERM handler may have shut the context down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
