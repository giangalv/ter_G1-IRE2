"""Register-level Inspire RH56 protocols + mock hand (rclpy-free).

Two transports share one device API (``connect``/``close``/``set_position``/
``set_force_limit``/``read_sample``):

* :class:`InspireSerialHand` -- RH56DFQ serial frames (described below);
* :class:`InspireModbusHand` -- RH56E2 Modbus TCP on the G1 LAN (see the
  "Modbus TCP transport" section at the end of this module).

Serial frame layout (both directions)::

    0xEB 0x90 | id | len | cmd | payload ... | checksum

* ``len``       -- number of bytes from ``cmd`` through the end of payload
                   (i.e. ``1 + len(payload)``).
* ``checksum``  -- low byte of the sum of ``id``, ``len``, ``cmd`` and every
                   payload byte (header excluded).
* register payloads start with the 16-bit register address, little-endian;
  register write payloads then carry little-endian uint16 values, read
  requests carry a single byte count, read responses carry the raw bytes.

Register map used by this workspace:

* ``REG_POSITION_SET`` -- 6x uint16, 0..1000. Raw 1000 = fully open,
  raw 0 = fully closed; normalized HandCmd position ``p`` in [0..1]
  (open -> closed) maps to ``raw = round((1 - p) * 1000)``.
* ``REG_FORCE_LIMIT_SET`` -- 5x uint16 per-finger force limit, centinewtons.
* ``REG_POSITION_ACT``  -- 6x uint16 actual positions (same scaling).
* ``REG_FORCE_ACT``     -- 15x uint16 per-phalanx forces, centinewtons
  (TACTILE-15 order: finger-major little..thumb, phalanx proximal->distal).

``pyserial`` is a guarded import: :data:`HAS_SERIAL` reports availability
and :class:`MockInspireHand` provides a deterministic simulation (used when
the bridge runs with ``mock=True`` or pyserial is missing).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import math
import socket
import struct

import numpy as np

try:  # Guarded heavy/host-specific dependency (see CONVENTIONS.md).
    import serial  # type: ignore

    HAS_SERIAL = True
except ImportError:  # pragma: no cover - exercised only without pyserial
    serial = None  # type: ignore[assignment]
    HAS_SERIAL = False

HEADER = b"\xeb\x90"

CMD_READ = 0x11
CMD_WRITE = 0x12

REG_POSITION_SET = 0x05CE
REG_FORCE_LIMIT_SET = 0x05DA
REG_POSITION_ACT = 0x060A
REG_FORCE_ACT = 0x062E

POSITION_RAW_MAX = 1000
#: raw force units are centinewtons: newtons = raw / FORCE_RAW_PER_NEWTON.
FORCE_RAW_PER_NEWTON = 100.0

NUM_DOF = 6
NUM_FINGERS = 5
NUM_TACTILE = 15


class ProtocolError(ValueError):
    """Malformed Inspire frame (header, length or structure)."""


class ChecksumError(ProtocolError):
    """Frame failed checksum validation."""


class InspireConnectionError(RuntimeError):
    """Transport failure (open, timeout, disconnect)."""


@dataclass(frozen=True)
class HandSample:
    """One poll of a hand, independent of the transport.

    Attributes:
        position: (6,) normalized positions, [0..1] open -> closed.
        tactile: (15,) TACTILE-15 per-phalanx pressures (N), or ``None`` when
            this poll carries no tactile data.
        actuator_force: (6,) per-actuator force (N) in HAND-12 joint order,
            or ``None`` when the transport does not report it.
        tactile_raw: (1062,) raw RH56E2-T1 cell counts (0..4095) in
            :data:`E2_TACTILE_REGIONS` order, or ``None``.
    """

    position: np.ndarray
    tactile: Optional[np.ndarray] = None
    actuator_force: Optional[np.ndarray] = None
    tactile_raw: Optional[np.ndarray] = None


# ---------------------------------------------------------------------------
# Frame primitives
# ---------------------------------------------------------------------------


def _checksum(body: bytes) -> int:
    """Low byte of the sum over ``id``..``payload``."""
    return sum(body) & 0xFF


def build_frame(hand_id: int, cmd: int, payload: bytes) -> bytes:
    """Assemble a full frame with header and checksum."""
    if not 0 <= hand_id <= 0xFF:
        raise ProtocolError(f"hand id {hand_id} out of range")
    if len(payload) > 0xFE:
        raise ProtocolError(f"payload too long ({len(payload)} bytes)")
    body = bytes([hand_id, len(payload) + 1, cmd & 0xFF]) + payload
    return HEADER + body + bytes([_checksum(body)])


@dataclass(frozen=True)
class Frame:
    """A validated Inspire frame."""

    hand_id: int
    cmd: int
    payload: bytes


def parse_frame(data: bytes) -> Frame:
    """Validate and decode one frame.

    Raises:
        ProtocolError: bad header, truncated data or length mismatch.
        ChecksumError: checksum validation failed.
    """
    data = bytes(data)
    if len(data) < 6:
        raise ProtocolError(f"frame too short ({len(data)} bytes)")
    if data[:2] != HEADER:
        raise ProtocolError(f"bad header {data[:2].hex()}")
    hand_id = data[2]
    length = data[3]
    expected = 4 + length + 1  # header(2) + id + len | cmd+payload | checksum
    if len(data) != expected:
        raise ProtocolError(
            f"frame length mismatch: len field {length} implies "
            f"{expected} bytes, got {len(data)}"
        )
    body = data[2:-1]
    if _checksum(body) != data[-1]:
        raise ChecksumError(
            f"checksum mismatch: computed {_checksum(body):#04x}, "
            f"frame carries {data[-1]:#04x}"
        )
    return Frame(hand_id=hand_id, cmd=data[4], payload=data[5:-1])


def uint16s_to_bytes(values: Sequence[int]) -> bytes:
    """Pack uint16 values little-endian, clamped to [0, 65535]."""
    out = bytearray()
    for v in values:
        raw = min(max(int(round(v)), 0), 0xFFFF)
        out += bytes([raw & 0xFF, (raw >> 8) & 0xFF])
    return bytes(out)


def bytes_to_uint16s(data: bytes) -> List[int]:
    """Unpack little-endian uint16 values."""
    if len(data) % 2 != 0:
        raise ProtocolError(f"odd uint16 payload length {len(data)}")
    return [data[i] | (data[i + 1] << 8) for i in range(0, len(data), 2)]


def decode_register_payload(payload: bytes) -> tuple:
    """Split a register payload into ``(register, data_bytes)``."""
    if len(payload) < 2:
        raise ProtocolError("register payload shorter than address")
    return payload[0] | (payload[1] << 8), payload[2:]


# ---------------------------------------------------------------------------
# Command encoding
# ---------------------------------------------------------------------------


def clamp_force_limits(
    forces_n: Sequence[float], limit_n: float
) -> np.ndarray:
    """Clamp per-finger force limits to ``[0, limit_n]`` newtons."""
    return np.clip(
        np.asarray(forces_n, dtype=np.float64), 0.0, float(limit_n)
    )


def encode_position_cmd(hand_id: int, positions: Sequence[float]) -> bytes:
    """Write frame for 6 normalized positions ([0..1] open -> closed)."""
    pos = np.clip(np.asarray(positions, dtype=np.float64).reshape(NUM_DOF), 0.0, 1.0)
    raw = [(1.0 - p) * POSITION_RAW_MAX for p in pos]
    payload = bytes([REG_POSITION_SET & 0xFF, (REG_POSITION_SET >> 8) & 0xFF])
    payload += uint16s_to_bytes(raw)
    return build_frame(hand_id, CMD_WRITE, payload)


def encode_force_limit_cmd(
    hand_id: int, forces_n: Sequence[float], limit_n: Optional[float] = None
) -> bytes:
    """Write frame for 5 per-finger force limits (newtons).

    Args:
        forces_n: 5 per-finger limits in newtons.
        limit_n: optional safety clamp applied before encoding.
    """
    forces = np.asarray(forces_n, dtype=np.float64).reshape(NUM_FINGERS)
    if limit_n is not None:
        forces = clamp_force_limits(forces, limit_n)
    forces = np.maximum(forces, 0.0)
    payload = bytes(
        [REG_FORCE_LIMIT_SET & 0xFF, (REG_FORCE_LIMIT_SET >> 8) & 0xFF]
    )
    payload += uint16s_to_bytes(forces * FORCE_RAW_PER_NEWTON)
    return build_frame(hand_id, CMD_WRITE, payload)


def encode_read_request(hand_id: int, register: int, num_bytes: int) -> bytes:
    """Read-request frame for ``num_bytes`` starting at ``register``."""
    if not 0 < num_bytes <= 0xFF:
        raise ProtocolError(f"read length {num_bytes} out of range")
    payload = bytes(
        [register & 0xFF, (register >> 8) & 0xFF, num_bytes & 0xFF]
    )
    return build_frame(hand_id, CMD_READ, payload)


def encode_read_response(
    hand_id: int, register: int, values_raw: Sequence[int]
) -> bytes:
    """Read-response frame carrying uint16 register values (device -> host)."""
    payload = bytes([register & 0xFF, (register >> 8) & 0xFF])
    payload += uint16s_to_bytes(values_raw)
    return build_frame(hand_id, CMD_READ, payload)


@dataclass(frozen=True)
class ReadResponse:
    """Decoded read-response: register address plus raw data bytes."""

    hand_id: int
    register: int
    data: bytes

    @property
    def uint16_values(self) -> List[int]:
        """Payload interpreted as little-endian uint16 values."""
        return bytes_to_uint16s(self.data)


def parse_response(data: bytes) -> ReadResponse:
    """Parse and checksum-validate a read-response frame.

    Raises:
        ProtocolError / ChecksumError: malformed or corrupted frame.
    """
    frame = parse_frame(data)
    if frame.cmd != CMD_READ:
        raise ProtocolError(
            f"expected read response (cmd {CMD_READ:#04x}), got {frame.cmd:#04x}"
        )
    register, payload = decode_register_payload(frame.payload)
    return ReadResponse(hand_id=frame.hand_id, register=register, data=payload)


def positions_from_raw(raw: Sequence[int]) -> np.ndarray:
    """Raw 0..1000 register positions -> normalized [0..1] open -> closed."""
    arr = np.asarray(raw, dtype=np.float64)
    return np.clip(1.0 - arr / POSITION_RAW_MAX, 0.0, 1.0)


def forces_from_raw(raw: Sequence[int]) -> np.ndarray:
    """Raw centinewton register forces -> newtons."""
    return np.asarray(raw, dtype=np.float64) / FORCE_RAW_PER_NEWTON


# ---------------------------------------------------------------------------
# Mock hand (mock=True or pyserial missing)
# ---------------------------------------------------------------------------


class MockInspireHand:
    """Deterministic RH56DFQ simulation for mock mode.

    * First-order position tracking with time constant ``time_constant_s``.
    * A virtual object sits at ``object_closure`` (60 %) of the flex range:
      reported flex positions saturate slightly past it (compliance) and the
      finger builds pressure proportional to the commanded overshoot, capped
      by the per-finger force limit.
    * Tactile output is TACTILE-15 (per-phalanx, proximal -> distal weights
      0.2 / 0.3 / 0.5 of the fingertip force).
    """

    has_tactile = True

    def __init__(
        self,
        side: str = "left",
        time_constant_s: float = 0.12,
        object_closure: float = 0.6,
        object_compliance: float = 0.08,
        grip_stiffness_n: float = 40.0,
        force_limit_n: float = 15.0,
        object_present: bool = True,
    ) -> None:
        self.side = side
        self._tau = max(float(time_constant_s), 1e-3)
        self._object_closure = float(object_closure)
        self._object_compliance = float(object_compliance)
        self._grip_stiffness_n = float(grip_stiffness_n)
        self._max_force_n = float(force_limit_n)
        self.object_present = bool(object_present)
        self._target = np.zeros(NUM_DOF, dtype=np.float64)
        self._pos = np.zeros(NUM_DOF, dtype=np.float64)
        self._force_limit = np.full(
            NUM_FINGERS, float(force_limit_n), dtype=np.float64
        )
        self._connected = True

    # -- transport-parity API (matches InspireSerialHand) -------------------

    @property
    def connected(self) -> bool:
        """Always True: the mock never disconnects."""
        return self._connected

    def connect(self) -> None:
        """No-op (interface parity with the serial transport)."""
        self._connected = True

    def close(self) -> None:
        """No-op (interface parity with the serial transport)."""

    # -- commands ------------------------------------------------------------

    def set_position(self, positions: Sequence[float]) -> None:
        """Command 6 normalized positions ([0..1] open -> closed)."""
        self._target = np.clip(
            np.asarray(positions, dtype=np.float64).reshape(NUM_DOF), 0.0, 1.0
        )

    def set_force_limit(self, forces_n: Sequence[float]) -> None:
        """Command 5 per-finger force limits (newtons, clamped)."""
        self._force_limit = clamp_force_limits(
            np.asarray(forces_n, dtype=np.float64).reshape(NUM_FINGERS),
            self._max_force_n,
        )

    def step(self, dt: float) -> None:
        """Advance the first-order actuator model by ``dt`` seconds."""
        alpha = 1.0 - math.exp(-max(float(dt), 0.0) / self._tau)
        self._pos += (self._target - self._pos) * alpha

    # -- reads ----------------------------------------------------------------

    def read_sample(self) -> HandSample:
        """Positions + synthetic tactile (no actuator-force channel)."""
        return HandSample(position=self.read_position(), tactile=self.read_tactile())

    def read_position(self) -> np.ndarray:
        """(6,) reported normalized positions (flex capped by the object)."""
        pos = self._pos.copy()
        if self.object_present:
            cap = self._object_closure + self._object_compliance
            pos[:NUM_FINGERS] = np.minimum(pos[:NUM_FINGERS], cap)
        return pos

    def read_tactile(self) -> np.ndarray:
        """(15,) synthetic per-phalanx pressures in newtons."""
        tactile = np.zeros(NUM_TACTILE, dtype=np.float64)
        if not self.object_present:
            return tactile
        weights = np.array([0.2, 0.3, 0.5])
        for finger in range(NUM_FINGERS):
            overshoot = max(0.0, self._pos[finger] - self._object_closure)
            force = min(
                overshoot * self._grip_stiffness_n, self._force_limit[finger]
            )
            tactile[3 * finger : 3 * finger + 3] = force * weights
        return tactile


# ---------------------------------------------------------------------------
# Serial transport
# ---------------------------------------------------------------------------


class InspireSerialHand:
    """Serial transport for one RH56DFQ hand.

    All I/O errors surface as :class:`InspireConnectionError` so the bridge
    node can run its hot-disconnect/reconnect policy; corrupted frames
    surface as :class:`ProtocolError` / :class:`ChecksumError`.
    """

    has_tactile = True

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        hand_id: int = 1,
        timeout_s: float = 0.02,
    ) -> None:
        """
        Args:
            port: serial device (e.g. ``/dev/inspire_left``).
            baudrate: line rate (RH56 default 115200).
            hand_id: protocol id of the hand on this bus.
            timeout_s: per-transaction read timeout.
        """
        if not HAS_SERIAL:
            raise InspireConnectionError(
                "pyserial is not installed; run the bridge with mock:=true"
            )
        self._port = port
        self._baudrate = int(baudrate)
        self._hand_id = int(hand_id)
        self._timeout_s = float(timeout_s)
        self._serial: Optional["serial.Serial"] = None

    @property
    def connected(self) -> bool:
        """True while the port is open."""
        return self._serial is not None and bool(self._serial.is_open)

    def connect(self) -> None:
        """Open the serial port (idempotent)."""
        if self.connected:
            return
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout_s,
                write_timeout=self._timeout_s,
            )
        except Exception as exc:  # serial.SerialException, OSError, ...
            self._serial = None
            raise InspireConnectionError(
                f"cannot open {self._port}: {exc}"
            ) from exc

    def close(self) -> None:
        """Close the port, swallowing shutdown errors."""
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # pragma: no cover - best-effort shutdown
                pass
            self._serial = None

    # -- commands ------------------------------------------------------------

    def set_position(self, positions: Sequence[float]) -> None:
        """Write 6 normalized positions to ``REG_POSITION_SET``."""
        self._write(encode_position_cmd(self._hand_id, positions))

    def set_force_limit(self, forces_n: Sequence[float]) -> None:
        """Write 5 per-finger force limits to ``REG_FORCE_LIMIT_SET``."""
        self._write(encode_force_limit_cmd(self._hand_id, forces_n))

    # -- reads ----------------------------------------------------------------

    def read_sample(self) -> HandSample:
        """Positions + per-phalanx tactile (two serial transactions)."""
        return HandSample(position=self.read_position(), tactile=self.read_tactile())

    def read_position(self) -> np.ndarray:
        """(6,) normalized actual positions from ``REG_POSITION_ACT``."""
        response = self._transact(REG_POSITION_ACT, NUM_DOF * 2)
        return positions_from_raw(response.uint16_values)

    def read_tactile(self) -> np.ndarray:
        """(15,) per-phalanx forces (N) from ``REG_FORCE_ACT``."""
        response = self._transact(REG_FORCE_ACT, NUM_TACTILE * 2)
        return forces_from_raw(response.uint16_values)

    # -- low-level -------------------------------------------------------------

    def _require_port(self) -> "serial.Serial":
        if self._serial is None or not self._serial.is_open:
            raise InspireConnectionError(f"{self._port} is not open")
        return self._serial

    def _write(self, frame: bytes) -> None:
        port = self._require_port()
        try:
            port.write(frame)
            # Discard any pending ack bytes so reads stay frame-aligned.
            if port.in_waiting:
                port.read(port.in_waiting)
        except Exception as exc:
            raise InspireConnectionError(
                f"write failed on {self._port}: {exc}"
            ) from exc

    def _transact(self, register: int, num_bytes: int) -> ReadResponse:
        port = self._require_port()
        request = encode_read_request(self._hand_id, register, num_bytes)
        expected = 6 + 2 + num_bytes  # frame overhead + addr + data
        try:
            port.write(request)
            raw = port.read(expected)
        except Exception as exc:
            raise InspireConnectionError(
                f"transaction failed on {self._port}: {exc}"
            ) from exc
        if len(raw) != expected:
            raise InspireConnectionError(
                f"short read on {self._port}: {len(raw)}/{expected} bytes"
            )
        return parse_response(raw)


# ---------------------------------------------------------------------------
# Modbus TCP transport (RH56E2-2R/2L-T1 on the G1 LAN)
# ---------------------------------------------------------------------------
#
# Observed on the real hands (G1 LAN: right 192.168.123.211, left
# 192.168.123.210, TCP 6000, unit id 1):
#
# * Register addresses are BYTE offsets. A read of N registers at address A
#   returns the N little-endian 16-bit words stored at bytes A, A+2, ...; the
#   words of a vendor block (ANGLE_ACT, FORCE_ACT, ...) therefore sit at
#   A, A+2, A+4. Reads at an odd offset return byte-shifted mixes of two
#   neighbouring words, so every request here uses an even address.
# * ANGLE_* are 0..1000 with 0 = closed, 1000 = open (same as the serial map).
# * FORCE_ACT holds 6 signed values, one per actuator (little, ring, middle,
#   index, thumb bend, thumb rotation) -- NOT 15 per-phalanx pressures. The
#   T1 tactile block (1062 cells, 17 regions, byte addresses 3000..5123) is
#   described in the "T1 tactile block" section below.
# * Reads at an address >= 6000 make the hand drop the TCP connection.
#
# NOT verified on hardware: the unit of the force registers (assumed
# gram-force, factory FORCE_SET = 2000) and the write path (function 0x10).
# Writes are therefore capped at the factory default raw value, and the
# bridge starts in read-only mode.

MODBUS_DEFAULT_PORT = 6000
MODBUS_UNIT_ID = 1
FN_READ_HOLDING = 0x03
FN_WRITE_MULTIPLE = 0x10
_MBAP_LEN = 7
_MODBUS_MAX_READ_WORDS = 125
_MODBUS_MAX_WRITE_WORDS = 123

E2_REG_ANGLE_SET = 1486
E2_REG_FORCE_SET = 1498
E2_REG_ANGLE_ACT = 1546
E2_REG_FORCE_ACT = 1582

#: Assumed unit of the force registers: raw = gram-force.
E2_FORCE_RAW_PER_NEWTON = 1.0 / 0.00980665
#: Factory FORCE_SET value read from both hands; writes never exceed it.
E2_FORCE_SET_FACTORY_RAW = 2000
#: One read covers ANGLE_ACT .. FORCE_ACT (24 words) instead of two requests.
_E2_SAMPLE_WORDS = (E2_REG_FORCE_ACT - E2_REG_ANGLE_ACT) // 2 + NUM_DOF


class ModbusError(ProtocolError):
    """Modbus exception response (``code`` set) or malformed frame."""

    def __init__(self, message: str, code: Optional[int] = None) -> None:
        super().__init__(message)
        self.code = code


def build_modbus_read(
    transaction_id: int, unit_id: int, address: int, count: int
) -> bytes:
    """Read-holding-registers request (function 0x03)."""
    if not 1 <= count <= _MODBUS_MAX_READ_WORDS:
        raise ModbusError(f"read count {count} out of range")
    if not 0 <= address <= 0xFFFF:
        raise ModbusError(f"address {address} out of range")
    pdu = struct.pack(">BHH", FN_READ_HOLDING, address, count)
    mbap = struct.pack(">HHHB", transaction_id & 0xFFFF, 0, 1 + len(pdu), unit_id)
    return mbap + pdu


def build_modbus_write_multiple(
    transaction_id: int, unit_id: int, address: int, values: Sequence[int]
) -> bytes:
    """Write-multiple-registers request (function 0x10), values clamped to u16."""
    count = len(values)
    if not 1 <= count <= _MODBUS_MAX_WRITE_WORDS:
        raise ModbusError(f"write count {count} out of range")
    if not 0 <= address <= 0xFFFF:
        raise ModbusError(f"address {address} out of range")
    words = [min(max(int(v), 0), 0xFFFF) for v in values]
    pdu = struct.pack(">BHHB", FN_WRITE_MULTIPLE, address, count, 2 * count)
    pdu += struct.pack(">%dH" % count, *words)
    mbap = struct.pack(">HHHB", transaction_id & 0xFFFF, 0, 1 + len(pdu), unit_id)
    return mbap + pdu


def decode_modbus_response(
    frame: bytes, transaction_id: int, unit_id: int, function: int
) -> bytes:
    """Validate a full response frame and return the bytes after the function code.

    Raises:
        ModbusError: ``code`` is set for a well-formed exception response and
            ``None`` for any framing problem (bad protocol id, length,
            transaction or unit mismatch, unexpected function).
    """
    if len(frame) < _MBAP_LEN + 2:
        raise ModbusError(f"modbus frame too short ({len(frame)} bytes)")
    tid, proto, length, unit = struct.unpack(">HHHB", frame[:_MBAP_LEN])
    if proto != 0:
        raise ModbusError(f"unexpected modbus protocol id {proto}")
    if len(frame) != 6 + length:
        raise ModbusError(
            f"modbus length mismatch: field {length} implies {6 + length} "
            f"bytes, got {len(frame)}"
        )
    if tid != (transaction_id & 0xFFFF):
        raise ModbusError(
            f"modbus transaction mismatch: sent {transaction_id & 0xFFFF}, got {tid}"
        )
    if unit != unit_id:
        raise ModbusError(f"modbus unit mismatch: expected {unit_id}, got {unit}")
    got = frame[_MBAP_LEN]
    if got == (function | 0x80):
        code = frame[_MBAP_LEN + 1]
        raise ModbusError(f"modbus exception {code} for function {function:#04x}", code)
    if got != function:
        raise ModbusError(f"unexpected modbus function {got:#04x}")
    return frame[_MBAP_LEN + 1 :]


def decode_modbus_words(payload: bytes, count: int) -> List[int]:
    """Read-response payload (byte count + data) -> ``count`` big-endian u16."""
    if len(payload) != 1 + 2 * count or payload[0] != 2 * count:
        raise ModbusError(
            f"read payload carries {len(payload) - 1} data bytes, "
            f"expected {2 * count}"
        )
    return list(struct.unpack(">%dH" % count, payload[1:]))


def e2_position_raw(positions: Sequence[float]) -> List[int]:
    """6 normalized positions ([0..1] open -> closed) -> raw 1000..0."""
    pos = np.clip(np.asarray(positions, dtype=np.float64).reshape(NUM_DOF), 0.0, 1.0)
    return [int(round((1.0 - p) * POSITION_RAW_MAX)) for p in pos]


def e2_force_limit_raw(
    forces_n: Sequence[float],
    limit_n: float,
    raw_cap: int = E2_FORCE_SET_FACTORY_RAW,
) -> List[int]:
    """5 per-finger limits (N) -> 6 raw FORCE_SET words, hard-capped.

    The thumb-rotation actuator reuses the thumb limit. The result is clamped
    to ``[0, limit_n]`` newtons first and then to ``raw_cap`` raw units, so a
    wrong force-unit assumption can never raise a limit above the factory
    default.
    """
    per_finger = clamp_force_limits(
        np.asarray(forces_n, dtype=np.float64).reshape(NUM_FINGERS), limit_n
    )
    per_dof = np.concatenate([per_finger, per_finger[-1:]])
    raw = np.round(per_dof * E2_FORCE_RAW_PER_NEWTON)
    return [int(min(max(v, 0), int(raw_cap))) for v in raw]


def e2_actuator_forces_n(raw_words: Sequence[int]) -> np.ndarray:
    """6 raw FORCE_ACT words (signed int16) -> newtons."""
    raw = np.asarray(raw_words, dtype=np.int64).reshape(NUM_DOF)
    signed = np.where(raw > 0x7FFF, raw - 0x10000, raw).astype(np.float64)
    return signed / E2_FORCE_RAW_PER_NEWTON


# ---------------------------------------------------------------------------
# RH56E2-T1 tactile block
# ---------------------------------------------------------------------------
#
# 17 regions of 16-bit cells (0..4095, one register per cell, 0 = no contact)
# at byte addresses 3000..5123. The layout is the one of Inspire's RH56DFTP
# manual (section 2.6.20, as transcribed by the third-party ``inspire-rh56dftp``
# driver): per finger a 3x3 "tip", a 12x8 "nail" and a 10x8 "pad" region, the
# thumb has tip (3x3), nail (12x8), middle (3x3) and pad (12x8), and the palm is
# 8x14. Checked against the real RH56E2-2R-T1: while the right little finger
# was pulled only ``little_nail`` (92/96 cells) and ``little_pad`` (79/80 cells)
# moved, up to 1438 counts, and every other region of both hands stayed ~0.
#
# NOT verified: which physical part of the finger each region covers (the
# TACTILE-15 aggregation below is a convention) and the force per count.

E2_TACTILE_MAX_RAW = 4095
#: UNCALIBRATED placeholder: newtons per summed cell count. Only a rough
#: contact/no-contact scale; calibrate with a known weight before trusting it.
E2_TACTILE_N_PER_COUNT_DEFAULT = 0.001


@dataclass(frozen=True)
class TactileRegion:
    """One rectangular tactile region of the RH56E2-T1."""

    name: str
    address: int  # byte address of the first cell
    rows: int
    cols: int

    @property
    def count(self) -> int:
        """Number of cells (= registers)."""
        return self.rows * self.cols


E2_TACTILE_REGIONS: Tuple[TactileRegion, ...] = (
    TactileRegion("little_tip", 3000, 3, 3),
    TactileRegion("little_nail", 3018, 12, 8),
    TactileRegion("little_pad", 3210, 10, 8),
    TactileRegion("ring_tip", 3370, 3, 3),
    TactileRegion("ring_nail", 3388, 12, 8),
    TactileRegion("ring_pad", 3580, 10, 8),
    TactileRegion("middle_tip", 3740, 3, 3),
    TactileRegion("middle_nail", 3758, 12, 8),
    TactileRegion("middle_pad", 3950, 10, 8),
    TactileRegion("index_tip", 4110, 3, 3),
    TactileRegion("index_nail", 4128, 12, 8),
    TactileRegion("index_pad", 4320, 10, 8),
    TactileRegion("thumb_tip", 4480, 3, 3),
    TactileRegion("thumb_nail", 4498, 12, 8),
    TactileRegion("thumb_middle", 4690, 3, 3),
    TactileRegion("thumb_pad", 4708, 12, 8),
    TactileRegion("palm", 4900, 8, 14),
)
E2_TACTILE_NUM_CELLS = sum(r.count for r in E2_TACTILE_REGIONS)  # 1062
#: region name -> (start, stop) slice inside the flat (1062,) raw vector.
E2_TACTILE_SLICES: Dict[str, Tuple[int, int]] = {}
_offset = 0
for _region in E2_TACTILE_REGIONS:
    E2_TACTILE_SLICES[_region.name] = (_offset, _offset + _region.count)
    _offset += _region.count
del _offset, _region


def plan_tactile_batches(
    regions: Sequence[TactileRegion] = E2_TACTILE_REGIONS,
    max_words: int = _MODBUS_MAX_READ_WORDS,
) -> List[Tuple[int, int, Tuple[int, ...]]]:
    """Group address-contiguous regions into the fewest Modbus reads.

    Returns ``(byte_address, words, region_indices)`` per read; a region is
    never split and a read never exceeds ``max_words`` registers.
    """
    batches: List[Tuple[int, int, List[int]]] = []
    for index, region in enumerate(regions):
        if region.count > max_words:
            raise ValueError(
                f"region {region.name} ({region.count} words) exceeds {max_words}"
            )
        if batches:
            address, words, members = batches[-1]
            contiguous = region.address == address + 2 * words
            if contiguous and words + region.count <= max_words:
                batches[-1] = (address, words + region.count, members + [index])
                continue
        batches.append((region.address, region.count, [index]))
    return [(a, w, tuple(m)) for a, w, m in batches]


E2_TACTILE_BATCHES = plan_tactile_batches()


def e2_tactile_regions(raw: Sequence[int]) -> Dict[str, np.ndarray]:
    """Flat (1062,) raw vector -> ``{region name: (rows, cols) array}``."""
    flat = np.asarray(raw)
    if flat.shape != (E2_TACTILE_NUM_CELLS,):
        raise ValueError(f"expected {E2_TACTILE_NUM_CELLS} cells, got {flat.shape}")
    return {
        r.name: flat[slice(*E2_TACTILE_SLICES[r.name])].reshape(r.rows, r.cols)
        for r in E2_TACTILE_REGIONS
    }


def e2_tactile_phalanx_pressures(
    raw: Sequence[int], n_per_count: float = E2_TACTILE_N_PER_COUNT_DEFAULT
) -> np.ndarray:
    """Flat raw cells -> TACTILE-15 per-phalanx pressures (N).

    Convention (finger-major little..thumb, proximal -> distal): for the four
    fingers ``pad`` -> proximal, ``nail`` -> middle, ``tip`` -> distal; for the
    thumb ``pad`` -> proximal, ``middle`` -> middle, ``nail + tip`` -> distal.
    Each value is the SUM of the cell counts of its region(s) times
    ``n_per_count`` (an uncalibrated placeholder, see
    :data:`E2_TACTILE_N_PER_COUNT_DEFAULT`).
    """
    flat = np.asarray(raw, dtype=np.float64)
    if flat.shape != (E2_TACTILE_NUM_CELLS,):
        raise ValueError(f"expected {E2_TACTILE_NUM_CELLS} cells, got {flat.shape}")

    def total(*names: str) -> float:
        return float(sum(flat[slice(*E2_TACTILE_SLICES[n])].sum() for n in names))

    out = np.zeros(NUM_TACTILE, dtype=np.float64)
    for finger, name in enumerate(("little", "ring", "middle", "index")):
        out[3 * finger] = total(f"{name}_pad")
        out[3 * finger + 1] = total(f"{name}_nail")
        out[3 * finger + 2] = total(f"{name}_tip")
    out[12] = total("thumb_pad")
    out[13] = total("thumb_middle")
    out[14] = total("thumb_nail", "thumb_tip")
    return out * float(n_per_count)


class InspireModbusHand:
    """Modbus TCP transport for one RH56E2 hand.

    Same device API as :class:`InspireSerialHand`. Every transport failure
    surfaces as :class:`InspireConnectionError` (the socket is closed so the
    bridge's reconnect policy applies); well-formed Modbus exceptions surface
    as :class:`ModbusError` and keep the link up. Opening the connection never
    writes to the hand.
    """

    has_tactile = True

    def __init__(
        self,
        host: str,
        port: int = MODBUS_DEFAULT_PORT,
        unit_id: int = MODBUS_UNIT_ID,
        timeout_s: float = 0.02,
        connect_timeout_s: float = 0.3,
        force_limit_n: float = 15.0,
        force_raw_cap: int = E2_FORCE_SET_FACTORY_RAW,
        tactile_n_per_count: float = E2_TACTILE_N_PER_COUNT_DEFAULT,
    ) -> None:
        """
        Args:
            host: hand IP (right 192.168.123.211, left 192.168.123.210).
            port: Modbus TCP port of the hand.
            unit_id: Modbus unit/slave id.
            timeout_s: per-transaction timeout.
            connect_timeout_s: TCP connect timeout.
            force_limit_n: safety clamp for force-limit writes (N).
            force_raw_cap: raw ceiling for force-limit writes.
            tactile_n_per_count: newtons per summed tactile count (uncalibrated).
        """
        self._host = str(host)
        self._port = int(port)
        self._unit_id = int(unit_id)
        self._timeout_s = float(timeout_s)
        self._connect_timeout_s = float(connect_timeout_s)
        self._force_limit_n = float(force_limit_n)
        self._force_raw_cap = int(force_raw_cap)
        self._tactile_n_per_count = float(tactile_n_per_count)
        self._sock: Optional[socket.socket] = None
        self._tid = 0

    @property
    def connected(self) -> bool:
        """True while the TCP connection is open."""
        return self._sock is not None

    def connect(self) -> None:
        """Open the TCP connection (idempotent, no register access)."""
        if self._sock is not None:
            return
        try:
            sock = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout_s
            )
        except OSError as exc:
            raise InspireConnectionError(
                f"cannot connect to {self._host}:{self._port}: {exc}"
            ) from exc
        sock.settimeout(self._timeout_s)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:  # pragma: no cover - best-effort latency tweak
            pass
        self._sock = sock

    def close(self) -> None:
        """Close the connection, swallowing shutdown errors."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:  # pragma: no cover - best-effort shutdown
                pass
            self._sock = None

    # -- commands ------------------------------------------------------------

    def set_position(self, positions: Sequence[float]) -> None:
        """Write 6 normalized positions to ``ANGLE_SET``."""
        self._write(E2_REG_ANGLE_SET, e2_position_raw(positions))

    def set_force_limit(self, forces_n: Sequence[float]) -> None:
        """Write the 5 per-finger limits (thumb rotation shares the thumb's)."""
        self._write(
            E2_REG_FORCE_SET,
            e2_force_limit_raw(forces_n, self._force_limit_n, self._force_raw_cap),
        )

    # -- reads ----------------------------------------------------------------

    def read_sample(self) -> HandSample:
        """Positions + actuator forces in ONE 24-word read (no tactile).

        The 1062-cell tactile block takes several more requests; read it with
        :meth:`read_tactile_raw` at its own, lower rate.
        """
        words = self._read(E2_REG_ANGLE_ACT, _E2_SAMPLE_WORDS)
        force_start = (E2_REG_FORCE_ACT - E2_REG_ANGLE_ACT) // 2
        return HandSample(
            position=positions_from_raw(words[:NUM_DOF]),
            tactile=None,
            actuator_force=e2_actuator_forces_n(
                words[force_start : force_start + NUM_DOF]
            ),
        )

    def read_position(self) -> np.ndarray:
        """(6,) normalized actual positions from ``ANGLE_ACT``."""
        return positions_from_raw(self._read(E2_REG_ANGLE_ACT, NUM_DOF))

    def read_actuator_force(self) -> np.ndarray:
        """(6,) signed per-actuator forces (N) from ``FORCE_ACT``."""
        return e2_actuator_forces_n(self._read(E2_REG_FORCE_ACT, NUM_DOF))

    def read_tactile_raw(self) -> np.ndarray:
        """(1062,) raw tactile cells (uint16, 0..4095) in region order.

        One Modbus read per entry of :data:`E2_TACTILE_BATCHES`.
        """
        flat = np.zeros(E2_TACTILE_NUM_CELLS, dtype=np.uint16)
        for address, words, members in E2_TACTILE_BATCHES:
            values = self._read(address, words)
            cursor = 0
            for index in members:
                region = E2_TACTILE_REGIONS[index]
                start, stop = E2_TACTILE_SLICES[region.name]
                flat[start:stop] = values[cursor : cursor + region.count]
                cursor += region.count
        return flat

    def read_tactile(self) -> np.ndarray:
        """(15,) TACTILE-15 per-phalanx pressures (N), see
        :func:`e2_tactile_phalanx_pressures` (uncalibrated scale)."""
        return e2_tactile_phalanx_pressures(
            self.read_tactile_raw(), self._tactile_n_per_count
        )

    # -- low-level -------------------------------------------------------------

    def _next_tid(self) -> int:
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid

    def _read(self, address: int, count: int) -> List[int]:
        tid = self._next_tid()
        frame = build_modbus_read(tid, self._unit_id, address, count)
        payload = self._roundtrip(frame, tid, FN_READ_HOLDING)
        try:
            return decode_modbus_words(payload, count)
        except ModbusError:
            self.close()  # inconsistent payload: resynchronize by reconnecting
            raise

    def _write(self, address: int, values: Sequence[int]) -> None:
        tid = self._next_tid()
        frame = build_modbus_write_multiple(tid, self._unit_id, address, values)
        payload = self._roundtrip(frame, tid, FN_WRITE_MULTIPLE)
        if payload != struct.pack(">HH", address, len(values)):
            self.close()
            raise ModbusError("write echo does not match the request")

    def _recv_exact(self, sock: socket.socket, size: int) -> bytes:
        buf = bytearray()
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                raise ConnectionError("connection closed by the hand")
            buf += chunk
        return bytes(buf)

    def _roundtrip(self, frame: bytes, tid: int, function: int) -> bytes:
        if self._sock is None:
            raise InspireConnectionError(f"{self._host}:{self._port} is not open")
        sock = self._sock
        try:
            sock.sendall(frame)
            header = self._recv_exact(sock, _MBAP_LEN)
            length = struct.unpack(">H", header[4:6])[0]
            if not 2 <= length <= 254:
                self.close()
                raise ModbusError(f"implausible modbus length {length}")
            body = self._recv_exact(sock, length - 1)
        except (socket.timeout, OSError) as exc:
            self.close()
            raise InspireConnectionError(
                f"modbus transaction failed on {self._host}:{self._port}: {exc!r}"
            ) from exc
        try:
            return decode_modbus_response(header + body, tid, self._unit_id, function)
        except ModbusError as exc:
            if exc.code is None:  # framing error: stream may be out of sync
                self.close()
            raise
