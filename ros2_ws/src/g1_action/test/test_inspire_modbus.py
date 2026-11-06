"""RH56E2 Modbus TCP transport tests (numpy + stdlib sockets, no rclpy, no hardware).

``FakeE2Server`` reproduces what was observed on the real hands: register
addresses are byte offsets, a read of N registers at address A returns the N
little-endian 16-bit words stored at bytes A, A+2, ..., and a read at an
address >= 6000 makes the hand drop the connection.
"""

import socket
import struct
import threading

import numpy as np
import pytest

from g1_action.inspire_protocol import (
    E2_FORCE_RAW_PER_NEWTON,
    E2_FORCE_SET_FACTORY_RAW,
    E2_REG_ANGLE_ACT,
    E2_REG_ANGLE_SET,
    E2_REG_FORCE_ACT,
    E2_REG_FORCE_SET,
    E2_TACTILE_BATCHES,
    E2_TACTILE_N_PER_COUNT_DEFAULT,
    E2_TACTILE_NUM_CELLS,
    E2_TACTILE_REGIONS,
    E2_TACTILE_SLICES,
    HandSample,
    InspireConnectionError,
    InspireModbusHand,
    MockInspireHand,
    ModbusError,
    build_modbus_read,
    build_modbus_write_multiple,
    decode_modbus_response,
    decode_modbus_words,
    e2_actuator_forces_n,
    e2_force_limit_raw,
    e2_position_raw,
    e2_tactile_phalanx_pressures,
    e2_tactile_regions,
    plan_tactile_batches,
)

NEWTON_PER_RAW = 1.0 / E2_FORCE_RAW_PER_NEWTON


class FakeE2Server:
    """Single-client Modbus TCP server with the observed RH56E2 quirks."""

    def __init__(self) -> None:
        self.mem = bytearray(6100)
        self.log = []  # ("read"|"write", address, count)
        self.exception_code = None
        self.mode = "normal"  # normal | silent | drop | bad_tid
        self._stop = threading.Event()
        self._listener = socket.socket()
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(1)
        self._listener.settimeout(0.1)
        self.port = self._listener.getsockname()[1]
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- memory helpers --------------------------------------------------------

    def set_words(self, address, values) -> None:
        for i, value in enumerate(values):
            a = address + 2 * i
            self.mem[a] = value & 0xFF
            self.mem[a + 1] = (value >> 8) & 0xFF

    def get_words(self, address, count):
        return [self._word(address + 2 * i) for i in range(count)]

    def _word(self, address) -> int:
        return self.mem[address] | (self.mem[address + 1] << 8)

    # -- server ------------------------------------------------------------------

    def close(self) -> None:
        self._stop.set()
        self._listener.close()
        self._thread.join(timeout=2.0)

    @staticmethod
    def _recv(conn, size) -> bytes:
        buf = bytearray()
        while len(buf) < size:
            chunk = conn.recv(size - len(buf))
            if not chunk:
                raise ConnectionError("client closed")
            buf += chunk
        return bytes(buf)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(0.5)
            try:
                self._serve(conn)
            except (OSError, ConnectionError):
                pass
            finally:
                conn.close()

    def _serve(self, conn) -> None:
        while not self._stop.is_set():
            header = self._recv(conn, 7)
            tid, _, length, unit = struct.unpack(">HHHB", header)
            body = self._recv(conn, length - 1)
            function = body[0]
            if function == 0x03:
                address, count = struct.unpack(">HH", body[1:5])
                self.log.append(("read", address, count))
            else:
                address, count, _ = struct.unpack(">HHB", body[1:6])
                words = struct.unpack(">%dH" % count, body[6 : 6 + 2 * count])
                self.log.append(("write", address, count))
            if self.mode == "silent":
                continue
            if self.mode == "drop" or (function == 0x03 and address >= 6000):
                return  # closes the connection
            if self.exception_code is not None:
                pdu = bytes([function | 0x80, self.exception_code])
            elif function == 0x03:
                data = b"".join(
                    struct.pack(">H", self._word(address + 2 * i)) for i in range(count)
                )
                pdu = bytes([0x03, 2 * count]) + data
            else:
                self.set_words(address, words)
                pdu = struct.pack(">BHH", 0x10, address, count)
            out_tid = (tid + 1) & 0xFFFF if self.mode == "bad_tid" else tid
            conn.sendall(struct.pack(">HHHB", out_tid, 0, 1 + len(pdu), unit) + pdu)


@pytest.fixture
def server():
    srv = FakeE2Server()
    srv.set_words(E2_REG_ANGLE_ACT, [0, 0, 0, 5, 0, 984])
    srv.set_words(E2_REG_FORCE_ACT, [108, 135, 110, 103, 65506, 101])
    yield srv
    srv.close()


@pytest.fixture
def hand(server):
    h = InspireModbusHand("127.0.0.1", port=server.port, timeout_s=0.5)
    h.connect()
    yield h
    h.close()


class TestFrames:
    def test_read_request_bytes(self):
        frame = build_modbus_read(1, 1, E2_REG_ANGLE_ACT, 6)
        assert frame == bytes.fromhex("0001000000060103060a0006")

    def test_write_request_bytes(self):
        frame = build_modbus_write_multiple(2, 1, E2_REG_ANGLE_SET, [1000, 0])
        assert frame == bytes.fromhex("0002000000" "0b" "01" "10" "05ce" "0002" "04" "03e80000")

    def test_read_count_limits(self):
        with pytest.raises(ModbusError):
            build_modbus_read(1, 1, 1546, 0)
        with pytest.raises(ModbusError):
            build_modbus_read(1, 1, 1546, 126)

    def test_decode_read_response(self):
        frame = bytes.fromhex("0007000000070103" "04" "00050010")
        payload = decode_modbus_response(frame, 7, 1, 0x03)
        assert decode_modbus_words(payload, 2) == [5, 16]

    def test_exception_response_carries_code(self):
        frame = bytes.fromhex("000700000003" "01" "83" "02")
        with pytest.raises(ModbusError) as exc:
            decode_modbus_response(frame, 7, 1, 0x03)
        assert exc.value.code == 2

    @pytest.mark.parametrize(
        "frame",
        [
            bytes.fromhex("0008000000070103" "04" "00050010"),  # wrong transaction
            bytes.fromhex("0007000000070203" "04" "00050010"),  # wrong unit
            bytes.fromhex("0007000100070103" "04" "00050010"),  # wrong protocol id
            bytes.fromhex("0007000000080103" "04" "00050010"),  # length mismatch
            bytes.fromhex("0007000000070104" "04" "00050010"),  # wrong function
        ],
    )
    def test_framing_errors_have_no_code(self, frame):
        with pytest.raises(ModbusError) as exc:
            decode_modbus_response(frame, 7, 1, 0x03)
        assert exc.value.code is None

    def test_bad_payload_length_rejected(self):
        with pytest.raises(ModbusError):
            decode_modbus_words(bytes([4, 0, 5]), 2)


class TestScaling:
    def test_position_raw_is_open_1000_closed_0(self):
        assert e2_position_raw([0.0, 0.5, 1.0, 0.25, 0.75, 2.0]) == [
            1000, 500, 0, 750, 250, 0,
        ]

    def test_force_limit_thumb_rotation_shares_thumb_limit(self):
        raw = e2_force_limit_raw([1.0, 2.5, 5.0, 10.0, 15.0], 15.0)
        assert len(raw) == 6
        assert raw[5] == raw[4]
        assert raw[0] == round(1.0 * E2_FORCE_RAW_PER_NEWTON)

    def test_force_limit_clamped_in_newtons_then_capped_in_raw(self):
        assert e2_force_limit_raw([100.0] * 5, 15.0) == [
            round(15.0 * E2_FORCE_RAW_PER_NEWTON)
        ] * 6
        # A wrong unit assumption still cannot exceed the factory default.
        assert e2_force_limit_raw([100.0] * 5, 1000.0) == [E2_FORCE_SET_FACTORY_RAW] * 6
        assert e2_force_limit_raw([-3.0] * 5, 15.0) == [0] * 6

    def test_actuator_forces_are_signed(self):
        forces = e2_actuator_forces_n([108, 65506, 0, 65535, 100, 1])
        assert forces == pytest.approx(
            np.array([108, -30, 0, -1, 100, 1]) * NEWTON_PER_RAW
        )


class TestHandAgainstFakeServer:
    def test_read_sample_decodes_position_and_force_in_one_read(self, hand, server):
        sample = hand.read_sample()
        assert isinstance(sample, HandSample)
        assert sample.tactile is None
        assert sample.position == pytest.approx(
            [1.0, 1.0, 1.0, 0.995, 1.0, 0.016]
        )
        assert sample.actuator_force == pytest.approx(
            np.array([108, 135, 110, 103, -30, 101]) * NEWTON_PER_RAW
        )
        assert server.log == [("read", E2_REG_ANGLE_ACT, 24)]

    def test_individual_reads(self, hand):
        assert hand.read_position() == pytest.approx([1, 1, 1, 0.995, 1, 0.016])
        assert hand.read_actuator_force()[4] == pytest.approx(-30 * NEWTON_PER_RAW)

    def test_every_request_uses_an_even_byte_address(self, hand, server):
        hand.read_sample()
        hand.read_position()
        hand.read_actuator_force()
        hand.set_position([0.5] * 6)
        hand.set_force_limit([5.0] * 5)
        assert server.log and all(addr % 2 == 0 for _, addr, _ in server.log)

    def test_connecting_never_writes(self, hand, server):
        hand.read_sample()
        assert all(kind == "read" for kind, _, _ in server.log)

    def test_set_position_writes_angle_set(self, hand, server):
        hand.set_position([1.0, 0.5, 0.0, 0.25, 0.75, 1.0])
        assert server.get_words(E2_REG_ANGLE_SET, 6) == [0, 500, 1000, 750, 250, 0]

    def test_set_force_limit_writes_force_set(self, hand, server):
        hand.set_force_limit([1.0, 2.5, 5.0, 10.0, 15.0])
        expected = e2_force_limit_raw([1.0, 2.5, 5.0, 10.0, 15.0], 15.0)
        assert server.get_words(E2_REG_FORCE_SET, 6) == expected

    def test_force_limit_never_exceeds_factory_default(self, server):
        h = InspireModbusHand("127.0.0.1", port=server.port, force_limit_n=1000.0)
        h.connect()
        try:
            h.set_force_limit([500.0] * 5)
        finally:
            h.close()
        assert server.get_words(E2_REG_FORCE_SET, 6) == [E2_FORCE_SET_FACTORY_RAW] * 6

    def test_exception_response_keeps_link_up(self, hand, server):
        server.exception_code = 2
        with pytest.raises(ModbusError) as exc:
            hand.read_position()
        assert exc.value.code == 2
        assert hand.connected
        server.exception_code = None
        assert hand.read_position() == pytest.approx([1, 1, 1, 0.995, 1, 0.016])

    def test_dropped_connection_then_reconnect(self, hand, server):
        server.mode = "drop"
        with pytest.raises(InspireConnectionError):
            hand.read_sample()
        assert not hand.connected
        with pytest.raises(InspireConnectionError):
            hand.read_sample()  # closed socket, no hidden retry
        server.mode = "normal"
        hand.connect()
        assert hand.read_position() == pytest.approx([1, 1, 1, 0.995, 1, 0.016])

    def test_timeout_closes_the_connection(self, server):
        server.mode = "silent"
        h = InspireModbusHand("127.0.0.1", port=server.port, timeout_s=0.05)
        h.connect()
        with pytest.raises(InspireConnectionError):
            h.read_sample()
        assert not h.connected

    def test_transaction_mismatch_resyncs_by_closing(self, hand, server):
        server.mode = "bad_tid"
        with pytest.raises(ModbusError) as exc:
            hand.read_position()
        assert exc.value.code is None
        assert not hand.connected

    def test_connect_refused(self):
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        h = InspireModbusHand("127.0.0.1", port=port)
        with pytest.raises(InspireConnectionError):
            h.connect()
        assert not h.connected

    def test_read_beyond_the_map_drops_the_link(self, hand):
        with pytest.raises(InspireConnectionError):
            hand._read(6000, 1)
        assert not hand.connected


class TestDeviceCapabilities:
    def test_modbus_hand_has_tactile_but_not_in_read_sample(self, server):
        assert InspireModbusHand.has_tactile is True
        hand = InspireModbusHand("127.0.0.1", server.port, timeout_s=0.5)
        hand.connect()
        sample = hand.read_sample()  # the fast loop never pays for the 11 tactile reads
        hand.close()
        assert sample.tactile is None and sample.tactile_raw is None
        assert sample.actuator_force is not None

    def test_mock_sample_has_tactile_and_no_actuator_force(self):
        hand = MockInspireHand("left")
        sample = hand.read_sample()
        assert MockInspireHand.has_tactile is True
        assert sample.tactile.shape == (15,)
        assert sample.position.shape == (6,)
        assert sample.actuator_force is None


# ---------------------------------------------------------------------------
# T1 tactile block
# ---------------------------------------------------------------------------


def _fill_tactile(srv, values_by_region):
    """Write ``{region name: flat list of counts}`` into the fake hand memory."""
    for region in E2_TACTILE_REGIONS:
        srv.set_words(region.address, values_by_region.get(region.name, [0] * region.count))


def test_tactile_region_table_is_consistent():
    assert len(E2_TACTILE_REGIONS) == 17
    assert E2_TACTILE_NUM_CELLS == 1062
    names = [r.name for r in E2_TACTILE_REGIONS]
    assert len(set(names)) == 17
    end_of_previous = 0
    for region in E2_TACTILE_REGIONS:
        assert region.address % 2 == 0  # odd addresses return byte-shifted garbage
        assert region.address >= end_of_previous  # sorted, no overlap
        end_of_previous = region.address + 2 * region.count
    assert end_of_previous < 6000  # reads at >= 6000 drop the connection
    assert sum(b - a for a, b in E2_TACTILE_SLICES.values()) == E2_TACTILE_NUM_CELLS


def test_tactile_batches_cover_every_region_once_within_modbus_limit():
    seen = []
    for address, words, members in E2_TACTILE_BATCHES:
        assert 1 <= words <= 125 and address % 2 == 0
        cursor = address
        for index in members:
            region = E2_TACTILE_REGIONS[index]
            assert region.address == cursor  # contiguous inside a batch
            cursor += 2 * region.count
            seen.append(index)
        assert cursor == address + 2 * words
    assert seen == list(range(len(E2_TACTILE_REGIONS)))
    assert len(E2_TACTILE_BATCHES) == 11


def test_plan_tactile_batches_never_exceeds_the_limit():
    plan = plan_tactile_batches(max_words=112)  # the palm (112 cells) is the largest region
    assert all(words <= 112 for _, words, _ in plan)
    assert sum(len(m) for _, _, m in plan) == len(E2_TACTILE_REGIONS)
    assert len(plan_tactile_batches(max_words=2000)) == 1  # the whole block is one contiguous run
    with pytest.raises(ValueError):
        plan_tactile_batches(max_words=100)  # regions are never split


def test_read_tactile_raw_maps_regions_and_only_reads(server):
    little_nail = list(range(1, 97))
    thumb_pad = [4000 + i for i in range(96)]
    palm = [7] * 112
    _fill_tactile(server, {"little_nail": little_nail, "thumb_pad": thumb_pad, "palm": palm})
    hand = InspireModbusHand("127.0.0.1", server.port, timeout_s=0.5)
    hand.connect()
    server.log.clear()
    raw = hand.read_tactile_raw()
    hand.close()
    assert raw.shape == (1062,) and raw.dtype == np.uint16
    regions = e2_tactile_regions(raw)
    assert regions["little_nail"].shape == (12, 8)
    assert regions["little_nail"].ravel().tolist() == little_nail
    assert regions["thumb_pad"].ravel().tolist() == thumb_pad
    assert (regions["palm"] == 7).all() and regions["palm"].shape == (8, 14)
    assert int(regions["ring_pad"].sum()) == 0
    assert len(server.log) == 11
    assert all(kind == "read" and address % 2 == 0 and address < 6000 and count <= 125
               for kind, address, count in server.log)


def test_phalanx_pressure_convention_and_scale():
    raw = np.zeros(E2_TACTILE_NUM_CELLS, dtype=np.uint16)

    def put(region, total):
        start, stop = E2_TACTILE_SLICES[region]
        raw[start] = total

    put("little_pad", 10)
    put("little_nail", 20)
    put("little_tip", 30)
    put("index_pad", 40)
    put("thumb_pad", 50)
    put("thumb_middle", 60)
    put("thumb_nail", 70)
    put("thumb_tip", 80)
    put("palm", 999)  # not part of TACTILE-15
    out = e2_tactile_phalanx_pressures(raw, n_per_count=0.5)
    assert out.shape == (15,)
    assert out[0:3].tolist() == [5.0, 10.0, 15.0]  # little: pad, nail, tip
    assert out[9:12].tolist() == [20.0, 0.0, 0.0]  # index
    assert out[12:15].tolist() == [25.0, 30.0, 75.0]  # thumb: pad, middle, nail+tip
    assert out[3:9].sum() == 0.0
    default = e2_tactile_phalanx_pressures(raw)
    assert default[0] == pytest.approx(10 * E2_TACTILE_N_PER_COUNT_DEFAULT)


def test_tactile_helpers_reject_wrong_shapes():
    with pytest.raises(ValueError):
        e2_tactile_regions(np.zeros(10))
    with pytest.raises(ValueError):
        e2_tactile_phalanx_pressures(np.zeros(1000))


def test_read_tactile_returns_tactile15_with_configured_scale(server):
    _fill_tactile(server, {"middle_pad": [100] * 80})
    hand = InspireModbusHand("127.0.0.1", server.port, timeout_s=0.5, tactile_n_per_count=0.01)
    hand.connect()
    tactile = hand.read_tactile()
    hand.close()
    assert tactile.shape == (15,)
    assert tactile[6] == pytest.approx(80 * 100 * 0.01)
    assert tactile.sum() == pytest.approx(tactile[6])


def test_tactile_read_failure_closes_the_link(server):
    hand = InspireModbusHand("127.0.0.1", server.port, timeout_s=0.3)
    hand.connect()
    server.mode = "drop"
    with pytest.raises(InspireConnectionError):
        hand.read_tactile_raw()
    assert not hand.connected
