"""Inspire RH56 protocol framing + mock hand tests (numpy only, no rclpy)."""

import numpy as np
import pytest

from g1_action.inspire_protocol import (
    CMD_READ,
    CMD_WRITE,
    ChecksumError,
    MockInspireHand,
    ProtocolError,
    REG_FORCE_ACT,
    REG_FORCE_LIMIT_SET,
    REG_POSITION_ACT,
    REG_POSITION_SET,
    bytes_to_uint16s,
    clamp_force_limits,
    decode_register_payload,
    encode_force_limit_cmd,
    encode_position_cmd,
    encode_read_request,
    encode_read_response,
    forces_from_raw,
    parse_frame,
    parse_response,
    positions_from_raw,
)


class TestFrameRoundTrip:
    def test_position_cmd_round_trip(self):
        positions = [0.0, 0.25, 0.5, 0.75, 1.0, 0.5]
        frame = encode_position_cmd(1, positions)
        parsed = parse_frame(frame)
        assert parsed.hand_id == 1
        assert parsed.cmd == CMD_WRITE
        register, data = decode_register_payload(parsed.payload)
        assert register == REG_POSITION_SET
        raw = bytes_to_uint16s(data)
        assert raw == [1000, 750, 500, 250, 0, 500]  # (1 - p) * 1000
        assert np.allclose(positions_from_raw(raw), positions)

    def test_force_limit_cmd_round_trip(self):
        frame = encode_force_limit_cmd(2, [1.0, 2.5, 5.0, 10.0, 15.0])
        parsed = parse_frame(frame)
        assert parsed.cmd == CMD_WRITE
        register, data = decode_register_payload(parsed.payload)
        assert register == REG_FORCE_LIMIT_SET
        raw = bytes_to_uint16s(data)
        assert raw == [100, 250, 500, 1000, 1500]  # centinewtons
        assert np.allclose(forces_from_raw(raw), [1.0, 2.5, 5.0, 10.0, 15.0])

    def test_read_request_round_trip(self):
        frame = encode_read_request(1, REG_POSITION_ACT, 12)
        parsed = parse_frame(frame)
        assert parsed.cmd == CMD_READ
        register, data = decode_register_payload(parsed.payload)
        assert register == REG_POSITION_ACT
        assert data == bytes([12])

    def test_read_response_round_trip(self):
        values = [0, 150, 1500, 65535, 42]
        frame = encode_read_response(1, REG_FORCE_ACT, values)
        response = parse_response(frame)
        assert response.hand_id == 1
        assert response.register == REG_FORCE_ACT
        assert response.uint16_values == values

    def test_frame_layout(self):
        frame = encode_read_request(3, REG_FORCE_ACT, 30)
        assert frame[:2] == b"\xeb\x90"  # header
        assert frame[2] == 3  # id
        assert frame[3] == 1 + 3  # len = cmd + payload bytes
        # checksum = low byte of sum(id, len, cmd, payload).
        assert frame[-1] == sum(frame[2:-1]) & 0xFF


class TestChecksumRejection:
    def test_corrupted_payload_byte_rejected(self):
        frame = bytearray(encode_position_cmd(1, [0.5] * 6))
        frame[-2] ^= 0xFF  # flip a payload byte, checksum now stale
        with pytest.raises(ChecksumError):
            parse_frame(bytes(frame))

    def test_corrupted_checksum_rejected(self):
        frame = bytearray(encode_read_request(1, REG_POSITION_ACT, 12))
        frame[-1] = (frame[-1] + 1) & 0xFF
        with pytest.raises(ChecksumError):
            parse_frame(bytes(frame))

    def test_bad_header_rejected(self):
        frame = bytearray(encode_read_request(1, REG_POSITION_ACT, 12))
        frame[0] = 0x00
        with pytest.raises(ProtocolError):
            parse_frame(bytes(frame))

    def test_truncated_frame_rejected(self):
        frame = encode_position_cmd(1, [0.5] * 6)
        with pytest.raises(ProtocolError):
            parse_frame(frame[:-3])

    def test_write_frame_is_not_a_read_response(self):
        with pytest.raises(ProtocolError):
            parse_response(encode_position_cmd(1, [0.0] * 6))


class TestForceClamp:
    def test_clamp_force_limits(self):
        clamped = clamp_force_limits([20.0, -3.0, 7.5, 15.0001, 0.0], 15.0)
        assert np.allclose(clamped, [15.0, 0.0, 7.5, 15.0, 0.0])

    def test_encode_applies_safety_clamp(self):
        frame = encode_force_limit_cmd(1, [50.0] * 5, limit_n=15.0)
        _, data = decode_register_payload(parse_frame(frame).payload)
        assert bytes_to_uint16s(data) == [1500] * 5  # 15 N in centinewtons


class TestMockInspireHand:
    def test_contact_pressure_when_closing_on_virtual_object(self):
        hand = MockInspireHand("left", force_limit_n=15.0)
        hand.set_force_limit([10.0] * 5)
        hand.set_position([1.0] * 6)  # fully close
        for _ in range(500):
            hand.step(0.01)

        tactile = hand.read_tactile()
        assert tactile.shape == (15,)
        per_finger = tactile.reshape(5, 3).sum(axis=1)
        # All five fingers press on the virtual object at 60 % closure...
        assert np.all(per_finger > 1.0)
        # ...capped by the commanded per-finger force limit.
        assert np.all(per_finger <= 10.0 + 1e-9)
        # Distal phalanx carries the largest share.
        assert np.all(tactile[2::3] >= tactile[0::3])

    def test_positions_saturate_at_object(self):
        hand = MockInspireHand("right")
        hand.set_position([1.0] * 6)
        for _ in range(500):
            hand.step(0.01)
        pos = hand.read_position()
        # Flex fingers stop just past the object closure (0.6 + compliance).
        assert np.all(pos[:5] <= 0.6 + 0.08 + 1e-9)
        assert np.all(pos[:5] > 0.6)
        # Thumb rotation is not blocked by the object.
        assert pos[5] == pytest.approx(1.0, abs=1e-3)

    def test_open_hand_has_no_pressure(self):
        hand = MockInspireHand("left")
        hand.set_position([1.0] * 6)
        for _ in range(500):
            hand.step(0.01)
        hand.set_position([0.0] * 6)
        for _ in range(500):
            hand.step(0.01)
        assert np.allclose(hand.read_tactile(), 0.0)
        assert np.all(hand.read_position()[:5] < 0.05)

    def test_force_limit_clamped_to_hand_maximum(self):
        hand = MockInspireHand("left", force_limit_n=15.0)
        hand.set_force_limit([100.0] * 5)
        hand.set_position([1.0] * 6)
        for _ in range(500):
            hand.step(0.01)
        per_finger = hand.read_tactile().reshape(5, 3).sum(axis=1)
        assert np.all(per_finger <= 15.0 + 1e-9)

    def test_no_object_means_no_tactile(self):
        hand = MockInspireHand("left", object_present=False)
        hand.set_position([1.0] * 6)
        for _ in range(500):
            hand.step(0.01)
        assert np.allclose(hand.read_tactile(), 0.0)
        assert hand.read_position()[0] == pytest.approx(1.0, abs=1e-3)
