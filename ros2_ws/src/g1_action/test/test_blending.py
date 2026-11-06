"""Tests for dual-path FAST <-> PRECISE blending (numpy only, no rclpy)."""

import numpy as np
import pytest

from g1_action.blending import DualPathBlender
from g1_action.executor_core import MODE_FAST, MODE_PRECISE


def test_alpha_ramp_length_equals_blend_window_steps():
    window = 25
    blender = DualPathBlender(window)
    fast = np.zeros(4)
    precise = np.ones(4)

    blender.notify_mode(MODE_FAST)
    assert np.allclose(blender.step(fast, None), fast)

    blender.notify_mode(MODE_PRECISE)
    outputs = [blender.step(precise, fast)[0] for _ in range(window + 5)]

    # The ramp is exactly `window` ticks: linspace(0, 1, window).
    assert np.allclose(outputs[:window], np.linspace(0.0, 1.0, window))
    # After the ramp the output is fully the new path.
    assert all(v == pytest.approx(1.0) for v in outputs[window:])


def test_midpoint_is_average_of_both_paths():
    window = 25  # odd window has an exact midpoint tick
    blender = DualPathBlender(window)
    fast = np.full(3, 2.0)
    precise = np.full(3, 4.0)

    blender.notify_mode(MODE_FAST)
    blender.step(fast, None)
    blender.notify_mode(MODE_PRECISE)
    outputs = [blender.step(precise, fast) for _ in range(window)]
    midpoint = outputs[window // 2]
    assert np.allclose(midpoint, (fast + precise) / 2.0)


def test_passthrough_when_single_path_available():
    blender = DualPathBlender(10)
    precise = np.array([0.1, -0.2, 0.3])
    blender.notify_mode(MODE_FAST)
    blender.step(np.zeros(3), None)
    blender.notify_mode(MODE_PRECISE)
    # No last chunk of the other mode: pass through even during the ramp.
    for _ in range(5):
        assert np.allclose(blender.step(precise, None), precise)


def test_no_blending_without_mode_switch():
    blender = DualPathBlender(10)
    fast = np.array([1.0, 2.0])
    other = np.array([-5.0, -5.0])
    blender.notify_mode(MODE_FAST)
    for _ in range(20):
        # Even with the other path available, no switch -> no blending.
        assert np.allclose(blender.step(fast, other), fast)
    assert not blender.blending


def test_switch_back_and_forth_restarts_ramp():
    window = 4
    blender = DualPathBlender(window)
    fast = np.zeros(1)
    precise = np.ones(1)

    blender.notify_mode(MODE_FAST)
    blender.step(fast, None)

    blender.notify_mode(MODE_PRECISE)
    ramp_up = [blender.step(precise, fast)[0] for _ in range(window)]
    assert np.allclose(ramp_up, np.linspace(0.0, 1.0, window))

    blender.notify_mode(MODE_FAST)
    assert blender.blending
    ramp_down = [blender.step(fast, precise)[0] for _ in range(window)]
    assert np.allclose(ramp_down, 1.0 - np.linspace(0.0, 1.0, window))


def test_shape_mismatch_raises():
    blender = DualPathBlender(5)
    blender.notify_mode(MODE_FAST)
    blender.step(np.zeros(3), None)
    blender.notify_mode(MODE_PRECISE)
    with pytest.raises(ValueError):
        blender.step(np.zeros(3), np.zeros(4))
