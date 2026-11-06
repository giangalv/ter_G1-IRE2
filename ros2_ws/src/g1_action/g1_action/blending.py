"""Dual-path FAST <-> PRECISE action blending (rclpy-free).

When the VLA switches between the fast (flow-matching) and precise (DDIM)
action heads, the executor must not jump between the two trajectories.
:class:`DualPathBlender` ramps a blending factor ``alpha`` linearly from 0 to
1 over ``blend_window_steps`` executor ticks (50 Hz), producing

    action = (1 - alpha) * previous_path + alpha * new_path

The blender needs the last action of each path; if only one path is
available the new action passes through unchanged.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


class DualPathBlender:
    """Linear cross-fade between the two VLA action paths.

    The blender is driven once per executor tick:

    1. :meth:`notify_mode` with the mode of the chunk currently being
       executed. A mode change restarts the ramp.
    2. :meth:`step` with the current action of the active mode and, if
       known, the last action produced by the previously active mode.

    ``alpha`` follows ``linspace(0, 1, blend_window_steps)``: the first tick
    after a switch still outputs the old path (alpha = 0) and the ramp is
    complete after exactly ``blend_window_steps`` ticks.
    """

    def __init__(self, blend_window_steps: int) -> None:
        """
        Args:
            blend_window_steps: number of 50 Hz ticks over which alpha ramps
                from 0 to 1. Values below 1 are clamped to 1 (instant switch).
        """
        self._window: int = max(1, int(blend_window_steps))
        self._mode: Optional[int] = None
        # Start with a completed ramp so the very first mode is a passthrough.
        self._ticks: int = self._window

    @property
    def window(self) -> int:
        """Ramp length in ticks."""
        return self._window

    @property
    def active_mode(self) -> Optional[int]:
        """Mode of the path currently being ramped toward (None until first notify)."""
        return self._mode

    @property
    def blending(self) -> bool:
        """True while a mode transition ramp is in progress."""
        return self._ticks < self._window

    @property
    def alpha(self) -> float:
        """Current blend factor in [0, 1] (1.0 once the ramp has completed)."""
        if self._ticks >= self._window or self._window == 1:
            return 1.0
        return self._ticks / (self._window - 1)

    def notify_mode(self, mode: int) -> None:
        """Inform the blender of the mode of the chunk being executed.

        The first mode ever seen is adopted without a ramp; any subsequent
        change (FAST <-> PRECISE) restarts the alpha ramp at 0.
        """
        if self._mode is None:
            self._mode = int(mode)
            self._ticks = self._window
            return
        if int(mode) != self._mode:
            self._mode = int(mode)
            self._ticks = 0

    def reset(self) -> None:
        """Abort any ramp in progress and forget the active mode."""
        self._mode = None
        self._ticks = self._window

    def step(
        self,
        action: np.ndarray,
        previous_mode_action: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Advance one tick and return the blended action.

        Args:
            action: action of the currently active mode (any shape).
            previous_mode_action: last known action of the previously active
                mode (same shape), or None if only one path is available.

        Returns:
            Blended action. Passthrough (a copy of ``action``) when no ramp
            is in progress or when ``previous_mode_action`` is None.
        """
        new = np.asarray(action, dtype=np.float64)
        a = self.alpha
        if self._ticks < self._window:
            self._ticks += 1
        if previous_mode_action is None or a >= 1.0:
            return new.copy()
        old = np.asarray(previous_mode_action, dtype=np.float64)
        if old.shape != new.shape:
            raise ValueError(
                f"blend shape mismatch: {old.shape} vs {new.shape}"
            )
        return (1.0 - a) * old + a * new
