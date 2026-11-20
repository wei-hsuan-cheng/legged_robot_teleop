"""ROS-independent input validation, deadman control, and height integration."""

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Settings:
    """Immutable command mapping and limits; velocities are normalized fractions."""

    axis_forward: int = 1
    axis_lateral: int = 0
    axis_yaw: int = 2
    axis_enable: int = 4
    sign_forward: float = 1.0
    sign_lateral: float = 1.0
    sign_yaw: float = 1.0
    sign_enable: float = -1.0
    scale_forward: float = 0.25
    scale_lateral: float = 0.25
    scale_yaw: float = 0.25
    enable_threshold: float = 0.5
    release_threshold: float = 0.2
    button_stop: int = 1
    button_height_up: int = 11
    button_height_down: int = 12
    button_height_reset: int = 3
    reference_height: float = 0.6
    minimum_height: float = 0.2
    maximum_height: float = 1.0
    height_rate: float = 0.05
    input_timeout: float = 0.25
    neutral_threshold: float = 0.1
    publish_rate: float = 50.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if name.startswith(('axis_', 'button_')):
                if type(value) is not int or value < -1:
                    raise ValueError(f'{name} must be an integer >= -1')
            elif not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.axis_enable < 0 or self.button_stop < 0:
            raise ValueError('Enable axis and stop button cannot be disabled')
        if self.axis_enable in (self.axis_forward, self.axis_lateral, self.axis_yaw):
            raise ValueError('Enable axis must be separate from motion axes')
        if not 0 <= self.release_threshold < self.enable_threshold <= 1:
            raise ValueError('Require 0 <= release_threshold < enable_threshold <= 1')
        buttons = [v for k, v in vars(self).items() if k.startswith('button_') and v >= 0]
        if len(set(buttons)) != len(buttons):
            raise ValueError('Button assignments must be distinct')
        for name in ('forward', 'lateral', 'yaw', 'enable'):
            if getattr(self, 'sign_' + name) not in (-1.0, 1.0):
                raise ValueError('Axis signs must be -1 or 1')
        for name in ('forward', 'lateral', 'yaw'):
            if not 0.0 <= getattr(self, 'scale_' + name) <= 1.0:
                raise ValueError('Normalized velocity scales must be within [0, 1]')
        if not 0 < self.minimum_height <= self.reference_height <= self.maximum_height:
            raise ValueError('Require 0 < minimum_height <= reference_height <= maximum_height')
        if self.height_rate <= 0 or self.input_timeout <= 0:
            raise ValueError('Height rate and input timeout must be positive')
        if not 1 <= self.publish_rate <= 200:
            raise ValueError('Publish rate must be within [1, 200] Hz')
        if not 0 <= self.neutral_threshold < 1:
            raise ValueError('Neutral threshold must be within [0, 1)')


@dataclass(frozen=True)
class Command:
    """Normalized velocities and absolute base height in meters."""

    forward: float
    lateral: float
    yaw: float
    height: float


class TeleopControl:
    """Require fresh input and a neutral release/press sequence before motion."""

    def __init__(self, settings: Settings):
        self.settings = settings
        self.height = settings.reference_height
        self.state = 'waiting_for_input'
        self.enabled = False
        self._released = False
        self._last_input = None
        self._last_tick = None
        self._axes = (0.0, 0.0, 0.0)
        self._buttons = (False, False, False)

    def stop(self, state='disabled'):
        """Clear permission to move without changing the selected height."""
        self.enabled = False
        self._released = False
        self._axes = (0.0, 0.0, 0.0)
        self._buttons = (False, False, False)
        self.state = state

    def receive(self, axes: Sequence[float], buttons: Sequence[int], now: float):
        """Validate input and update the enable state using monotonic receipt time."""
        s = self.settings
        if self._last_input is not None and now - self._last_input >= s.input_timeout:
            self.stop('input_timeout')
        axis_indices = (s.axis_forward, s.axis_lateral, s.axis_yaw)
        button_indices = (
            s.button_stop, s.button_height_up,
            s.button_height_down, s.button_height_reset,
        )
        valid = (
            math.isfinite(now)
            and all(i < len(axes) for i in (*axis_indices, s.axis_enable))
            and all(i < len(buttons) for i in button_indices)
            and all(math.isfinite(a) and abs(a) <= 1.0001 for a in axes)
            and all(b in (0, 1) for b in buttons)
        )
        if not valid:
            self.stop('invalid_input')
            self._last_input = None
            return

        def pressed(index):
            return index >= 0 and bool(buttons[index])

        values = tuple(
            max(-1.0, min(1.0, axes[i])) if i >= 0 else 0.0 for i in axis_indices
        )
        self._last_input = now
        if pressed(s.button_stop):
            self.stop('stop_button')
            return
        pressure = max(0.0, min(1.0, s.sign_enable * axes[s.axis_enable]))
        if pressure <= s.release_threshold:
            self.stop('disabled')
            self._released = True
            return
        if not self.enabled and pressure < s.enable_threshold:
            self.state = (
                'press_trigger_to_enable' if self._released else 'release_and_center_to_enable'
            )
            return
        height_buttons = tuple(pressed(i) for i in button_indices[1:])
        active_axes = [
            a for a, scale in zip(values, (s.scale_forward, s.scale_lateral, s.scale_yaw))
            if scale > 0
        ]
        if not self.enabled:
            if (not self._released
                    or any(abs(a) > s.neutral_threshold for a in active_axes)
                    or any(height_buttons)):
                self._released = False
                self.state = 'release_and_center_to_enable'
                return
            self.enabled = True
            self._released = False
            self._last_tick = now
        self._axes = values
        self._buttons = height_buttons
        self.state = 'enabled'

    def command(self, now: float) -> Command:
        """Return a safe command even when the ROS simulation clock is paused."""
        s = self.settings
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now
        if self._last_input is not None and now - self._last_input >= s.input_timeout:
            self.stop('input_timeout')
        if not self.enabled:
            return Command(0.0, 0.0, 0.0, self.height)
        up, down, reset = self._buttons
        if reset:
            self.height = s.reference_height
        else:
            # A delayed callback must not cause a large jump in the height target.
            self.height += (int(up) - int(down)) * s.height_rate * min(dt, 2 / s.publish_rate)
            self.height = max(s.minimum_height, min(s.maximum_height, self.height))
        forward, lateral, yaw = self._axes
        return Command(
            forward * s.sign_forward * s.scale_forward,
            lateral * s.sign_lateral * s.scale_lateral,
            yaw * s.sign_yaw * s.scale_yaw,
            self.height,
        )
