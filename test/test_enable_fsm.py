"""Check FSM transitions, interleaved controls, and safety invariants."""

import math
import random

from legged_robot_teleop.control import EnableState, Settings, TeleopControl
import pytest


def send(control, now, pressure=0.0, forward=0.0, buttons=()):
    """Deliver a complete game-controller snapshot rather than separate events."""
    axes = [0.0, forward, 0.0, 0.0, -pressure, 0.0]
    keys = [int(i in buttons) for i in range(15)]
    control.receive(axes, keys, now)


def test_ready_survives_slow_trigger_squeeze_after_center():
    control = TeleopControl(Settings())
    send(control, 0.0, buttons=(9,))
    assert control.enable_state is EnableState.READY
    for i, pressure in enumerate((0.21, 0.3, 0.45, 0.49), 1):
        send(control, 0.01 * i, pressure)
        assert control.enable_state is EnableState.READY
        assert control.command(0.01 * i).forward == 0
    send(control, 0.05, 0.5)
    assert control.enable_state is EnableState.ENABLED


@pytest.mark.parametrize('buttons,forward,reason', [
    ((9,), 0.0, 'center_held'),
    ((11,), 0.0, 'height_buttons_held'),
    ((), 0.2, 'sticks_not_centered'),
])
def test_blocked_press_explains_reason_and_requires_release(buttons, forward, reason):
    control = TeleopControl(Settings())
    send(control, 0.0)
    send(control, 0.01, 0.6, forward, buttons)
    assert control.enable_state is EnableState.WAIT_RELEASE
    assert control.state == 'enable_blocked_' + reason
    send(control, 0.02, 0.9)
    assert control.state == 'enable_blocked_' + reason
    assert control.command(0.03).forward == 0
    send(control, 0.04)
    assert control.enable_state is EnableState.READY
    send(control, 0.05, 0.6)
    assert control.enabled


def test_ready_can_wait_for_neutral_before_trigger_crosses_threshold():
    control = TeleopControl(Settings())
    send(control, 0.0, forward=0.5)
    assert control.enable_state is EnableState.READY
    send(control, 0.01, 0.4, forward=0.5)
    assert control.state == 'sticks_not_centered'
    send(control, 0.02, 0.45)
    assert control.state == 'ready'
    send(control, 0.03, 0.6)
    assert control.enabled


def test_enabled_hysteresis_does_not_require_neutral_sticks():
    control = TeleopControl(Settings())
    send(control, 0.0)
    send(control, 0.01, 0.6)
    for i, pressure in enumerate((0.49, 0.3, 0.201, 0.6), 2):
        send(control, 0.01 * i, pressure, forward=0.8, buttons=(9,))
        assert control.enabled
        assert control.command(0.01 * i).forward == 0.2
    send(control, 0.06, 0.2, forward=0.8)
    assert control.enable_state is EnableState.READY
    assert control.command(0.07).forward == 0


@pytest.mark.parametrize('failure', ['stop', 'timeout', 'malformed'])
def test_fault_and_center_release_recovery(failure):
    control = TeleopControl(Settings())
    send(control, 0.0)
    send(control, 0.01, 0.6)
    if failure == 'stop':
        send(control, 0.02, 0.6, buttons=(1,))
    elif failure == 'timeout':
        control.command(0.3)
    else:
        control.receive([], [], 0.02)
    assert control.enable_state is EnableState.WAIT_RELEASE
    send(control, 0.4, 0.6)
    assert not control.enabled
    send(control, 0.41, buttons=(9,))
    assert control.enable_state is EnableState.READY
    send(control, 0.42, 0.6)
    assert control.enabled


@pytest.mark.parametrize('now', [math.nan, math.inf, -0.1])
@pytest.mark.parametrize('method', ['receive', 'command'])
def test_invalid_time_disarms_and_can_recover(now, method):
    control = TeleopControl(Settings())
    send(control, 0.0)
    send(control, 0.01, 0.6)
    if method == 'receive':
        send(control, now, 0.6)
    else:
        command = control.command(now)
        assert command.forward == command.lateral == command.yaw == 0
    assert control.state == 'invalid_time'
    assert control.enable_state is EnableState.WAIT_RELEASE
    send(control, 0.1, 0.6)
    assert not control.enabled
    send(control, 0.11)
    send(control, 0.12, 0.6)
    assert control.enabled


@pytest.mark.parametrize('seed', range(6))
def test_randomized_input_sequences_never_bypass_enable_guards(seed):
    rng = random.Random(seed)
    control = TeleopControl(Settings())
    now = 0.0
    for _ in range(1000):
        now += rng.choice((0.001, 0.01, 0.02, 0.3))
        previous_mode = control.enable_state
        pressure = rng.choice((0.0, 0.19, 0.2, 0.21, 0.49, 0.5, 0.8, 1.0))
        forward = rng.choice((0.0, 0.05, 0.3, 1.0, -1.0))
        buttons = rng.choice(((), (9,), (1,), (11,), (12,), (1, 9), (11, 12)))
        send(control, now, pressure, forward, buttons)
        command = control.command(now)
        assert control.enabled == (control.enable_state is EnableState.ENABLED)
        assert control.settings.minimum_height <= command.height <= control.settings.maximum_height
        assert all(math.isfinite(v) for v in vars(command).values())
        assert abs(command.forward) <= control.settings.scale_forward
        if not control.enabled:
            assert command.forward == command.lateral == command.yaw == 0
        if pressure <= control.settings.release_threshold or 1 in buttons:
            assert not control.enabled
        if control.enabled and previous_mode is not EnableState.ENABLED:
            assert previous_mode is EnableState.READY
            assert pressure >= control.settings.enable_threshold
            assert abs(forward) <= control.settings.neutral_threshold
            assert not buttons
