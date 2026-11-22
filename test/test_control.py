"""Exercise command safety without ROS, hardware, or a running simulation."""

import math

from legged_robot_teleop.control import Settings, TeleopControl
import pytest


def sample(*pressed, axes=(0.0,) * 6, pressure=0.0):
    """Construct standardized SDL buttons and a ROS-sign L2 trigger axis."""
    axes = list(axes)
    axes[4] = -pressure
    buttons = [0] * 15
    for index in pressed:
        buttons[index] = 1
    return axes, buttons


def arm(control, now=0.0):
    """Release the deadman and then press it with centered controls."""
    control.receive(*sample(), now)
    control.receive(*sample(pressure=1.0), now + 0.01)
    assert control.enabled


def test_startup_requires_release_and_neutral_press():
    control = TeleopControl(Settings())
    assert control.command(0).forward == 0
    control.receive(*sample(pressure=1.0), 0.0)
    assert not control.enabled
    control.receive(*sample(), 0.01)
    control.receive(*sample(axes=(0, 1, 0, 0, 0, 0), pressure=1.0), 0.02)
    assert not control.enabled
    control.receive(*sample(pressure=1.0), 0.03)
    assert not control.enabled
    arm(control, 0.04)


def test_axes_are_normalized_with_ros_signs():
    control = TeleopControl(Settings(scale_forward=0.4, scale_lateral=0.2, scale_yaw=0.3))
    arm(control)
    control.receive(*sample(axes=(-0.5, 1.0, -1.0, 0, 0, 0), pressure=1.0), 0.02)
    command = control.command(0.03)
    assert (command.forward, command.lateral, command.yaw) == (0.4, -0.1, -0.3)


def test_release_zeroes_velocities_preserving_height():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(11, pressure=1.0), 0.02)
    height = control.command(0.04).height
    control.receive(*sample(), 0.05)
    command = control.command(0.06)
    assert (command.forward, command.lateral, command.yaw) == (0, 0, 0)
    assert command.height == height
    control.receive(*sample(pressure=1.0), 0.07)
    assert control.enabled


def test_circle_is_unbound_during_arming_and_driving():
    control = TeleopControl(Settings())
    control.receive(*sample(1, pressure=1.0), 0.0)
    assert not control.enabled, 'Circle must not bypass startup release'
    control.receive(*sample(1), 0.01)
    control.receive(*sample(1, pressure=1.0), 0.02)
    assert control.enabled, 'Holding Circle must not block a safe L2 press'
    control.receive(*sample(1, axes=(0, 1, 0, 0, 0, 0), pressure=1.0), 0.03)
    assert control.command(0.04).forward == control.settings.scale_forward
    control.receive(*sample(pressure=1.0), 0.05)
    assert control.enabled, 'Releasing Circle must not disarm driving'
    control.receive(*sample(1), 0.06)
    assert not control.enabled, 'L2 release must still stop motion'


def test_timeout_and_reconnection_require_fresh_release():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(axes=(0, 1, 0, 0, 0, 0), pressure=1.0), 0.02)
    assert control.command(0.03).forward > 0
    assert control.command(0.28).forward == 0
    assert control.state == 'input_timeout'
    control.receive(*sample(pressure=1.0), 0.29)
    assert not control.enabled
    arm(control, 0.30)


def test_receive_detects_timeout_without_a_timer_callback():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(pressure=1.0), 1.0)
    assert not control.enabled


@pytest.mark.parametrize('axes,buttons', [
    ([], [0] * 15),
    ([0] * 6, []),
    ([math.nan] * 6, [0] * 15),
    ([math.inf] * 6, [0] * 15),
    ([1.1] * 6, [0] * 15),
    ([0] * 6, [2] * 15),
])
def test_invalid_input_disarms(axes, buttons):
    control = TeleopControl(Settings())
    arm(control)
    control.receive(axes, buttons, 0.02)
    assert control.state == 'invalid_input'
    assert control.command(0.03).forward == 0
    control.receive(*sample(pressure=1.0), 0.04)
    assert not control.enabled


def test_height_limits_and_center():
    control = TeleopControl(Settings(
        reference_height=0.22, minimum_height=0.2, maximum_height=0.33, height_rate=0.5,
    ))
    arm(control)
    for i in range(1, 50):
        now = 0.02 * i
        control.receive(*sample(11, pressure=1.0), now)
        control.command(now)
    assert control.height == 0.33
    for i in range(50, 100):
        now = 0.02 * i
        control.receive(*sample(12, pressure=1.0), now)
        control.command(now)
    assert control.height == 0.2
    control.receive(*sample(9), 2.0)
    command = control.command(2.0)
    assert command.height == 0.22
    assert (command.forward, command.lateral, command.yaw) == (0, 0, 0)
    assert not control.enabled


def test_both_height_buttons_cancel_and_disabled_height_is_ignored():
    control = TeleopControl(Settings())
    control.receive(*sample(11), 0.0)
    assert control.command(0.02).height == 0.6
    arm(control, 0.03)
    control.receive(*sample(11, 12, pressure=1.0), 0.05)
    assert control.command(0.06).height == 0.6


def test_height_does_not_jump_after_a_delayed_callback():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(11, pressure=1.0), 0.2)
    assert control.command(0.21).height - 0.6 <= 0.05 * 0.04 + 1e-12


def test_upkie_ignores_lateral_axis_and_optional_buttons_can_be_disabled():
    control = TeleopControl(Settings(
        scale_lateral=0.0, axis_lateral=-1, button_center=-1,
    ))
    arm(control)
    control.receive(*sample(axes=(1, 0.5, 0, 0, 0, 0), pressure=1.0), 0.02)
    command = control.command(0.03)
    assert command.forward == 0.125
    assert command.lateral == 0


@pytest.mark.parametrize('parameters', [
    {'minimum_height': 0.0}, {'reference_height': 2.0},
    {'height_rate': -0.1}, {'input_timeout': 0.0},
    {'scale_forward': 1.1}, {'scale_yaw': math.nan},
    {'axis_enable': -1}, {'button_center': 11},
    {'axis_forward': -2}, {'axis_yaw': 1.0},
    {'sign_forward': 0.0}, {'publish_rate': 0.0},
    {'neutral_threshold': 1.0},
    {'enable_threshold': 0.2}, {'enable_threshold': 1.1},
    {'release_threshold': -0.1}, {'axis_enable': 1}, {'sign_enable': 0.0},
])
def test_invalid_settings_fail_fast(parameters):
    with pytest.raises(ValueError):
        Settings(**parameters)


def test_l1_does_not_enable_motion():
    control = TeleopControl(Settings())
    control.receive(*sample(), 0.0)
    control.receive(*sample(9), 0.01)
    assert not control.enabled
    assert control.command(0.02).forward == 0


def test_trigger_threshold_hysteresis_and_gradual_press():
    control = TeleopControl(Settings())
    control.receive(*sample(), 0.0)
    control.receive(*sample(pressure=0.3), 0.01)
    assert not control.enabled
    control.receive(*sample(pressure=0.5), 0.02)
    assert control.enabled
    control.receive(*sample(pressure=0.3), 0.03)
    assert control.enabled
    control.receive(*sample(pressure=0.2), 0.04)
    assert not control.enabled
    assert control.command(0.05).forward == 0


def test_partly_held_trigger_cannot_rearm_after_timeout_or_stop():
    control = TeleopControl(Settings())
    arm(control)
    control.command(1.0)
    control.receive(*sample(pressure=0.3), 1.01)
    control.receive(*sample(pressure=1.0), 1.02)
    assert not control.enabled
    arm(control, 1.03)
    control.stop('robot_inactive')
    control.receive(*sample(pressure=0.3), 1.06)
    control.receive(*sample(pressure=1.0), 1.07)
    assert not control.enabled


def test_missing_enable_axis_disarms():
    control = TeleopControl(Settings())
    arm(control)
    control.receive([0.0] * 4, [0] * 15, 0.02)
    assert control.state == 'invalid_input'


def test_center_is_ignored_while_enabled_and_not_queued_until_release():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(11, pressure=1.0), 0.02)
    height = control.command(0.04).height
    assert height > control.settings.reference_height
    control.receive(*sample(9, axes=(0, 1, 0, 0, 0, 0), pressure=1.0), 0.05)
    command = control.command(0.06)
    assert command.forward == 0.25
    assert command.height == height
    control.receive(*sample(9), 0.07)
    command = control.command(0.08)
    assert command.forward == 0
    assert command.height == height
    control.receive(*sample(), 0.09)
    control.receive(*sample(9), 0.1)
    assert control.command(0.11).height == control.settings.reference_height


def test_center_held_cannot_enable_motion():
    control = TeleopControl(Settings())
    control.receive(*sample(9), 0.0)
    control.receive(*sample(9, pressure=1.0), 0.01)
    assert not control.enabled
    control.receive(*sample(pressure=1.0), 0.02)
    assert not control.enabled
    arm(control, 0.03)


def test_center_is_ignored_with_a_partially_pressed_trigger():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(11, pressure=1.0), 0.02)
    height = control.command(0.04).height
    control.receive(*sample(), 0.05)
    control.receive(*sample(9, pressure=0.3), 0.06)
    assert control.command(0.07).height == height
    control.receive(*sample(9), 0.08)
    assert control.command(0.09).height == height


def test_circle_does_not_block_center():
    control = TeleopControl(Settings())
    arm(control)
    control.receive(*sample(11, pressure=1.0), 0.02)
    assert control.command(0.04).height > control.settings.reference_height
    control.receive(*sample(1, 9), 0.05)
    assert control.state == 'centered'
    control.receive(*sample(9), 0.06)
    command = control.command(0.07)
    assert command.height == control.settings.reference_height
    assert command.forward == 0


def test_center_release_and_l2_press_can_arrive_in_the_same_frame():
    control = TeleopControl(Settings())
    control.receive(*sample(9), 0.0)
    assert not control.enabled
    control.receive(*sample(pressure=1.0), 0.01)
    assert control.enabled


def test_center_release_during_gradual_l2_press_preserves_ready_state():
    control = TeleopControl(Settings())
    control.receive(*sample(9), 0.0)
    control.receive(*sample(pressure=0.3), 0.01)
    assert not control.enabled
    control.receive(*sample(pressure=0.6), 0.02)
    assert control.enabled
