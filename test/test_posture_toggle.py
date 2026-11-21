"""Exercise posture-button edges and interlocks through ROS services, without a robot."""

import time
from uuid import uuid4

import pytest


@pytest.mark.parametrize('posture_button', [2, 10])
def test_posture_toggle_requires_release_and_safe_fresh_robot_state(posture_button):
    """A held or blocked button never becomes a delayed stand/sit request."""
    pytest.importorskip('rclpy')
    from legged_robot_teleop.node import BaseTwistTeleop
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    context = Context()
    context.init(domain_id=156)
    namespace = '/teleop_posture_test_' + uuid4().hex
    overrides = [
        Parameter('joy_topic', value=namespace + '/joy'),
        Parameter('command_topic', value=namespace + '/command'),
        Parameter('posture_controller', value=namespace),
    ]
    if posture_button != 2:
        overrides.append(Parameter('button_posture', value=posture_button))
    teleop = BaseTwistTeleop(context=context, parameter_overrides=overrides)
    assert teleop._posture_button == posture_button
    probe = Node('posture_probe', context=context)
    calls = []

    def toggle(request, response):
        calls.append(request)
        response.success = True
        response.message = 'accepted'
        return response

    service = probe.create_service(Trigger, namespace + '/toggle_stand', toggle)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    state_publisher = probe.create_publisher(String, namespace + '/motion_state', qos)
    joy_publisher = probe.create_publisher(Joy, namespace + '/joy', qos)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(teleop)
    executor.add_node(probe)

    def pump(pressed=(), pressure=0.0, state='idle', forward=0.0, duration=0.15):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                state_publisher.publish(String(data=state))
                message = Joy()
                message.axes = [0.0, forward, 0.0, 0.0, -pressure, 0.0]
                message.buttons = [int(i in pressed) for i in range(15)]
                joy_publisher.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    try:
        pump((posture_button,), duration=0.5)
        assert not calls, 'A held posture button at connection is not a new request'
        pump()
        pump((10 if posture_button == 2 else 2,))
        assert not calls, 'The previous/other mapping must not request a posture'
        pump()
        pump((posture_button,))
        assert len(calls) == 1
        pump((posture_button,), duration=0.3)
        assert len(calls) == 1, 'Autorepeat must not toggle again'
        pump()
        end = time.monotonic() + .35
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        pump((posture_button,))
        assert len(calls) == 1, 'Held button after an input gap must not request a posture'
        pump()
        pump((posture_button,), pressure=1.0)
        pump((posture_button,), pressure=0.0)
        assert len(calls) == 1, 'A blocked posture button is consumed, not queued'
        for buttons, forward, state in (
                ((posture_button, 9), 0.0, 'idle'), ((posture_button, 1), 0.0, 'idle'),
                ((posture_button, 11), 0.0, 'idle'), ((posture_button,), 1.0, 'idle'),
                ((posture_button,), 0.0, 'standing up'), ((posture_button,), 0.0, 'fallen')):
            # Robot state and Joy are independent DDS topics; deliver the new state
            # before checking the adapter's guard. The real server also validates its phase.
            pump(state=state)
            assert teleop._motion_state == state
            pump(buttons, state=state, forward=forward)
            assert len(calls) == 1
        pump(state='locomotion')
        pump((posture_button,), state='locomotion')
        assert len(calls) == 2
        pump(state='sitting down')
        pump(pressure=1.0, state='sitting down')
        assert not teleop.control.enabled
        pump(pressure=1.0, state='locomotion')
        assert not teleop.control.enabled, 'Held L2 must not enable at handover'
        pump(state='locomotion')
        pump(pressure=1.0, state='locomotion')
        assert teleop.control.enabled
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert not teleop.control.enabled
        assert teleop._motion_state == 'unknown'
    finally:
        probe.destroy_service(service)
        executor.shutdown()
        teleop.destroy_node()
        probe.destroy_node()
        context.shutdown()


def test_shipped_posture_mapping_is_square():
    """Keep the launch YAML and the intended DualSense button consistent."""
    from pathlib import Path
    import yaml

    config = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/dualsense.yaml').read_text(),
    )
    assert config['base_twist_teleop']['ros__parameters']['button_posture'] == 2
