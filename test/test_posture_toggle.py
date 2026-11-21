"""Exercise R1 edges and trigger interlocks through real ROS services, without a robot."""

import time
from uuid import uuid4

import pytest


def test_r1_toggle_requires_release_and_safe_fresh_robot_state():
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
    teleop = BaseTwistTeleop(context=context, parameter_overrides=[
        Parameter('joy_topic', value=namespace + '/joy'),
        Parameter('command_topic', value=namespace + '/command'),
        Parameter('posture_controller', value=namespace),
    ])
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
        pump((10,), duration=0.5)
        assert not calls, 'A held R1 at connection is not a new request'
        pump()
        pump((10,))
        assert len(calls) == 1
        pump((10,), duration=0.3)
        assert len(calls) == 1, 'Autorepeat must not toggle again'
        pump()
        end = time.monotonic() + .35
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        pump((10,))
        assert len(calls) == 1, 'Held R1 after an input gap must not request a posture'
        pump()
        pump((10,), pressure=1.0)
        pump((10,), pressure=0.0)
        assert len(calls) == 1, 'A blocked R1 is consumed, not queued'
        for buttons, forward, state in (
                ((10, 9), 0.0, 'idle'), ((10, 1), 0.0, 'idle'),
                ((10, 11), 0.0, 'idle'), ((10,), 1.0, 'idle'),
                ((10,), 0.0, 'standing up'), ((10,), 0.0, 'fallen')):
            # Robot state and Joy are independent DDS topics; deliver the new state
            # before checking the adapter's guard. The real server also validates its phase.
            pump(state=state)
            assert teleop._motion_state == state
            pump(buttons, state=state, forward=forward)
            assert len(calls) == 1
        pump(state='locomotion')
        pump((10,), state='locomotion')
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
