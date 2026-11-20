"""Verify actual DDS output and a paused-clock timeout on isolated test topics."""

import time
from uuid import uuid4

import pytest


def test_transport_and_timeout_with_no_simulation_clock():
    pytest.importorskip('rclpy')
    pytest.importorskip('ocs2_msgs')
    from legged_robot_teleop.node import BaseTwistTeleop
    from ocs2_msgs.msg import WalkingVelocityCommand
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy

    context = Context()
    context.init(domain_id=156)
    suffix = 'run_' + uuid4().hex
    joy_topic = '/teleop_test/' + suffix + '/joy'
    command_topic = '/teleop_test/' + suffix + '/command'
    teleop = BaseTwistTeleop(context=context, parameter_overrides=[
        Parameter('joy_topic', value=joy_topic),
        Parameter('command_topic', value=command_topic),
        Parameter('use_sim_time', value=True),
    ])
    probe = Node('teleop_test_probe_' + suffix, context=context)
    publisher = probe.create_publisher(Joy, joy_topic, qos_profile_sensor_data)
    received = []
    subscription = probe.create_subscription(
        WalkingVelocityCommand, command_topic, received.append, qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(teleop)
    executor.add_node(probe)

    def pump(duration, axes=None, pressed=()):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if axes is not None and time.monotonic() >= next_send:
                message = Joy()
                message.axes = list(axes)
                message.buttons = [int(i in pressed) for i in range(15)]
                publisher.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    try:
        pump(0.5, [0.0] * 6)
        assert received and received[-1].linear_velocity_x == 0.0
        pump(0.1, [0.0] * 6, (9,))
        assert teleop.control.enabled
        pump(0.15, [0.0, 1.0, 0.5, 0.0, 0.0, 0.0], (9,))
        assert received[-1].linear_velocity_x == 0.25
        assert received[-1].angular_velocity_z == 0.125
        assert received[-1].desired_pelvis_height == 0.6
        count = len(received)
        pump(0.4)
        assert len(received) > count
        assert received[-1].linear_velocity_x == 0.0
        assert received[-1].angular_velocity_z == 0.0
        assert teleop.control.state == 'input_timeout'
        pump(0.1, [0.0] * 6, (9,))
        assert not teleop.control.enabled
        pump(0.1, [0.0] * 6)
        pump(0.1, [0.0] * 6, (9,))
        assert teleop.control.enabled
        pump(0.1, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], (9,))
        assert received[-1].linear_velocity_x > 0
        pump(0.1, [0.0] * 6)
        assert received[-1].linear_velocity_x == 0
        assert teleop.get_clock().now().nanoseconds == 0
    finally:
        probe.destroy_subscription(subscription)
        executor.shutdown()
        teleop.destroy_node()
        probe.destroy_node()
        context.shutdown()
