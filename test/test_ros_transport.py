"""Verify actual DDS output with a paused simulation clock on isolated test topics."""

import json
import time
from uuid import uuid4

import pytest


@pytest.mark.parametrize('bridge', [False, True])
def test_transport_with_no_simulation_clock(bridge):
    pytest.importorskip('rclpy')
    pytest.importorskip('ocs2_msgs')
    from legged_robot_teleop.node import BaseTwistTeleop
    from ocs2_msgs.msg import WalkingVelocityCommand
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    context = Context()
    context.init(domain_id=156)
    suffix = 'run_' + uuid4().hex
    joy_topic = '/teleop_test/' + suffix + '/joy'
    source_topic = joy_topic + '/raw' if bridge else joy_topic
    command_topic = '/teleop_test/' + suffix + '/command'
    teleop = BaseTwistTeleop(context=context, parameter_overrides=[
        Parameter('joy_topic', value=joy_topic),
        Parameter('joy_source_topic', value=source_topic if bridge else source_topic.lstrip('/')),
        Parameter('command_topic', value=command_topic),
        Parameter('use_sim_time', value=True),
    ])
    probe = Node('teleop_test_probe_' + suffix, context=context)
    publisher = probe.create_publisher(
        Joy, source_topic,
        QoSProfile(depth=1, reliability=(
            ReliabilityPolicy.RELIABLE if bridge else ReliabilityPolicy.BEST_EFFORT
        )),
    )
    received = []
    statuses = []
    forwarded = []
    subscription = probe.create_subscription(
        WalkingVelocityCommand, command_topic, received.append, qos_profile_sensor_data,
    )
    status_subscription = probe.create_subscription(
        String, '/base_twist_teleop/status', lambda msg: statuses.append(json.loads(msg.data)),
        qos_profile_sensor_data,
    )
    joy_subscription = None
    if bridge:
        joy_subscription = probe.create_subscription(
            Joy, joy_topic, forwarded.append, qos_profile_sensor_data,
        )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(teleop)
    executor.add_node(probe)

    def pump(duration, axes=None, pressed=(), pressure=0.0):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if axes is not None and time.monotonic() >= next_send:
                message = Joy()
                message.axes = list(axes)
                message.axes[4] = -pressure
                message.buttons = [int(i in pressed) for i in range(15)]
                publisher.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    try:
        pump(0.5, [0.0] * 6)
        assert received and received[-1].linear_velocity_x == 0.0
        pump(0.1, [0.0] * 6, pressure=1.0)
        assert teleop.control.enabled
        pump(0.15, [0.0, 1.0, 0.5, 0.0, 0.0, 0.0], pressure=1.0)
        assert teleop._subscription.qos_profile.depth == 1
        assert teleop._subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert received[-1].linear_velocity_x == 0.25
        assert received[-1].angular_velocity_z == 0.125
        assert received[-1].desired_pelvis_height == 0.6
        count = len(received)
        pump(0.4)
        assert len(received) > count
        assert received[-1].linear_velocity_x == 0.25, 'no input timeout: the last command holds'
        assert teleop.control.enabled
        pump(0.1, [0.0] * 6)
        assert not teleop.control.enabled
        assert received[-1].linear_velocity_x == 0.0
        pump(0.1, [0.0] * 6, pressure=1.0)
        assert teleop.control.enabled
        pump(0.1, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], pressure=1.0)
        assert received[-1].linear_velocity_x > 0
        pump(0.1, [0.0] * 6, pressed=(11,), pressure=1.0)
        assert received[-1].desired_pelvis_height > 0.6
        pump(0.1, [0.0] * 6, pressure=1.0)
        height = received[-1].desired_pelvis_height
        pump(0.1, [0.0] * 6, pressed=(9,), pressure=1.0)
        assert received[-1].desired_pelvis_height == height
        pump(0.1, [0.0] * 6, pressed=(9,))
        assert received[-1].desired_pelvis_height == height
        pump(0.1, [0.0] * 6)
        assert received[-1].linear_velocity_x == 0
        pump(0.1, [0.0] * 6, pressed=(9,))
        assert received[-1].desired_pelvis_height == 0.6
        assert received[-1].linear_velocity_x == 0
        assert received[-1].linear_velocity_y == 0
        assert received[-1].angular_velocity_z == 0
        pump(0.1, [0.0] * 6, pressure=1.0)
        assert teleop.control.enabled
        assert statuses[-1]['phase'] == 'enabled'
        assert statuses[-1]['enabled']
        if bridge:
            assert forwarded and forwarded[-1].axes[4] == -1.0
            info = probe.get_publishers_info_by_topic(joy_topic)
            assert len(info) == 1
            assert info[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
            assert teleop._joy_publisher.qos_profile.depth == 1
        else:
            assert teleop._joy_publisher is None
        assert teleop.get_clock().now().nanoseconds == 0
    finally:
        probe.destroy_subscription(subscription)
        probe.destroy_subscription(status_subscription)
        if joy_subscription is not None:
            probe.destroy_subscription(joy_subscription)
        executor.shutdown()
        teleop.destroy_node()
        probe.destroy_node()
        context.shutdown()
