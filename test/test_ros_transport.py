"""Verify actual DDS output with a paused simulation clock on isolated test topics."""

import time
from uuid import uuid4

from conftest import DOMAIN_ID, Status, wait_for
import pytest


@pytest.mark.parametrize('bridge', [False, True])
def test_transport_with_no_simulation_clock(bridge, teleop_process):
    pytest.importorskip('rclpy')
    pytest.importorskip('ocs2_msgs')
    from ocs2_msgs.msg import WalkingVelocityCommand
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    context = Context()
    context.init(domain_id=DOMAIN_ID)
    suffix = 'run_' + uuid4().hex
    node_name = 'teleop_' + suffix
    joy_topic = '/teleop_test/' + suffix + '/joy'
    source_topic = joy_topic + '/raw' if bridge else joy_topic
    command_topic = '/teleop_test/' + suffix + '/command'
    # Simulation time with no /clock publisher: output must still flow on the steady timer.
    teleop_process(
        '-r', '__node:=' + node_name,
        '-p', 'joy_topic:=' + joy_topic,
        '-p', 'joy_source_topic:=' + (source_topic if bridge else source_topic.lstrip('/')),
        '-p', 'command_topic:=' + command_topic,
        '-p', 'use_sim_time:=true',
    )
    probe = Node('teleop_test_probe_' + suffix, context=context)
    publisher = probe.create_publisher(
        Joy, source_topic,
        QoSProfile(depth=1, reliability=(
            ReliabilityPolicy.RELIABLE if bridge else ReliabilityPolicy.BEST_EFFORT
        )),
    )
    received = []
    status = Status()
    forwarded = []
    probe.create_subscription(
        WalkingVelocityCommand, command_topic, received.append, qos_profile_sensor_data,
    )
    probe.create_subscription(
        String, '/' + node_name + '/status', status, qos_profile_sensor_data,
    )
    if bridge:
        probe.create_subscription(Joy, joy_topic, forwarded.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
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
        wait_for(executor, lambda: received and status.history
                 and publisher.get_subscription_count() >= 1, what='teleop startup')
        pump(0.5, [0.0] * 6)
        assert received and received[-1].linear_velocity_x == 0.0
        pump(0.1, [0.0] * 6, pressure=1.0)
        assert status.enabled
        pump(0.15, [0.0, 1.0, 0.5, 0.0, 0.0, 0.0], pressure=1.0)
        subscriptions = probe.get_subscriptions_info_by_topic(source_topic)
        teleop_subscription = [i for i in subscriptions if i.node_name == node_name]
        assert len(teleop_subscription) == 1
        assert teleop_subscription[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert received[-1].linear_velocity_x == 0.25
        assert received[-1].angular_velocity_z == 0.125
        assert received[-1].desired_pelvis_height == 0.6
        count = len(received)
        pump(0.4)
        assert len(received) > count
        assert received[-1].linear_velocity_x == 0.25, 'no input timeout: the last command holds'
        assert status.enabled
        pump(0.1, [0.0] * 6)
        assert not status.enabled
        assert received[-1].linear_velocity_x == 0.0
        pump(0.1, [0.0] * 6, pressure=1.0)
        assert status.enabled
        pump(0.1, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], pressure=1.0)
        assert received[-1].linear_velocity_x > 0
        pump(0.1, [0.0, 0.0, 0.0, 0.25, 0.0, 0.0], pressure=1.0)
        assert received[-1].desired_pelvis_height == 0.6, 'height stick deadband'
        pump(0.1, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], pressure=1.0)
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
        assert status.enabled
        assert status.last['phase'] == 'enabled'
        if bridge:
            assert forwarded and forwarded[-1].axes[4] == -1.0
            info = probe.get_publishers_info_by_topic(joy_topic)
            assert len(info) == 1
            assert info[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        else:
            assert probe.count_publishers(joy_topic) == 1, 'no forwarding without a bridge'
    finally:
        executor.shutdown()
        probe.destroy_node()
        context.shutdown()


def test_pitch_only_publishes_orientation_and_levels_outside_locomotion(teleop_process):
    pytest.importorskip('rclpy')
    from geometry_msgs.msg import Vector3
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    context = Context()
    context.init(domain_id=DOMAIN_ID)
    suffix = '/teleop_test/pitch_' + uuid4().hex
    teleop_process(
        '-p', 'joy_topic:=' + suffix + '/joy',
        '-p', 'orientation_topic:=' + suffix + '/orientation',
        '-p', 'posture_controller:=' + suffix + '/controller',
        '-p', 'maximum_pitch:=0.2', '-p', 'pitch_rate:=0.3',
        '-p', 'use_sim_time:=true',
    )
    probe = Node('pitch_probe_' + uuid4().hex, context=context)
    joy = probe.create_publisher(Joy, suffix + '/joy', qos_profile_sensor_data)
    phase = probe.create_publisher(
        String, suffix + '/controller/motion_state', qos_profile_sensor_data,
    )
    received = []
    probe.create_subscription(
        Vector3, suffix + '/orientation', received.append, qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)

    def pump(duration, pressure=0.0, pitch=0, center=False, state='locomotion'):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                message = Joy()
                message.axes = [0.0, 0.0, 0.0, 0.0, -pressure, 0.0]
                message.buttons = [int(i == 9 and center) for i in range(15)]
                if pitch:
                    message.buttons[11 if pitch > 0 else 12] = 1
                phase.publish(String(data=state))
                joy.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    try:
        wait_for(executor, lambda: received and joy.get_subscription_count(), what='pitch startup')
        pump(0.3)
        pump(0.1, pressure=1.0)
        pump(0.3, pressure=1.0, pitch=1.0)
        assert 0.03 < received[-1].y <= 0.2
        assert received[-1].x == received[-1].z == 0.0
        pump(0.1)
        held = received[-1].y
        pump(0.1)
        assert received[-1].y == held
        pump(0.1, center=True)
        assert received[-1].y == 0.0
        pump(0.1)
        pump(0.1, pressure=1.0)
        pump(0.3, pressure=1.0, pitch=-1.0)
        assert received[-1].y < -0.03
        pump(0.2, state='sitting down')
        assert received[-1].y == 0.0
        info = probe.get_publishers_info_by_topic(suffix + '/orientation')
        assert info[0].qos_profile.reliability == qos_profile_sensor_data.reliability
    finally:
        executor.shutdown()
        probe.destroy_node()
        context.shutdown()
