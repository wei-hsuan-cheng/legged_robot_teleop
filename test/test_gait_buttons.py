"""Exercise the gait buttons (Circle cycles, Triangle holds the jump) over ROS, without a robot."""

import time
from uuid import uuid4

from conftest import DOMAIN_ID, Status, wait_for
import pytest


def test_gait_buttons_cycle_hold_and_reset_outside_locomotion(teleop_process):
    """Circle steps walk -> trot -> ladder, Triangle jumps while held, and only in locomotion."""
    pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    context = Context()
    context.init(domain_id=DOMAIN_ID)
    suffix = uuid4().hex
    namespace = '/teleop_gait_test_' + suffix
    node_name = 'teleop_' + suffix
    teleop_process(
        '-r', '__node:=' + node_name,
        '-p', 'joy_topic:=' + namespace + '/joy',
        '-p', 'command_topic:=' + namespace + '/command',
        '-p', 'posture_controller:=' + namespace,
        '-p', 'gait_topic:=' + namespace + '/gait',
        '-p', 'gait_cycle:=walk,trot',
        '-p', 'gait_hold:=jump',
    )
    probe = Node('gait_probe_' + suffix, context=context)
    gaits = []
    probe.create_subscription(String, namespace + '/gait', lambda m: gaits.append(m.data),
                              QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    state_publisher = probe.create_publisher(String, namespace + '/motion_state', qos)
    joy_publisher = probe.create_publisher(Joy, namespace + '/joy', qos)
    status = Status()
    probe.create_subscription(String, '/' + node_name + '/status', status, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)

    def pump(pressed=(), state='locomotion', duration=0.15):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                state_publisher.publish(String(data=state))
                message = Joy()
                message.axes = [0.0] * 6
                message.buttons = [int(i in pressed) for i in range(15)]
                joy_publisher.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    def latest():
        return gaits[-1] if gaits else None

    try:
        wait_for(executor, lambda: status.history and joy_publisher.get_subscription_count() >= 1
                 and state_publisher.get_subscription_count() >= 1
                 and probe.count_publishers(namespace + '/gait') >= 1, what='teleop startup')
        pump((1, 3), duration=0.4)
        assert latest() in (None, ''), 'Buttons held at connection request nothing'
        pump()
        pump((1,))
        assert latest() == 'walk'
        pump((1,), duration=0.4)
        assert latest() == 'walk', 'A held Circle steps once'
        pump()
        pump((1,))
        assert latest() == 'trot'
        pump((3,))
        assert latest() == 'jump'
        pump()
        assert latest() == 'trot', 'Releasing Triangle returns to the Circle selection'
        pump((1,))
        pump()
        assert latest() == '', 'The third Circle returns to the ladder'
        pump((1,))
        pump(state='sitting down')
        assert latest() == '', 'Leaving locomotion returns to the ladder'
        pump((3,), state='sitting down')
        pump((3,), state='locomotion')
        assert latest() == '', 'Triangle held through the transition must be released first'
        pump()
        pump((3,))
        assert latest() == 'jump'
    finally:
        executor.shutdown()
        probe.destroy_node()
        context.shutdown()


def test_shipped_gait_mapping_is_circle_and_triangle():
    """Keep the launch YAML and the intended DualSense buttons consistent."""
    from pathlib import Path
    import yaml

    parameters = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/dualsense.yaml').read_text(),
    )['base_twist_teleop']['ros__parameters']
    assert (parameters['button_gait_cycle'], parameters['button_gait_hold']) == (1, 3)
