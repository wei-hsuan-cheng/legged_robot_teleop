"""Exercise posture-button edges and interlocks through ROS services, without a robot."""

import time
from uuid import uuid4

from conftest import DOMAIN_ID, Status, wait_for
import pytest


@pytest.mark.parametrize('posture_button', [2, 10])
def test_posture_toggle_requires_release_and_safe_fresh_robot_state(
        posture_button, teleop_process):
    """A held or blocked button never becomes a delayed stand/sit request."""
    pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    context = Context()
    context.init(domain_id=DOMAIN_ID)
    suffix = uuid4().hex
    namespace = '/teleop_posture_test_' + suffix
    node_name = 'teleop_' + suffix
    args = [
        '-r', '__node:=' + node_name,
        '-p', 'joy_topic:=' + namespace + '/joy',
        '-p', 'command_topic:=' + namespace + '/command',
        '-p', 'posture_controller:=' + namespace,
        '-p', 'maximum_pitch:=0.2',
    ]
    if posture_button != 2:
        args += ['-p', 'button_posture:=%d' % posture_button]
    teleop_process(*args)
    probe = Node('posture_probe_' + suffix, context=context)
    calls = []

    def toggle(request, response):
        calls.append(request)
        response.success = True
        response.message = 'accepted'
        return response

    probe.create_service(Trigger, namespace + '/toggle_stand', toggle)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    state_publisher = probe.create_publisher(String, namespace + '/motion_state', qos)
    joy_publisher = probe.create_publisher(Joy, namespace + '/joy', qos)
    status = Status()
    probe.create_subscription(String, '/' + node_name + '/status', status, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)

    def pump(pressed=(), pressure=0.0, state='idle', forward=0.0, duration=0.15, height=0.0):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                state_publisher.publish(String(data=state))
                message = Joy()
                message.axes = [0.0, forward, 0.0, height, -pressure, 0.0]
                message.buttons = [int(i in pressed) for i in range(15)]
                joy_publisher.publish(message)
                next_send = time.monotonic() + 0.02
            executor.spin_once(timeout_sec=0.005)

    try:
        wait_for(executor, lambda: status.history
                 and joy_publisher.get_subscription_count() >= 1
                 and state_publisher.get_subscription_count() >= 1
                 and probe.count_clients(namespace + '/toggle_stand') >= 1,
                 what='teleop startup')
        pump((posture_button,), duration=0.5)
        assert not calls, 'A held posture button at connection is not a new request'
        pump()
        pump((10 if posture_button == 2 else 2,))
        assert not calls, 'The previous/other mapping must not request a posture'
        pump()
        pump((1,))
        assert not calls, 'Circle must not request a posture'
        pump((posture_button, 1))
        assert len(calls) == 1
        pump((posture_button, 1), duration=0.3)
        assert len(calls) == 1, 'Autorepeat must not toggle again'
        pump()
        pump((posture_button,), pressure=1.0)
        pump((posture_button,), pressure=0.0)
        assert len(calls) == 1, 'A blocked posture button is consumed, not queued'
        pump(height=0.4)
        pump((posture_button,), height=0.4)
        assert len(calls) == 1, 'Height stick outside the deadband must block stand/sit'
        for buttons, forward, state in (
                ((posture_button, 9), 0.0, 'idle'),
                ((posture_button, 11), 0.0, 'idle'), ((posture_button,), 1.0, 'idle'),
                ((posture_button,), 0.0, 'standing up'), ((posture_button,), 0.0, 'fallen')):
            # Robot state and Joy are independent DDS topics; deliver the new state
            # before checking the adapter's guard. The real server also validates its phase.
            pump(state=state)
            pump(buttons, state=state, forward=forward)
            assert len(calls) == 1
        pump(state='locomotion')
        pump((posture_button, 1), state='locomotion')
        assert len(calls) == 2
        pump(state='sitting down')
        pump(pressure=1.0, state='sitting down')
        assert not status.enabled
        pump(pressure=1.0, state='locomotion')
        assert not status.enabled, 'Held L2 must not enable at handover'
        pump(state='locomotion')
        pump(pressure=1.0, state='locomotion')
        assert status.enabled
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert not status.enabled
        assert status.reason == 'robot_state_timeout'
    finally:
        executor.shutdown()
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
    assert 'button_stop' not in config['base_twist_teleop']['ros__parameters']


def test_shipped_body_controls_use_dpad_tilt_and_thresholded_stick_height():
    """Keep body tilt and height bindings distinct in the installed configuration."""
    from pathlib import Path
    import yaml

    parameters = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/dualsense.yaml').read_text(),
    )['base_twist_teleop']['ros__parameters']
    assert (parameters['button_pitch_forward'], parameters['button_pitch_backward']) == (11, 12)
    assert (parameters['button_roll_left'], parameters['button_roll_right']) == (13, 14)
    assert parameters['axis_height'] == 3
    assert parameters['height_axis_threshold'] == 0.3
    assert 'axis_pitch' not in parameters
    assert 'button_height_up' not in parameters
