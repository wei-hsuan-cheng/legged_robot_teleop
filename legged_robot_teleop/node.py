"""Publish the same normalized base command message as the MPC GUI."""

from dataclasses import fields
import json
import time

from legged_robot_teleop.control import EnableState, Settings, TeleopControl
from ocs2_msgs.msg import WalkingVelocityCommand
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BaseTwistTeleop(Node):
    """Convert fresh Joy messages to bounded base commands on a steady timer."""

    def __init__(self, **kwargs):
        super().__init__('base_twist_teleop', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('joy_topic', '/joy', descriptor)
        self.declare_parameter('joy_source_topic', '', descriptor)
        self.declare_parameter('command_topic', '/humanoid/walking_velocity_command', descriptor)
        posture_controller = self.declare_parameter('posture_controller', '', descriptor).value
        self._posture_button = self.declare_parameter('button_posture', 10, descriptor).value
        if self._posture_button < 0:
            raise ValueError('button_posture must be nonnegative')
        self._posture_button_held = None
        self._last_joy_receipt = None
        self._posture_future = None
        self._posture_request_time = 0.0
        self._motion_state = 'unknown'
        self._motion_state_time = 0.0
        self._posture_client = None
        if posture_controller:
            namespace = posture_controller.rstrip('/')
            self._posture_client = self.create_client(Trigger, namespace + '/toggle_stand')
            self.create_subscription(
                String, namespace + '/motion_state', self._receive_motion_state,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        defaults = Settings()
        values = {}
        for field in fields(Settings):
            values[field.name] = self.declare_parameter(
                field.name, getattr(defaults, field.name), descriptor,
            ).value
        self.control = TeleopControl(Settings(**values))
        self._last_state = None
        self._last_status_time = 0.0
        self._publisher = self.create_publisher(
            WalkingVelocityCommand,
            self.get_parameter('command_topic').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        joy_topic = self.get_parameter('joy_topic').value
        source_topic = self.get_parameter('joy_source_topic').value or joy_topic
        self._joy_publisher = None
        if self.resolve_topic_name(source_topic) != self.resolve_topic_name(joy_topic):
            self._joy_publisher = self.create_publisher(
                Joy, joy_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        self._subscription = self.create_subscription(
            Joy, source_topic, self._receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._status_publisher = self.create_publisher(
            String, '~/status', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / self.control.settings.publish_rate, self._publish, clock=self._steady_clock,
        )
        self.get_logger().info(
            f'Output: {self._publisher.topic_name}. Disable other command publishers. '
            'Release the enable trigger, center sticks, then hold it to drive '
            '(DualSense default: L2). Circle stops motion.',
        )

    def _receive(self, message):
        if self._joy_publisher is not None:
            self._joy_publisher.publish(message)
        now = time.monotonic()
        if (self._last_joy_receipt is None
                or now - self._last_joy_receipt >= self.control.settings.input_timeout):
            self._posture_button_held = None
        self._last_joy_receipt = now
        self.control.receive(message.axes, message.buttons, now)
        if self._posture_client is not None:
            self._receive_posture_button(message)
            if self._motion_state != 'locomotion':
                self.control.stop('robot_' + self._motion_state)

    def _receive_motion_state(self, message):
        self._motion_state = message.data
        self._motion_state_time = time.monotonic()
        if message.data != 'locomotion':
            self.control.stop('robot_' + message.data)

    def _receive_posture_button(self, message):
        if (self.control.state in ('invalid_input', 'invalid_time')
                or len(message.buttons) <= self._posture_button):
            self._posture_button_held = None
            return
        held = bool(message.buttons[self._posture_button])
        edge = self._posture_button_held is False and held
        self._posture_button_held = held
        s = self.control.settings
        if not edge or self.control.last_trigger_pressure > s.release_threshold:
            return
        blocking_buttons = (
            s.button_stop, s.button_center, s.button_height_up, s.button_height_down,
        )
        if any(message.buttons[i] for i in blocking_buttons if i >= 0):
            return
        if any(abs(message.axes[i]) > s.neutral_threshold
               for i in (s.axis_forward, s.axis_lateral, s.axis_yaw) if i >= 0):
            return
        if self._motion_state not in ('idle', 'locomotion'):
            return
        if time.monotonic() - self._motion_state_time > 0.5:
            return
        if self._posture_future is not None and not self._posture_future.done():
            return
        if not self._posture_client.service_is_ready():
            self.get_logger().warning('Stand/sit service is unavailable; release R1 and retry.')
            return
        self.control.stop('posture_requested')
        self._posture_request_time = time.monotonic()
        self._posture_future = self._posture_client.call_async(Trigger.Request())
        self._posture_future.add_done_callback(self._posture_response)

    def _posture_response(self, future):
        if not future.cancelled():
            if future.exception() is not None:
                self.get_logger().error(str(future.exception()))
            else:
                self.get_logger().info(future.result().message)

    def _publish(self):
        now = time.monotonic()
        if (self._posture_future is not None and not self._posture_future.done()
                and now - self._posture_request_time > 2.0):
            self._posture_future.cancel()
            self.get_logger().warning(
                'Stand/sit request timed out; check robot state before retrying.',
            )
        if self._posture_client is not None and now - self._motion_state_time > 0.5:
            self._motion_state = 'unknown'
            self.control.stop('robot_state_timeout')
        command = self.control.command(now)
        message = WalkingVelocityCommand()
        message.linear_velocity_x = command.forward
        message.linear_velocity_y = command.lateral
        message.angular_velocity_z = command.yaw
        message.desired_pelvis_height = command.height
        self._publisher.publish(message)
        changed = self.control.state != self._last_state
        if changed or now - self._last_status_time >= 1.0:
            status = String()
            status.data = json.dumps({
                'phase': self.control.enable_state.value,
                'reason': self.control.state,
                'enabled': self.control.enabled,
                'last_trigger_pressure': self.control.last_trigger_pressure,
            })
            self._status_publisher.publish(status)
            self._last_status_time = now
        if changed:
            if self._motion_state in ('fallen', 'FAULT (damping)'):
                hint = (
                    ' Robot is latched; restore a supported idle pose '
                    'before resetting the controller.'
                )
            elif self._posture_client is not None and self._motion_state != 'locomotion':
                hint = ' Wait for locomotion before enabling L2.'
            else:
                hint = (
                    ' Release L2, center sticks, release L1/D-pad, then press L2 again.'
                    if (self.control.enable_state is EnableState.WAIT_RELEASE
                        and self.control.state != 'waiting_for_input') else ''
                )
            self.get_logger().info(
                f'Teleop: {self.control.state} '
                f'(phase={self.control.enable_state.value}, '
                f'L2={self.control.last_trigger_pressure:.2f}).{hint}',
            )
            self._last_state = self.control.state


def main(args=None):
    """Run until interrupted; zeroing on input loss is not a hardware E-stop."""
    rclpy.init(args=args)
    node = None
    try:
        node = BaseTwistTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.control.stop('shutdown')
            if rclpy.ok():
                node._publish()
            node.destroy_node()
        rclpy.try_shutdown()
