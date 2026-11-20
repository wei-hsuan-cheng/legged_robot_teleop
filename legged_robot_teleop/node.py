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


class BaseTwistTeleop(Node):
    """Convert fresh Joy messages to bounded base commands on a steady timer."""

    def __init__(self, **kwargs):
        super().__init__('base_twist_teleop', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('joy_topic', '/joy', descriptor)
        self.declare_parameter('joy_source_topic', '', descriptor)
        self.declare_parameter('command_topic', '/humanoid/walking_velocity_command', descriptor)
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
        self.control.receive(message.axes, message.buttons, time.monotonic())

    def _publish(self):
        now = time.monotonic()
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
