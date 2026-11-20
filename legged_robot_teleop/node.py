"""Publish the same normalized base command message as the MPC GUI."""

from dataclasses import fields
import time

from legged_robot_teleop.control import Settings, TeleopControl
from ocs2_msgs.msg import WalkingVelocityCommand
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class BaseTwistTeleop(Node):
    """Convert fresh Joy messages to bounded base commands on a steady timer."""

    def __init__(self, **kwargs):
        super().__init__('base_twist_teleop', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('joy_topic', '/joy', descriptor)
        self.declare_parameter('command_topic', '/humanoid/walking_velocity_command', descriptor)
        defaults = Settings()
        values = {}
        for field in fields(Settings):
            values[field.name] = self.declare_parameter(
                field.name, getattr(defaults, field.name), descriptor,
            ).value
        self.control = TeleopControl(Settings(**values))
        self._last_state = None
        self._publisher = self.create_publisher(
            WalkingVelocityCommand,
            self.get_parameter('command_topic').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._subscription = self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self._receive, qos_profile_sensor_data,
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
        self.control.receive(message.axes, message.buttons, time.monotonic())

    def _publish(self):
        command = self.control.command(time.monotonic())
        message = WalkingVelocityCommand()
        message.linear_velocity_x = command.forward
        message.linear_velocity_y = command.lateral
        message.angular_velocity_z = command.yaw
        message.desired_pelvis_height = command.height
        self._publisher.publish(message)
        if self.control.state != self._last_state:
            self.get_logger().info(f'Teleop: {self.control.state}')
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
