"""Launch a standard gamepad driver and robot-profiled base-command adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from legged_robot_teleop.profile import load_robot_profile


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    controllers_file = value('controllersFile')
    if not controllers_file:
        controllers_file = str(
            Path(get_package_share_directory('legged_robot_mpc_controller'))
            / 'config' / value('robotName') / 'ros2_controllers.yaml'
        )
    profile, lateral_enabled = load_robot_profile(
        controllers_file, value('mpcControllerName'), value('refBaseHeight'),
    )
    if not lateral_enabled:
        profile['scale_lateral'] = 0.0
    start_joy = IfCondition(LaunchConfiguration('startJoy')).evaluate(context)
    joy_topic = value('joyTopic')
    driver_topic = joy_topic.rstrip('/') + '/raw'
    return [
        Node(
            package='joy', executable='game_controller_node', name='game_controller_node',
            output='screen', condition=IfCondition(LaunchConfiguration('startJoy')),
            parameters=[{
                'device_id': ParameterValue(LaunchConfiguration('deviceId'), value_type=int),
                'device_name': value('deviceName'),
                'deadzone': 0.08,
                'autorepeat_rate': 50.0,
                'sticky_buttons': False,
            }],
            remappings=[('joy', driver_topic)],
        ),
        Node(
            package='legged_robot_teleop', executable='base_twist', name='base_twist_teleop',
            output='screen', parameters=[value('paramsFile'), profile, {
                'joy_topic': joy_topic,
                'joy_source_topic': driver_topic if start_joy else joy_topic,
                'command_topic': value('commandTopic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool,
                ),
            }],
        ),
    ]


def generate_launch_description():
    """Keep device choice, robot selection, and test topics independently configurable."""
    share = Path(get_package_share_directory('legged_robot_teleop'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'robotName', default_value='g1', choices=['g1', 'h2', 'upkie', 'pineapple'],
            description='Robot profile; reads heights from its controller YAML.',
        ),
        DeclareLaunchArgument('controllersFile', default_value=''),
        DeclareLaunchArgument(
            'mpcControllerName',
            default_value='humanoid_whole_body_inverse_dynamics_mpc_controller',
        ),
        DeclareLaunchArgument(
            'refBaseHeight', default_value='',
            description='Optional height override [m]; match the robot launch override.',
        ),
        DeclareLaunchArgument('paramsFile', default_value=str(share / 'config/dualsense.yaml')),
        DeclareLaunchArgument('deviceId', default_value='0'),
        DeclareLaunchArgument('deviceName', default_value='DualSense Wireless Controller'),
        DeclareLaunchArgument(
            'startJoy', default_value='true',
            description='False when an existing driver already publishes joyTopic.',
        ),
        DeclareLaunchArgument('joyTopic', default_value='/joy'),
        DeclareLaunchArgument('commandTopic', default_value='/humanoid/walking_velocity_command'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
