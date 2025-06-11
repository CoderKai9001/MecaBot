import datetime
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Launch in simulation mode.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='namespace'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'record',
            default_value='False',
            description='Record in rosbag'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Launch RVIZ on startup'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joystick',
            default_value='True',
            description='Enable joystick control'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'))
    declared_arguments.append(
        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'))

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    use_rviz = LaunchConfiguration('use_rviz')
    use_joystick = LaunchConfiguration('use_joystick')
    joy_dev = LaunchConfiguration('joy_dev')

    # Package Path
    package_path = get_package_share_directory('robot_bringup')

    # Include the main robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_path, '/launch/robot_bringup.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'record': record,
            'use_rviz': use_rviz,
        }.items(),
    )

    # Include joystick control
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_path, '/launch/joystick_control.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'joy_dev': joy_dev,
        }.items(),
        condition=IfCondition(use_joystick),
    )

    nodes = [
        robot_launch,
        joystick_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
