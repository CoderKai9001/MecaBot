from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for nodes'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device'))

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')

    # Joy node - publishes joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'dev': joy_dev},
            {'deadzone': 0.3},
            {'autorepeat_rate': 20.0},
        ],
        output='screen'
    )

    # Teleop twist joy node - converts joystick input to cmd_vel
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            # Joystick mapping parameters
            {'axis_linear.x': 1},      # Left stick vertical
            {'axis_linear.y': 0},      # Left stick horizontal  
            {'axis_angular.yaw': 3},   # Right stick horizontal
            {'scale_linear': 1.0},
            {'scale_linear_turbo': 2.0},
            {'scale_angular': 2.0},
            {'scale_angular_turbo': 4.0},
            {'enable_button': 0},      # A button (Xbox) / X button (PS4)
            {'enable_turbo_button': 1}, # B button (Xbox) / Circle button (PS4)
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ],
        output='screen'
    )

    nodes = [
        joy_node,
        teleop_twist_joy_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
