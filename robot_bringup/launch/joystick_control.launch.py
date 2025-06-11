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
            {'deadzone': 0.05},  # Reduced deadzone
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
            # Updated joystick mapping parameters
            {'axis_linear.x': 1},        # Left stick vertical (forward/backward)
            {'axis_linear.y': 0},        # Left stick horizontal (strafe left/right)
            {'axis_angular.yaw': 2},     # Right stick horizontal (rotate) - changed from 3 to 2
            {'scale_linear': 0.5},       # Reduced for better control
            {'scale_linear_turbo': 1.0},
            {'scale_angular': 1.0},
            {'scale_angular_turbo': 2.0},
            {'enable_button': 4},        # L1/LB button - changed from 0 to 4
            {'enable_turbo_button': 5},  # R1/RB button - changed from 1 to 5
            {'require_enable_button': True},  # Must hold enable button
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