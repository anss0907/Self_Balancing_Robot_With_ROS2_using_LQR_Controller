import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch simulation environment (no balancing controller).

    Gazebo + joint_state_broadcaster + effort controller + pitch monitor
    + optional encoder noise.

    Start balancing separately:
        ros2 launch grace_controller lqr_balancing.launch.py use_sim_time:=True
    """

    use_encoder_noise_arg = DeclareLaunchArgument(
        'use_encoder_noise', default_value='false',
        description='Enable realistic hoverboard encoder simulation')

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('grace_description'),
            'launch', 'gazebo.launch.py'))

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    simple_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_effort_controller', '--controller-manager', '/controller_manager'])

    pitch_monitor = Node(
        package='grace_controller',
        executable='pitch_monitor.py',
        name='pitch_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}])

    encoder_simulator = Node(
        package='grace_controller',
        executable='hoverboard_encoder_simulator.py',
        name='hoverboard_encoder_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'encoder_resolution': 360,
            'position_noise_stddev': 0.001,
            'velocity_noise_stddev': 0.05,
            'velocity_filter_alpha': 0.7,
            'enable_quantization': True,
            'enable_noise': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_encoder_noise')))

    return LaunchDescription([
        use_encoder_noise_arg,
        gazebo,
        joint_state_broadcaster_spawner,
        simple_effort_controller_spawner,
        pitch_monitor,
        encoder_simulator,
    ])
