import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch real robot for LQR balancing.

    grace_interface → IMU + joint_states
    Madgwick filter → orientation
    pitch_monitor   → /robot_pitch

    Start balancing separately:
        ros2 launch grace_controller lqr_balancing.launch.py use_sim_time:=False
    """

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/grace_esp32',
        description='Serial port for ESP32')

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('grace_firmware'),
            'launch', 'hardware_interface.launch.py'),
        launch_arguments={
            'port': LaunchConfiguration('serial_port'),
            'command_source': 'direct',
        }.items())

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    imu2_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu2_madgwick_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'base_link',
        }],
        remappings=[
            ('imu/data_raw', '/imu2/data'),
            ('imu/data', '/imu/out'),
        ])

    pitch_monitor = Node(
        package='grace_controller',
        executable='pitch_monitor.py',
        name='pitch_monitor',
        output='screen',
        parameters=[{'use_sim_time': False}])

    return LaunchDescription([
        serial_port_arg,
        hardware_interface,
        joint_state_broadcaster_spawner,
        imu2_filter,
        pitch_monitor,
    ])
