import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Launch LQR balancing controller (standalone).

    Mode derived from use_sim_time:
      use_sim_time=True  → simulation (effort to /simple_effort_controller/commands)
      use_sim_time=False → hardware   (firmware units to /motor_torque_commands)

    When use_encoder_noise=true (default in sim), remaps /joint_states → /joint_states_noisy
    so the controller reads the noisy encoder data from hoverboard_encoder_simulator.
    """

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='True = simulation, False = hardware')

    use_observer_arg = DeclareLaunchArgument(
        'use_observer', default_value='False',
        description='Enable Luenberger observer for state estimation')

    use_encoder_noise_arg = DeclareLaunchArgument(
        'use_encoder_noise', default_value='false',
        description='Remap /joint_states to /joint_states_noisy (sim with encoder noise)')

    balance_mode_arg = DeclareLaunchArgument(
        'is_balance_only', default_value='True',
        description='True for balance-only mode')

    csv_log_path_arg = DeclareLaunchArgument(
        'csv_log_path',
        default_value=os.path.join(os.path.expanduser('~'),
                                   'balancing_ws/src/LQR_Calculations/balancing_log.csv'),
        description='Path to CSV log file')

    common_params = [{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'use_observer': LaunchConfiguration('use_observer'),
        'is_balance_only': LaunchConfiguration('is_balance_only'),
        'csv_log_path': LaunchConfiguration('csv_log_path'),
    }]

    # With encoder noise remap
    lqr_with_noise = Node(
        package='grace_controller',
        executable='lqr_balancing_controller.py',
        name='lqr_balancing_controller',
        output='screen',
        parameters=common_params,
        remappings=[('/joint_states', '/joint_states_noisy')],
        condition=IfCondition(LaunchConfiguration('use_encoder_noise')))

    # Without encoder noise (or hardware mode)
    lqr_without_noise = Node(
        package='grace_controller',
        executable='lqr_balancing_controller.py',
        name='lqr_balancing_controller',
        output='screen',
        parameters=common_params,
        condition=UnlessCondition(LaunchConfiguration('use_encoder_noise')))

    return LaunchDescription([
        use_sim_time_arg,
        use_observer_arg,
        use_encoder_noise_arg,
        balance_mode_arg,
        csv_log_path_arg,
        lqr_with_noise,
        lqr_without_noise,
    ])
