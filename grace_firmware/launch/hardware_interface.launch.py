import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch robot_state_publisher and ros2_control_node for real robot hardware.
    
    Arguments:
    - port: Serial port for ESP32 (default: /dev/grace_esp32)
    - command_source: "ros2_control" or "direct" (default: ros2_control)
    """

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/grace_esp32',
        description='Serial port for ESP32'
    )
    
    command_source_arg = DeclareLaunchArgument(
        'command_source',
        default_value='ros2_control',
        description='Source of motor commands: "ros2_control" or "direct"'
    )

    # Use Command substitution to generate robot_description with xacro arguments
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("grace_description"),
                    "urdf",
                    "grace.urdf.xacro",
                ),
                " is_sim:=False",
                " port:=", LaunchConfiguration('port'),
                " command_source:=", LaunchConfiguration('command_source'),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("grace_controller"),
                "config",
                "grace_controllers.yaml",
            ),
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            command_source_arg,
            robot_state_publisher_node,
            controller_manager,
        ]
    )