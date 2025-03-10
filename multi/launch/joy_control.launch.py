import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=EnvironmentVariable('MASTER'),
        description='Namespace for the robot (e.g. robot_master)'
    )
    remap_cmd_vel_arg = DeclareLaunchArgument(
        'remap_cmd_vel', default_value='/controller/cmd_vel',
        description='Remap topic for cmd_vel'
    )

    # Get the values of launch arguments
    namespace = LaunchConfiguration('namespace')
    remap_cmd_vel = LaunchConfiguration('remap_cmd_vel')

    # Path to the joystick control launch file
    joystick_control_launch_file = os.path.join(
        get_package_share_directory('peripherals'),
        'launch',
        'joystick_control.launch.py'
    )

    # Grouping actions under a namespace
    robot_namespace_group = GroupAction(
        actions=[
            # Push the namespace
            PushRosNamespace(namespace),
            # Include joystick control launch file with arguments
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(joystick_control_launch_file),
                launch_arguments={
                    'max_linear': '0.2',
                    'max_angular': '0.5',
                    'remap_cmd_vel': remap_cmd_vel
                }.items(),
            ),
        ]
    )

    # Return the complete launch description
    return LaunchDescription([
        namespace_arg,
        remap_cmd_vel_arg,
        robot_namespace_group,
    ])

