import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    master_arg = DeclareLaunchArgument('master', default_value=EnvironmentVariable('MASTER'))
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=EnvironmentVariable('HOST'))
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/controller/cmd_vel')

    # Get the values of the launch arguments
    master = LaunchConfiguration('master')
    robot_name = LaunchConfiguration('robot_name')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # Define namespace group
    robot_namespace_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),

            # Include the hiwonder_controller launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('controller'), 'launch', 'controller.launch.py')
                ),
                launch_arguments={'cmd_vel': cmd_vel_topic}.items(),
            ),

            # Define the init_pose node

        ]
    )

    # Include joy_control.launch.py if master == robot_name
    joy_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('multi'), 'launch', 'joy_control.launch.py')
        ),
        condition=IfCondition(PythonExpression(['"', master, '" == "', robot_name, '"']))
    )

    return LaunchDescription([
        master_arg,
        robot_name_arg,
        cmd_vel_topic_arg,
        robot_namespace_group,
        joy_control_include,
    ])

