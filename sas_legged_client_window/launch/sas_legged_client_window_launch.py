"""
Starts the window of sas_legged_client_window for a legged robot driver of sas_tools.

    ros2 launch sas_legged_client_window sas_legged_client_window_launch.py
    ros2 launch sas_legged_client_window sas_legged_client_window_launch.py legged_robot_prefix:=sas_g1/g1_1
    ros2 launch sas_legged_client_window sas_legged_client_window_launch.py mode:=monitor own_driver_limb_prefixes:="['sas_z1/z1_1']"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='sas_legged_client_window',
                              description='Name of the node: give each window its own name to run several.'),
        DeclareLaunchArgument('legged_robot_prefix', default_value='sas_b1/b1_1',
                              description='Topic prefix of the legged robot driver, e.g. sas_g1/g1_1.'),
        DeclareLaunchArgument('own_driver_limb_prefixes', default_value='[]',
                              description="Limbs with their own driver, e.g. \"['sas_z1/z1_1']\"."),
        DeclareLaunchArgument('mode', default_value='control',
                              description='"control" (commands the robot) or "monitor" (only reads it).'),
        DeclareLaunchArgument('watchdog_period_in_seconds', default_value='1.0'),
        DeclareLaunchArgument('watchdog_maximum_acceptable_delay_in_seconds', default_value='0.5'),
        DeclareLaunchArgument('shutdown_topic', default_value='/sas/set/shutdown',
                              description='The emergency stop also publishes on it (empty string to disable).'),
        Node(
            package='sas_legged_client_window',
            executable='sas_legged_client_window',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'legged_robot_prefix': ParameterValue(LaunchConfiguration('legged_robot_prefix'), value_type=str),
                'own_driver_limb_prefixes': ParameterValue(LaunchConfiguration('own_driver_limb_prefixes'), value_type=list[str]),
                'mode': ParameterValue(LaunchConfiguration('mode'), value_type=str),
                'watchdog_period_in_seconds': ParameterValue(LaunchConfiguration('watchdog_period_in_seconds'), value_type=float),
                'watchdog_maximum_acceptable_delay_in_seconds':
                    ParameterValue(LaunchConfiguration('watchdog_maximum_acceptable_delay_in_seconds'), value_type=float),
                'shutdown_topic': ParameterValue(LaunchConfiguration('shutdown_topic'), value_type=str),
            }]
        ),
    ])
