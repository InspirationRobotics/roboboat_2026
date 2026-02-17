from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # gps node
    gps_node = Node(
        package='roboboat_2026',
        executable='gps_node',
        output='screen'
    )

    # Teensy node
    teensy_node = Node(
        package='roboboat_2026',
        executable='teensy_node',
        output='screen'
    )

    # launcher node
    launcher_node = Node(
        package='roboboat_2026',
        executable='launcher_node',
        output='screen'
    )

    # led node
    led_node = Node(
        package='roboboat_2026',
        executable='led_node',
        output='screen'
    )

    return LaunchDescription([
        gps_node,
        teensy_node,
        launcher_node,
        led_node
    ])
