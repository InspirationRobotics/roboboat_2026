import launch, launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='roboboat_2026',
            name='tf_node',
            executable='tf_node',
        ),
        Node(
            package='roboboat_2026',
            name='ekf_node',
            executable='ekf_node',
        )


    ])

