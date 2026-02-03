from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():


    # Lidar projection
    lidar_costmap_node = Node(
        package='roboboat_2026',
        namespace='lidarprojection',
        executable='lidar_costmap'
    )

    #

    return LaunchDescription([
        lidar_costmap_node,
    ])

