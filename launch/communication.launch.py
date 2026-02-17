from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import SetEnvironmentVariable
import os


def generate_launch_description():


    # gps node
    ivc_node = Node(
        package='roboboat_2026',
        executable='ivc_node',
        output='screen'
    )

    report_node = Node(
        package='roboboat_2026',
        executable='report_node',
        output='screen'
    )
    


    return LaunchDescription([
    SetEnvironmentVariable(
        name='ROS_LOG_DIR',
        value='/data/comm_logs'
    ),
        ivc_node,

        TimerAction(
            period=2.0,
            actions=[report_node]
        ),
    ])
