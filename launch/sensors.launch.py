from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Livox MID360 RViz launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch',
                'rviz_MID360_launch.py'
            )
        )
    )

    # DepthAI camera launch
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            )
        )
    )

    # GPS Python module
    gps_node = ExecuteProcess(
        cmd=['python3', '-m', 'API.GPS.gps_node'],
        output='screen'
    )

    return LaunchDescription([
        livox_launch,
        depthai_launch,
        gps_node
    ])
