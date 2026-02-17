from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    gps_node = Node(
        package='roboboat_2026',
        executable='gps_node',
        output='screen'
    )

    teensy_node = Node(
        package='roboboat_2026',
        executable='teensy_node',
        output='screen'
    )

    launcher_node = Node(
        package='roboboat_2026',
        executable='launcher_node',
        output='screen'
    )

    led_node = Node(
        package='roboboat_2026',
        executable='led_node',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='ROS_LOG_DIR',
            value='/data/low_level_logs'
        ),
        gps_node,

        TimerAction(
            period=2.0,
            actions=[teensy_node]
        ),

        TimerAction(
            period=4.0,
            actions=[launcher_node]
        ),

        TimerAction(
            period=6.0,
            actions=[led_node]
        ),
    ])
