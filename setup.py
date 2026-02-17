from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'roboboat_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),

        # Action files
        ('share/' + package_name + '/action',
         glob('roboboat_2026/action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chase Chen',
    maintainer_email='chase001cz@gmail.com',
    description='Team Inspiration RoboBoat 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_node = roboboat_2026.test_pub:main',
            'gps_node = roboboat_2026.api.gps.gps_node:main',
            'teensy_node = roboboat_2026.api.motors.teensy:main',
            'report_node = roboboat_2026.api.report.rb_client:main',
            'ivc_node = roboboat_2026.api.ivc.ivc_node:main'
            'launcher_node = roboboat_2026.api.servos.ball_launcher:main',
            'led_node = roboboat_2026.api.led.led_node:main',
            'lidar_costmap = roboboat_2026.mapping.lidar_costmap:main',
            'odom_node = roboboat_2026.navigation.odom_node:main',
            'ekf_node = roboboat_2026.navigation.ekf:main',
            'ftp_node = roboboat_2026.missions.ftp:main',
        ],
    },
)
