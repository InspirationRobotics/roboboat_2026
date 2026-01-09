from setuptools import find_packages, setup
from glob import glob
package_name = 'roboboat_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chase Chen',
    maintainer_email='chase001cz@gmail.com',
    description='Team Inspiraiton roboboat 2026 repo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_node = roboboat_2026.test_pub:main',
            'gps_node = roboboat_2026.api.gps.gps_node:main',
            'teensy_node = roboboat_2026.api.motors.teensy:main'
            'launcher_node = roboboat_2026.api.servos.ball_launcher:main',
            'lidar_costmap = roboboat_2026.mapping.lidar_costmap:main',
            'odom_node = roboboat_2026.navigation.odom_node:main',
            'ekf_node = roboboat_2026.navigation.ekf:main',
        ],
    },
)
