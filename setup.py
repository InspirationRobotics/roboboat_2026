from setuptools import setup

package_name = 'roboboat_2026'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # This is the key part
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        ('share/' + package_name + '/launch', ['launch/sensors.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='...',
    license='License',
    entry_points={
        'console_scripts': [

        ],
    },
)
