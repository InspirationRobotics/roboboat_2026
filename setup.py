from setuptools import setup

package_name = 'roboboat_2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'API',
        'API.GPS',
        'tf',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chase Chen',
    maintainer_email='Chase001cz@gmail.com',
    description='RoboBoat 2026 GPS and TF nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            # GPS
            'gps_node = API.GPS.gps_node:main',

            # TF
            'tf_pub = tf.tf_pub:main',
        ],
    },
)
