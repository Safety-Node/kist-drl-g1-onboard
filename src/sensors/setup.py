from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Onboard sensor publishers for Unitree G1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mic_node         = sensors.mic_node:main',
            'speaker_node     = sensors.speaker_node:main',
            'joint_state_node = sensors.joint_state_node:main',
            'imu_node         = sensors.imu_node:main',
            'uwb_node         = sensors.uwb_node:main',
            'odom_node        = sensors.odom_node:main',
            'location_node    = sensors.location_node:main',
            'lidar_node        = sensors.lidar_node:main',
            'obstacle_map_node = sensors.obstacle_map_node:main',
        ],
    },
)
