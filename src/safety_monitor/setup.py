from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'safety_monitor'

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
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Real-time motion command validation and 200ms E-STOP for the G1 Onboard.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor_node = safety_monitor.safety_monitor_node:main',
        ],
    },
)
