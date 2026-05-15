from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'navigation'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Point-to-point UWB navigator (goto_node) — Nav2 stack removed.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_node = navigation.goto_node:main',
        ],
    },
)
