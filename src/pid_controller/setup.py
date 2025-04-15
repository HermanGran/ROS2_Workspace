from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', ['config/parameters.yaml']),
    ('share/' + package_name + '/launch', ['launch/launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsjen',
    maintainer_email='matsjen@todo.todo',
    description='ROS2 PID Controller Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = pid_controller.pid_controller_node:main',
        ],
    },
)
