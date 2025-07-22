from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'serial_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [os.path.join(os.path.dirname(__file__), 'package.xml')]),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='submj',
    maintainer_email='submj@todo.todo',
    description='ROS2 serial communication driver with pub/sub',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = serial_driver.serial_node:main',
        ],
    },
)
