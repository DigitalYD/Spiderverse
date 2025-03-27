from setuptools import setup
import os
from glob import glob

package_name = 'lidar_udp_receiver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asher',
    maintainer_email='quiqui.goodwin@gmail.com',
    description='ROS2 node for receiving LIDAR data over UDP',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_udp_receiver = lidar_udp_receiver.lidar_udp_receiver:main',
            'tf_broadcaster = lidar_udp_receiver.tf_broadcaster:main',
        ],
    },
)