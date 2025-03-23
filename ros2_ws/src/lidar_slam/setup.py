from setuptools import setup

package_name = 'lidar_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asher',
    maintainer_email='user@example.com',
    description='LIDAR SLAM package for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_udp_receiver=lidar_slam.lidar_udp_receiver:main',
        ],
    },
)   