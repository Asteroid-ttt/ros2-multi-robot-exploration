import os
from glob import glob
from setuptools import setup

package_name = 'car_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['README.md', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 bridge between Nav2 and serial-controlled robot car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册可执行节点：ros2 run car_interface car_interface_node
            'car_interface_node = car_interface.car_interface_node:main',
        ],
    },
)
