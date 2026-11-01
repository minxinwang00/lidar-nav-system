import os
from glob import glob
from setuptools import setup

package_name = 'chassis'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxw',
    maintainer_email='mxw@todo.todo',
    description='Serial bridge between /cmd_vel and the wheel controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'chassis_bridge = chassis.chassis_bridge:main',
            'chassis_serial_probe = chassis.serial_probe:main',
            'chassis_motion_test = chassis.motion_test:main',
            'chassis_rotation_sweep = chassis.rotation_sweep:main',
        ],
    },
)
