import os
from glob import glob
from setuptools import setup

package_name = 'decision_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxw',
    maintainer_email='mxw@todo.todo',
    description='Custom navigation decision layer using Behavior Trees',
    license='Apache-2.0',
    # 删除了 tests_require 警告
    entry_points={
        'console_scripts': [
            'bt_main = decision_core.bt_main:main',
        ],
    },
)