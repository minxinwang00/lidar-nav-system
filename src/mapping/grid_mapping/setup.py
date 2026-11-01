import os
from glob import glob
from setuptools import setup

package_name = 'grid_mapping'

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
    description='3D LiDAR point cloud to 2D occupancy grid projection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pointcloud_to_grid = grid_mapping.pointcloud_to_grid:main',
        ],
    },
)
