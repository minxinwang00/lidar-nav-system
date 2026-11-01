import os
from glob import glob
from setuptools import setup

package_name = 'li_init_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxw',
    maintainer_email='mxw@todo.todo',
    description='LI-Init calibration workflow tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'li_init_manager = li_init_tools.li_init_manager:main',
            'apply_li_init = li_init_tools.apply_li_init:main',
        ],
    },
)
