from setuptools import setup

package_name = 'nav_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxw',
    maintainer_email='mxw@todo.todo',
    description='FAST-LIO odometry to navigation TF bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lio_odom_bridge = nav_localization.lio_odom_bridge:main',
            'cloud_to_scan = nav_localization.cloud_to_scan:main',
        ],
    },
)
