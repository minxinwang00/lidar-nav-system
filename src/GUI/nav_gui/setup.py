from setuptools import setup

package_name = 'nav_gui'

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
    description='Qt5 console for the navigation decision layer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_gui = nav_gui.gui_node:main',
        ],
    },
)
