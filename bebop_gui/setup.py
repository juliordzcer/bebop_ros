# Copyright 2025 Julio César Rodríguez
# Licensed under the Apache License, Version 2.0
# https://www.apache.org/licenses/LICENSE-2.0

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bebop_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jr',
    maintainer_email='hello@juliordz.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bebop_gui = bebop_gui.gui_node:main',
            'bebop_gui_one = bebop_gui.gui_node_one:main',
            'bebop_gui_swarm = bebop_gui.gui_swarm:main',
            'bebop_gui_swarm_no_cameras = bebop_gui.gui_swarm_no_cameras:main',
            'test_gui = bebop_gui.test:main', 
        ],
    },
)
