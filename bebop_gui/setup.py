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
    maintainer_email='juliordzcer@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bebop_gui = bebop_gui.gui_node:main',
            'bebop_one = bebop_gui.gui_one:main',
            'test_gui = bebop_gui.test:main', 
        ],
    },
)
