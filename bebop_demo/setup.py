from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bebop_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'models'), glob('models/****/***/**/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
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
            "position_control = bebop_demo.position_control:main",
            "joystick = bebop_demo.joystick:main",
            "setpoint = bebop_demo.setpoint:main",
            "setpoint_followers = bebop_demo.setpoint_followers:main",
            "joystick_swarm = bebop_demo.joystick_swarm:main",
            "graficas = bebop_demo.graficas:main",
            "imagenes = bebop_demo.image_viewer:main",
            "set_pose = bebop_demo.set_pose:main",
        ],
    },
)
