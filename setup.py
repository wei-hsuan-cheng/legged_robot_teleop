"""Install the teleoperation node, launch file, and controller mapping."""

from glob import glob

from setuptools import find_packages, setup


PACKAGE = 'legged_robot_teleop'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wei-Hsuan Cheng',
    maintainer_email='johnathancheng0125@gmail.com',
    description='Gamepad velocity and base-height commands for legged robot MPC.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['base_twist = legged_robot_teleop.node:main'],
    },
)
