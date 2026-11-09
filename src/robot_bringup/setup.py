from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'models/diff_bot'), glob('models/diff_bot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christopher-wilson',
    maintainer_email='christopher-wilson@todo.todo',
    description='TurtleBot3 Manipulation simulation bringup and task management for Intelligent Robotics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = robot_bringup.task_manager:main',
            'test_robot = robot_bringup.test_robot:main',
            'vacuum_demo = robot_bringup.vacuum_demo:main',
        ],
    },
)
