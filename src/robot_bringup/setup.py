from glob import glob
import os
from setuptools import find_packages, setup

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
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christopher-wilson',
    maintainer_email='christopher-wilson@todo.todo',
    description='Unified TurtleBot3 Manipulation home simulation, SLAM mapping, autonomous navigation, and room task execution package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'alpha_cleaning = robot_bringup.alpha_cleaning:main',
            'beta_pick_place = robot_bringup.beta_pick_place:main',
            'doorway_navigator = robot_bringup.doorway_navigator:main',
            'frontier_explorer = robot_bringup.frontier_explorer:main',
            'save_map = robot_bringup.save_map:main',
            'task_manager = robot_bringup.task_manager:main',
            'test_robot = robot_bringup.test_robot:main',
            'vacuum_demo = robot_bringup.vacuum_demo:main',
        ],
    },
)
