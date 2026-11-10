"""Spawn the TurtleBot3 Manipulation robot (robot_bringup) in the home world.

    ros2 launch home_nav_sim robot_in_home.launch.py                  # with walking humans
    ros2 launch home_nav_sim robot_in_home.launch.py humans:=false    # no humans (for SLAM)
    ros2 launch home_nav_sim robot_in_home.launch.py gui:=false       # headless

The Gazebo server and GUI run as two separate processes, so the robot's GPU
lidar and RGB-D camera don't share an Ogre2 renderer with the GUI window.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _robot(context, *args, **kwargs):
    humans = LaunchConfiguration('humans').perform(context).lower() in ('true', '1', 'yes')
    world = os.path.join(get_package_share_directory('home_nav_sim'), 'worlds',
                         'home.sdf' if humans else 'home_no_humans.sdf')
    bringup = os.path.join(get_package_share_directory('robot_bringup'),
                           'launch', 'simulation.launch.py')
    # robot_bringup passes `world` into gz_args, so "-s" starts only the server.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup),
        launch_arguments={
            'world': f'{world} -s',
            'x_pose': LaunchConfiguration('x_pose').perform(context),
            'y_pose': LaunchConfiguration('y_pose').perform(context),
            'start_rviz': LaunchConfiguration('start_rviz').perform(context),
        }.items(),
    )]


def generate_launch_description():
    gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen',
                         condition=IfCondition(LaunchConfiguration('gui')))
    return LaunchDescription([
        DeclareLaunchArgument('humans', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('x_pose', default_value='19.21'),
        DeclareLaunchArgument('y_pose', default_value='0.79'),
        OpaqueFunction(function=_robot),
        TimerAction(period=5.0, actions=[gui]),
    ])
