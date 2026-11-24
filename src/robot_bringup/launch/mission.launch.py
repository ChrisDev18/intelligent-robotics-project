"""Full autonomous mission in one command.

    ros2 launch robot_bringup mission.launch.py                  # no humans
    ros2 launch robot_bringup mission.launch.py humans:=true     # with walking humans

Starts explore.launch.py (robot, SLAM, Nav2, frontier explorer; no return home,
because doorway_navigator does that at the end), doorway_navigator waiting for
each room task, and mission_supervisor, which runs alpha_cleaning and
beta_pick_place when their rooms are ready, retries each once, and logs
results to log/mission_results.csv.

Watch progress with:  ros2 topic echo /mission_status
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_bringup')

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'explore.launch.py')),
        launch_arguments={
            'humans': LaunchConfiguration('humans'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'return_home': 'false',
        }.items(),
    )

    doorway_navigator = Node(
        package='robot_bringup', executable='doorway_navigator', name='doorway_navigator',
        output='screen', parameters=[{'wait_for_teammates': True}],
    )

    supervisor = Node(
        package='robot_bringup', executable='mission_supervisor', name='mission_supervisor',
        output='screen', parameters=[{'max_attempts': ParameterValue(LaunchConfiguration('max_attempts'),
                                                    value_type=int)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('humans', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('max_attempts', default_value='2',
                              description='Tries per room task, including the first'),
        explore,
        doorway_navigator,
        supervisor,
    ])
