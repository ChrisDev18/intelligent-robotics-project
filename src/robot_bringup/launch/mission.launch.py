"""Full autonomous mission in one command.

    ros2 launch robot_bringup mission.launch.py                  # no humans
    ros2 launch robot_bringup mission.launch.py humans:=true     # with walking humans

Starts explore.launch.py (robot, SLAM, Nav2, frontier explorer; no return home,
because doorway_navigator does that at the end), doorway_navigator (interleaved
room-discovery + passive target mapping), and mission_supervisor, which runs
alpha_cleaning and beta_pick_place in whichever room is entered first (with
autonomous visual target discovery enabled), resumes exploration until both rooms
and 100% of the house are mapped, then returns home and logs results to
log/mission_results.csv.

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
        output='screen', parameters=[{
            'wait_for_teammates': True,
            'interleaved_tasks': ParameterValue(
                LaunchConfiguration('interleaved_tasks'), value_type=bool
            ),
        }],
    )

    supervisor = Node(
        package='robot_bringup', executable='mission_supervisor', name='mission_supervisor',
        output='screen', parameters=[{
            'max_attempts': ParameterValue(
                LaunchConfiguration('max_attempts'), value_type=int
            ),
            'discover_targets': ParameterValue(
                LaunchConfiguration('discover_targets'), value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('humans', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('max_attempts', default_value='2',
                              description='Tries per room task, including the first'),
        DeclareLaunchArgument('discover_targets', default_value='true',
                              description='Visually discover cleaning zone, box, and drop spot'),
        DeclareLaunchArgument('interleaved_tasks', default_value='true',
                              description='Pause exploration and run each room task upon entry'),
        explore,
        doorway_navigator,
        supervisor,
    ])
