"""Phase 4: navigate the home world with Nav2 on the saved SLAM map.

    ros2 launch home_nav_sim nav.launch.py                 # no humans (first tests)
    ros2 launch home_nav_sim nav.launch.py humans:=true    # with walking humans

Starts the robot at the hall corner, Nav2 (map server, AMCL, planner,
controller, behaviours) on maps/home_map.yaml, and RViz. The map frame was
created where the robot spawns, so the launch sends AMCL the start pose
(0, 0, facing east) about 35 s in. If the robot isn't localised, click
"2D Pose Estimate" in RViz on the robot instead. Send goals with "Nav2 Goal" in RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('home_nav_sim')
    nav2 = get_package_share_directory('nav2_bringup')

    # Nav2's standard Jazzy settings, with only what this robot and map need changed.
    params = RewrittenYaml(
        source_file=os.path.join(nav2, 'params', 'nav2_params.yaml'),
        param_rewrites={
            'use_sim_time': 'True',
            'laser_max_range': '10.0',       # the robot's lidar range
            'robot_radius': '0.22',          # TurtleBot3 Waffle Pi with the arm folded
            'inflation_radius': '0.45',      # keeps the 1.2 m doors passable
        },
        convert_types=True,
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot_in_home.launch.py')),
        launch_arguments={'humans': LaunchConfiguration('humans'),
                          'gui': LaunchConfiguration('gui')}.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )

    # Tell AMCL the robot starts at the map origin facing east (where SLAM began),
    # same as clicking "2D Pose Estimate" there in RViz.
    initial_pose = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, '
             'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
             '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.07]}}'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'home_map.yaml')),
        DeclareLaunchArgument('humans', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        robot,
        # Wait for the robot, /clock, /scan and /odom before starting Nav2.
        TimerAction(period=15.0, actions=[nav, rviz]),
        TimerAction(period=35.0, actions=[initial_pose]),
    ])
