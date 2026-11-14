"""Phase A: the robot explores the unknown home world by itself.

    ros2 launch home_nav_sim explore.launch.py                 # no humans (start here)
    ros2 launch home_nav_sim explore.launch.py humans:=true    # with walking humans

No map is loaded. slam_toolbox builds the map live, Nav2 drives (planning and
obstacle avoidance on that growing map) and frontier_explorer keeps sending
the robot to the edge of the unknown until the whole house is mapped. It then
publishes True on /exploration_done and drives back to the start.

Save the finished map with:
    python3 src/home_nav_sim/tools/save_map.py src/home_nav_sim/maps/explored_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('home_nav_sim')
    nav2 = get_package_share_directory('nav2_bringup')

    # Same Nav2 changes as nav.launch.py.
    nav_params = RewrittenYaml(
        source_file=os.path.join(nav2, 'params', 'nav2_params.yaml'),
        param_rewrites={
            'use_sim_time': 'True',
            'robot_radius': '0.22',
            'inflation_radius': '0.45',
            # Gazebo runs well below real time in WSL; allow older transforms.
            'transform_tolerance': '1.0',
        },
        convert_types=True,
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot_in_home.launch.py')),
        launch_arguments={'humans': LaunchConfiguration('humans'),
                          'gui': LaunchConfiguration('gui')}.items(),
    )

    # Live mapping (slam_toolbox's own launcher activates its lifecycle node).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
                          'use_sim_time': 'true'}.items(),
    )

    # Nav2 without map_server/AMCL: SLAM provides both the map and the map->odom transform.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': nav_params,
                          'autostart': 'True'}.items(),
    )

    explorer = Node(
        package='home_nav_sim',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'return_home': ParameterValue(LaunchConfiguration('return_home'),
                                                   value_type=bool)}],
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
        DeclareLaunchArgument('humans', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('return_home', default_value='true'),
        robot,
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(period=15.0, actions=[nav, rviz]),
        # The explorer waits for /map, TF and Nav2 by itself; start it last.
        TimerAction(period=25.0, actions=[explorer]),
    ])
