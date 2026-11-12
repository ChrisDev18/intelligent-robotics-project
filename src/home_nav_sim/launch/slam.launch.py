"""Phase 3: build a map of the home world with slam_toolbox.

    ros2 launch home_nav_sim slam.launch.py

Starts the robot in the home world WITHOUT the walking humans (moving people
would be baked into the map as obstacles), slam_toolbox and RViz.

Drive with teleop in another terminal, then save the map:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/intelligent-robotics-project/src/home_nav_sim/maps/home_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('home_nav_sim')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'slam.rviz')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot_in_home.launch.py')),
        launch_arguments={'humans': 'false', 'gui': LaunchConfiguration('gui')}.items(),
    )

    # slam_toolbox's own launcher configures and activates the lifecycle node.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params, 'use_sim_time': 'true'}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        robot,
        # Wait for the robot, /clock and /scan before starting SLAM and RViz.
        TimerAction(period=10.0, actions=[slam, rviz]),
    ])
