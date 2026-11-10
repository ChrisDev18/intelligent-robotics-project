"""Open the home world in Gazebo Harmonic (ROS 2 Jazzy, Ubuntu 24.04).

    ros2 launch home_nav_sim world.launch.py                 # with walking humans
    ros2 launch home_nav_sim world.launch.py humans:=false   # empty of people (for SLAM)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('home_nav_sim')
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    humans = LaunchConfiguration('humans')

    world_with = PathJoinSubstitution([pkg, 'worlds', 'home.sdf'])
    world_without = PathJoinSubstitution([pkg, 'worlds', 'home_no_humans.sdf'])

    gz_with = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r ', world_with], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(humans),
    )
    gz_without = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r ', world_without], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(humans),
    )

    # Simulation clock into ROS 2 (needed later for SLAM / Nav2 with use_sim_time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('humans', default_value='true',
                              description='Spawn the three walking humans'),
        gz_with,
        gz_without,
        clock_bridge,
    ])
