import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    
    # Path to bridge config and model file
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    model_path = os.path.join(pkg_share, 'models', 'diff_bot', 'model.sdf')

    # Start Gazebo simulation (empty world or custom world)
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Spawn the differential drive robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', model_path, '-name', 'diff_bot', '-z', '0.2'],
        output='screen'
    )

    # Start the ros_gz_bridge parameter bridge
    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        spawn_robot,
        start_bridge
    ])