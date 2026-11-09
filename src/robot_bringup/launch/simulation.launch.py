import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    tb3_manip_desc_share = get_package_share_directory('turtlebot3_manipulation_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty.sdf')
    prefix = LaunchConfiguration('prefix', default='""')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    start_rviz = LaunchConfiguration('start_rviz', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file name or path',
    )
    declare_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Prefix for robot links and joints',
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Initial X position of the robot'
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Initial Y position of the robot'
    )
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.01', description='Initial Z position of the robot'
    )
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz', default_value='false', description='Launch RViz2 alongside Gazebo'
    )

    # Ensure Gazebo Harmonic can resolve package://turtlebot3_manipulation_description/meshes/...
    gz_resource_paths = [
        str(Path(pkg_share).parent.resolve()),
        str(Path(tb3_manip_desc_share).parent.resolve()),
        os.path.join(pkg_share, 'models'),
    ]
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(gz_resource_paths),
    )

    # Process TurtleBot3 Manipulation + Project Sensors URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot3_manipulation.urdf.xacro')
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' prefix:=',
            prefix,
            ' use_sim:=true',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Start Gazebo Harmonic simulation
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': [world, ' -r -v 4']}.items(),
    )

    # Spawn TurtleBot3 Manipulation robot into Gazebo from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_manipulation',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )

    # Start ros_gz_bridge for clock, cmd_vel, odom, tf, scan, imu, and rgbd_camera
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Spawn ros2_control controllers for OpenManipulator-X arm & parallel-jaw gripper
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_jsb_after_create = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    # Optional RViz2 node
    rviz_config_file = os.path.join(tb3_manip_desc_share, 'rviz', 'model.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_prefix,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_start_rviz,
        set_gz_resource_path,
        robot_state_publisher,
        start_gazebo,
        spawn_robot,
        start_bridge,
        spawn_jsb_after_create,
        spawn_controllers_after_jsb,
        rviz_node,
    ])
