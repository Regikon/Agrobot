from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetLaunchConfiguration, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_agrobot_sim = get_package_share_directory('agrobot_sim')
    pkg_agrobot_urdf = get_package_share_directory('agrobot_urdf')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    world_model_path = PathJoinSubstitution([pkg_agrobot_sim])
    robot_urdf_path = PathJoinSubstitution([pkg_agrobot_urdf, 'urdf', 'robot', 'robot.urdf.xacro.xml'])

    use_sim_time_arg = LaunchConfiguration('use_sim_time', default=True)

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='mini_greenhouse',
            choices=['empty', 'mini_greenhouse'],
            description='World to load into Gazebo'
    )
    world_file_arg_substitution = SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf.xml')])

    nav_config = PathJoinSubstitution(
        [
            pkg_agrobot_sim,
            'config',
            'nav_config.yaml'
        ]
    )

    slam_launch_path = PathJoinSubstitution(
        [
            pkg_slam_toolbox,
            'launch',
            'online_sync_launch.py'
        ]
    )

    bridge_config = PathJoinSubstitution(
        [
            pkg_agrobot_sim,
            'config',
            'bridge_config.yaml'
        ]
    )

    # Getting urdf via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_urdf_path
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            pkg_agrobot_urdf,
            'config',
            'ros2_control.yaml'
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'agrobot', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    agrobot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--param-file',
            robot_controllers,
            'agrobot_controller',
        ]
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': bridge_config
            }
        ],
        output='screen'
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': nav_config,
        }.items()
    )
    
    gz_sim_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', world_model_path)

    return LaunchDescription([
        # Set Environment
        world_arg,
        world_file_arg_substitution,
        gz_sim_env,
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [TextSubstitution(text='-r '), PathJoinSubstitution([pkg_agrobot_sim, 'worlds', LaunchConfiguration('world_file')])]
            }.items()
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[agrobot_controller_spawner],
            )
        ),
        nav,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time_arg,
            description='If true, use simulated clock',
        )
    ])
