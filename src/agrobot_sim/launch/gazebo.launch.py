from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_agrobot_sim = get_package_share_directory('agrobot_sim')
    pkg_agrobot_urdf = get_package_share_directory('agrobot_urdf')
    pkg_urdf_launch = get_package_share_directory('urdf_launch')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_spawn_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'])
    world_model_path = PathJoinSubstitution([pkg_agrobot_sim])
    robot_urdf_path = PathJoinSubstitution([pkg_agrobot_urdf, 'urdf', 'robot', 'robot.urdf.xacro.xml'])

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='mini_greenhouse',
            choices=['empty', 'mini_greenhouse'],
            description='World to load into Gazebo'
    )

    world_file_arg_substitution = SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf.xml')])

    gz_sim_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', world_model_path)

    gz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_agrobot_sim, 'worlds', LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        )

    run_state_publisher = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_urdf_launch, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'agrobot_urdf',
            'urdf_package_path': robot_urdf_path
        }.items()
    )

    gz_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_path),
        launch_arguments={
            'topic': '/robot_description'
        }.items()
    )

    return LaunchDescription([
        world_arg,
        world_file_arg_substitution,
        gz_sim_env,
        gz_launch,
        run_state_publisher,
        gz_spawn_robot
    ])
