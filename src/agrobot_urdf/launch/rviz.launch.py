from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_agrobot_urdf = get_package_share_directory('agrobot_urdf')
    config_path = PathJoinSubstitution([pkg_agrobot_urdf, 'config', 'agrobot_config.rviz'])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', config_path]
    )

    return LaunchDescription([rviz2])
