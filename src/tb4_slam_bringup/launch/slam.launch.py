import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('tb4_slam_bringup')

    slam_config = os.path.join(
        bringup_dir,
        'config',
        'slam_backend.yaml'
    )

    slam_manager_node = Node(
        package='tb4_slam_core',
        executable='slam_manager',
        name='slam_manager',
        output='screen',
        parameters=[slam_config],
    )

    return LaunchDescription([
        slam_manager_node
    ])