import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('tb4_slam_bringup')

    turtlebot4_navigation_dir = get_package_share_directory(
        'turtlebot4_navigation'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'sim.launch.py'
            )
        ),
        launch_arguments={
            'world': 'warehouse',
            'model': 'standard',
            'rviz': 'false',
            'namespace': '',
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
        }.items()
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'rtabmap.launch.py'
            )
        )
    )

    custom_nav2_params = os.path.join(
        bringup_dir,
        'config',
        'nav2_custom.yaml'
    )

    official_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot4_navigation_dir,
                'launch',
                'nav2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': '',
            'params_file': custom_nav2_params,
        }.items()
    )

    slam_config = os.path.join(
        bringup_dir,
        'config',
        'slam_backend.yaml'
    )

    exploration_config = os.path.join(
        bringup_dir,
        'config',
        'exploration.yaml'
    )

    slam_manager = Node(
        package='tb4_slam_core',
        executable='slam_manager',
        name='slam_manager',
        output='screen',
        parameters=[slam_config],
    )

    exploration_manager = Node(
        package='tb4_slam_core',
        executable='exploration_manager',
        name='exploration_manager',
        output='screen',
        parameters=[exploration_config],
    )

    return LaunchDescription([
        sim_launch,
        rtabmap_launch,
        official_nav2, 
        slam_manager,
        exploration_manager,
    ])