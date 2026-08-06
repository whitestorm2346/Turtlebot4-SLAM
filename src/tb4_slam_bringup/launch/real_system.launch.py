import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory(
        'tb4_slam_bringup'
    )

    turtlebot4_navigation_dir = get_package_share_directory(
        'turtlebot4_navigation'
    )

    frontier_exploration_dir = get_package_share_directory(
        'frontier_exploration_ros2'
    )

    # ==========================================
    # Launch arguments
    # ==========================================

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation clock'
    )

    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==========================================
    # Real TurtleBot4 hardware
    # ==========================================

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'model': model,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ==========================================
    # RTAB-Map + ROVER
    # ==========================================

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ==========================================
    # Nav2
    # ==========================================

    nav2_params = os.path.join(
        bringup_dir,
        'config',
        'nav2_custom.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot4_navigation_dir,
                'launch',
                'nav2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'params_file': nav2_params,
        }.items()
    )

    # ==========================================
    # Frontier exploration
    # ==========================================

    frontier_launch_file = os.path.join(
        frontier_exploration_dir,
        'launch',
        'frontier_explorer.launch.py'
    )

    frontier_params = os.path.join(
        bringup_dir,
        'config',
        'frontier_exploration_ros2.yaml'
    )

    frontier_explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            frontier_launch_file
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'control_service_enabled': 'false',
            'params_file': frontier_params,
            'log_level': 'info',
        }.items()
    )

    # ==========================================
    # SLAM manager
    # ==========================================

    slam_config = os.path.join(
        bringup_dir,
        'config',
        'slam_backend.yaml'
    )

    slam_manager = Node(
        package='tb4_slam_core',
        executable='slam_manager',
        name='slam_manager',
        namespace=namespace,
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        namespace_arg,
        model_arg,
        use_sim_time_arg,

        robot_launch,
        rtabmap_launch,
        nav2_launch,
        slam_manager,
        frontier_explorer,
    ])