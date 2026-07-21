import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot4_gz_bringup_dir = get_package_share_directory(
        'turtlebot4_gz_bringup'
    )

    tb4_slam_bringup_dir = get_package_share_directory(
        'tb4_slam_bringup'
    )

    turtlebot4_description_dir = get_package_share_directory(
        'turtlebot4_description'
    )

    irobot_create_description_dir = get_package_share_directory(
        'irobot_create_description'
    )

    irobot_create_gz_bringup_dir = get_package_share_directory(
        'irobot_create_gz_bringup'
    )

    ros_gz_sim_dir = get_package_share_directory(
        'ros_gz_sim'
    )

    arguments = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start RViz'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='Gazebo world'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            choices=['true', 'false'],
            description='Start Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='standard',
            choices=['standard', 'lite'],
            description='TurtleBot4 model'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0'
        ),
    ]

    # Gazebo needs these paths to find worlds, models and plugins.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(
                tb4_slam_bringup_dir,
                'worlds'
            ),
            os.path.join(
                tb4_slam_bringup_dir,
                'models'
            ),
            os.path.join(
                turtlebot4_gz_bringup_dir,
                'worlds'
            ),
            os.path.join(
                irobot_create_gz_bringup_dir,
                'worlds'
            ),
            str(Path(turtlebot4_description_dir).parent.resolve()),
            str(Path(irobot_create_description_dir).parent.resolve()),
        ])
    )

    gz_sim_launch = PathJoinSubstitution([
        ros_gz_sim_dir,
        'launch',
        'gz_sim.launch.py'
    ])

    # Start Gazebo server without the GUI.
    # --headless-rendering keeps RGB-D rendering available.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_sim_launch
        ]),
        launch_arguments={
            'gz_args': PythonExpression([
                "'",
                LaunchConfiguration('world'),
                ".sdf -r -v 4",
                "' if '",
                LaunchConfiguration('gui'),
                "' == 'true' else '",
                LaunchConfiguration('world'),
                ".sdf -r -s --headless-rendering -v 4",
                "'"
            ])
        }.items()
    )

    robot_spawn_launch = PathJoinSubstitution([
        turtlebot4_gz_bringup_dir,
        'launch',
        'turtlebot4_spawn.launch.py'
    ])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            robot_spawn_launch
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'rviz': LaunchConfiguration('rviz'),
            'model': LaunchConfiguration('model'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    return LaunchDescription(
        arguments + [
            gz_resource_path,
            gazebo_server,
            robot_spawn,
            clock_bridge,
        ]
    )