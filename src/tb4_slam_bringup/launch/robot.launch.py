from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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

    # 真機硬體啟動內容會在到研究室後補上。
    #
    # 預計包含：
    # 1. TurtleBot4 / Create 3 base
    # 2. OAK-D camera
    # 3. LiDAR
    # 4. robot_state_publisher
    # 5. 必要的 topic bridge
    #
    # 目前先保留為安全的 launch 骨架，
    # 避免在沒有真機時引用錯誤的官方 launch package。

    startup_info = LogInfo(
        msg=[
            '\n',
            '======================================\n',
            'TurtleBot4 Real Robot Launch\n',
            '======================================\n',
            'Namespace    : ', namespace, '\n',
            'Model        : ', model, '\n',
            'Use sim time : ', use_sim_time, '\n',
            '\n',
            'Hardware drivers are not included yet.\n',
            'Complete robot.launch.py after verifying the real robot setup.\n',
        ]
    )

    return LaunchDescription([
        namespace_arg,
        model_arg,
        use_sim_time_arg,
        startup_info,
    ])