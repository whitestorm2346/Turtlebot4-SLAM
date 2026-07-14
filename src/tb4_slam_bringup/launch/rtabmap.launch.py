import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rtabmap_launch_dir,
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'frame_id': 'base_link',
            'rgb_topic': '/oakd/rgb/preview/image_raw',
            'depth_topic': '/oakd/rgb/preview/depth',
            'camera_info_topic': '/oakd/rgb/preview/camera_info',
            'odom_topic': '/odom',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'rtabmap_viz': 'true',
            'rviz': 'false',
            'approx_sync': 'true',
            'qos': '2',
            # 'log_level': 'debug',

            # RTAB-Map grid / octomap settings
            'args': (
                '--delete_db_on_start '
                '--Grid/3D true '
                '--Grid/Sensor 1 '
                '--Grid/RayTracing true '

                # 保留 loop closure candidate，不要過度嚴格
                # '--Rtabmap/LoopThr 0.15 '
                '--Rtabmap/LoopThr 0.11 '

                # Fundamental matrix geometric verification
                '--VhEp/Enabled false '
                '--VhEp/MatchCountMin 12 '
                '--VhEp/RansacParam1 2.0 '
                '--VhEp/RansacParam2 0.99 '

                # Visual transformation verification
                # '--Vis/MinInliers 20 '
                '--Vis/MinInliers 10 '
                '--Vis/MinInliersDistribution 0.05 '
                '--Vis/MinInliersDistribution 0.0 '

                # Reject graph constraints producing excessive optimization error
                # '--RGBD/OptimizeMaxError 2.0 '
                '--RGBD/OptimizeMaxError 0 '

                # 第一階段先關閉 local proximity closure，方便隔離問題
                '--RGBD/ProximityBySpace false '
                '--RGBD/ProximityByTime false '

                # debug
                # '--Rtabmap/PublishStats true '
            ),
        }.items()
    )

    return LaunchDescription([
        rtabmap
    ])