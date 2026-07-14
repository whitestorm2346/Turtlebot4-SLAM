from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

from tb4_slam_backends.base_slam_backend import BaseSlamBackend


class RTABMapBackend(BaseSlamBackend):
    def __init__(self, node):
        super().__init__(node)

        self.latest_map = None
        self.latest_cloud = None

    def get_name(self):
        return 'rtabmap'

    def start(self):
        self.is_running = True
        self.node.get_logger().info('Starting RTAB-Map backend wrapper...')

        self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.node.create_subscription(
            PointCloud2,
            '/cloud_map',
            self.cloud_callback,
            10
        )

    def stop(self):
        self.is_running = False
        self.node.get_logger().info('Stopping RTAB-Map backend wrapper...')

    def map_callback(self, msg):
        self.latest_map = msg

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def get_map(self):
        return self.latest_map

    def get_point_cloud(self):
        return self.latest_cloud

    def get_status(self):
        return {
            'name': self.get_name(),
            'is_running': self.is_running,
            'has_map': self.latest_map is not None,
            'has_cloud': self.latest_cloud is not None,
        }