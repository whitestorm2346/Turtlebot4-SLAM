from tb4_slam_backends.base_slam_backend import BaseSlamBackend


class ORBSlamBackend(BaseSlamBackend):
    def get_name(self):
        return 'orb_slam'

    def start(self):
        self.is_running = True
        self.node.get_logger().info('Starting ORB-SLAM backend...')

    def stop(self):
        self.is_running = False
        self.node.get_logger().info('Stopping ORB-SLAM backend...')