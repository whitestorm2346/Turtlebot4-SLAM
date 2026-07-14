from tb4_exploration.base_explorer import BaseExplorer


class RandomExplorer(BaseExplorer):
    def get_name(self):
        return 'random'

    def start(self):
        self.is_running = True
        self.node.get_logger().info('Starting Random Explorer...')

    def stop(self):
        self.is_running = False
        self.node.get_logger().info('Stopping Random Explorer...')