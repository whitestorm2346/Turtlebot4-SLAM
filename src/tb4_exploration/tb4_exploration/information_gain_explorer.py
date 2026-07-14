from tb4_exploration.base_explorer import BaseExplorer


class InformationGainExplorer(BaseExplorer):
    def get_name(self):
        return 'information_gain'

    def start(self):
        self.is_running = True
        self.node.get_logger().info('Starting Information Gain Explorer...')

    def stop(self):
        self.is_running = False
        self.node.get_logger().info('Stopping Information Gain Explorer...')