from abc import ABC, abstractmethod


class BaseExplorer(ABC):
    def __init__(self, node):
        self.node = node
        self.is_running = False

    @abstractmethod
    def get_name(self):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    def compute_next_goal(self, map_msg, robot_tf):
        self.node.get_logger().warn(
            f'{self.get_name()} compute_next_goal() is not implemented yet.'
        )
        return None

    def get_status(self):
        return {
            'name': self.get_name(),
            'is_running': self.is_running,
        }