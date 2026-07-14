from abc import ABC, abstractmethod


class BaseSlamBackend(ABC):
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

    def reset(self):
        self.node.get_logger().warn(
            f'{self.get_name()} reset() is not implemented yet.'
        )
        return False

    def get_map(self):
        self.node.get_logger().warn(
            f'{self.get_name()} get_map() is not implemented yet.'
        )
        return None

    def get_point_cloud(self):
        self.node.get_logger().warn(
            f'{self.get_name()} get_point_cloud() is not implemented yet.'
        )
        return None

    def save_map(self, path):
        self.node.get_logger().warn(
            f'{self.get_name()} save_map() is not implemented yet. path={path}'
        )
        return False

    def get_status(self):
        return {
            'name': self.get_name(),
            'is_running': self.is_running,
        }