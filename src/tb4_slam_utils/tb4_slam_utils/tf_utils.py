import rclpy
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFHelper:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def get_robot_pose(self, target_frame='map', source_frame='base_link'):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().warn(
                f'Failed to lookup transform {target_frame} -> {source_frame}: {e}'
            )
            return None