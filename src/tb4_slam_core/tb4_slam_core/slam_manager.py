import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tb4_slam_backends.slam_backend_factory import create_slam_backend
from tb4_slam_utils.tf_utils import TFHelper


class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')

        self.declare_parameter('slam_backend', 'rtabmap')
        backend_name = self.get_parameter('slam_backend').value

        self.get_logger().info(f'Selected SLAM backend: {backend_name}')

        self.backend = create_slam_backend(backend_name, self)
        self.backend.start()

        self.status_pub = self.create_publisher(
            String,
            '/slam/status',
            10
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        self.tf_helper = TFHelper(self)

    
    def publish_status(self):
        status = self.backend.get_status()

        robot_pose = self.tf_helper.get_robot_pose()
        has_robot_pose = robot_pose is not None
        status['has_robot_pose'] = has_robot_pose

        msg = String()
        msg.data = str(status)

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SlamManager()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Shutting down SLAM manager...')

    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Failed to start SLAM manager: {e}')
        else:
            print(f'Failed to create SLAM manager: {e}')

    finally:
        if node is not None:
            if hasattr(node, 'backend'):
                node.backend.stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()