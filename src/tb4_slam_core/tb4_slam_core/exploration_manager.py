import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Undock

from tb4_exploration.exploration_factory import create_explorer
from tb4_slam_utils.tf_utils import TFHelper


class ExplorationManager(Node):
    def __init__(self):
        super().__init__('exploration_manager')

        self.declare_parameter(
            'exploration_method',
            'frontier'
        )

        exploration_method = (
            self.get_parameter('exploration_method')
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info(
            f'Exploration method: {exploration_method}'
        )

        try:
            self.explorer = create_explorer(
                exploration_method,
                self
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            raise
        
        self.explorer.start()

        self.latest_map = None
        self.is_navigating = False
        self.goal_sent_count = 0

        self.current_goal_handle = None
        self.goal_start_time = None
        self.navigation_timeout = 60.0  # seconds

        self.tf_helper = TFHelper(self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/exploration/status',
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        self.exploration_timer = self.create_timer(
            5.0,
            self.exploration_step
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10
        )

        self.undock_client = ActionClient(self, Undock, '/undock')

        self.initialization_done = False
        self.init_state = 'undock'
        self.init_start_time = self.get_clock().now()

        self.undock_goal_sent = False
        self.undock_done = False

        self.rotate_duration = 5.0
        self.rotate_speed = 0.35


    def map_callback(self, msg):
        self.latest_map = msg

    def publish_status(self):
        status = self.explorer.get_status()

        msg = String()
        msg.data = (
            f"method={status['name']}, "
            f"is_running={status['is_running']}, "
            f"has_map={self.latest_map is not None}, "
            f"is_navigating={self.is_navigating}, "
            f"goal_sent_count={self.goal_sent_count}"
        )

        self.status_pub.publish(msg)

    def exploration_step(self):
        if not self.initialization_done:
            self.run_initialization()
            return

        self.get_logger().info("=== Exploration Step ===")

        # Robot is already navigating
        if self.is_navigating:
            if self.goal_start_time is not None:
                elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9

                self.get_logger().info(
                    f"Already navigating. elapsed={elapsed:.1f}s"
                )

                if elapsed > self.navigation_timeout:
                    self.get_logger().warn("Navigation timeout. Canceling current goal.")

                    if self.current_goal_handle is not None:
                        self.current_goal_handle.cancel_goal_async()

                    self.is_navigating = False
                    self.current_goal_handle = None
                    self.goal_start_time = None

            return

        # Wait until map is available
        if self.latest_map is None:
            self.get_logger().warn("No map received yet.")
            return

        # Get robot pose from TF
        robot_tf = self.tf_helper.get_robot_pose(
            target_frame='map',
            source_frame='base_link'
        )

        if robot_tf is None:
            self.get_logger().warn("Robot pose is not available yet.")
            return

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not available yet.")
            return

        self.get_logger().info("Calling compute_next_goal()")

        # Compute next exploration goal
        goal = self.explorer.compute_next_goal(
            self.latest_map,
            robot_tf
        )

        if goal is None:
            self.get_logger().warn("compute_next_goal() returned None")
            return

        self.get_logger().info("Goal generated successfully.")

        self.get_logger().info(
            f"Sending frontier goal to Nav2: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        # Create Nav2 goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.is_navigating = True
        self.goal_start_time = self.get_clock().now()

        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

        self.goal_sent_count += 1

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected.')
            self.is_navigating = False
            self.current_goal_handle = None
            self.goal_start_time = None
            return

        self.current_goal_handle = goal_handle

        self.get_logger().info('Navigation goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result_msg = future.result()
        result = result_msg.result
        status = result_msg.status

        self.get_logger().info(
            f'Navigation finished. status={status}, '
            f'error_code={result.error_code}, '
            f'error_msg={result.error_msg}'
        )

        self.is_navigating = False
        self.current_goal_handle = None
        self.goal_start_time = None

    def feedback_callback(self, feedback_msg):
        pass

    def run_initialization(self):
        now = self.get_clock().now()

        if self.init_state == 'undock':
            if not self.undock_goal_sent:
                self.send_undock_goal()
                return False

            if not self.undock_done:
                self.get_logger().info('Initialization: waiting for undock...')
                return False

            self.init_state = 'rotate'
            self.init_start_time = now
            self.get_logger().info('Initialization: undock done, start rotating.')
            return False

        elif self.init_state == 'rotate':
            elapsed = (now - self.init_start_time).nanoseconds / 1e9

            self.get_logger().info(f'Initialization: rotating... {elapsed:.1f}s')

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.angular.z = self.rotate_speed
            self.cmd_vel_pub.publish(cmd)

            if elapsed >= self.rotate_duration:
                self.stop_robot()
                self.initialization_done = True
                self.get_logger().info('Initialization finished. Starting exploration.')

            return False

        return True


    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(cmd)
        

    def send_undock_goal(self):
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /undock action server...')
            return False

        goal_msg = Undock.Goal()

        self.get_logger().info('Sending undock goal...')
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_goal_response_callback)

        self.undock_goal_sent = True
        return True


    def undock_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected.')
            self.undock_done = True
            return

        self.get_logger().info('Undock goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)


    def undock_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Undock finished with status: {result.status}')
        self.undock_done = True
   


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Exploration manager...')

        if hasattr(node, 'explorer'):
            node.explorer.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()