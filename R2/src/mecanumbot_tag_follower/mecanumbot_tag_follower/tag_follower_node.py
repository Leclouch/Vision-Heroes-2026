import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped

class TagFollowerNode(Node):
    """
    A simplified Tag Follower that receives information from the /tag_info topic.
    It moves forward if the tag is further than 1 meter and stops once reached.
    """

    def __init__(self):
        super().__init__('tag_follower_node')
        
        # --- Parameters ---
        self.declare_parameter('target_tag_id', 2)
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_speed', 0.2) # Constant speed for simplicity

        # Subscription: Receives [ID, Distance, Yaw] from the distance node
        self.subscription = self.create_subscription(
            PointStamped,
            '/tag_info',
            self.tag_info_callback,
            10)

        # Publisher: sends velocity commands
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.get_logger().info('Tag Follower Node (Subscriber Mode) Initialized.')

    def tag_info_callback(self, msg):
        """
        Callback triggered whenever new tag info is published.
        msg.point.x = ID
        msg.point.y = Distance
        msg.point.z = Yaw
        """
        target_id = self.get_parameter('target_tag_id').get_parameter_value().integer_value
        target_dist = self.get_parameter('target_distance').get_parameter_value().double_value
        speed = self.get_parameter('linear_speed').get_parameter_value().double_value

        current_id = int(msg.point.x)
        current_distance = msg.point.y
        current_yaw = msg.point.z

        # Only react to our target tag
        if current_id != target_id:
            return

        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'

        # Control Logic: Move forward if further than target_distance
        if current_distance > target_dist:
            cmd_msg.twist.linear.x = speed
            self.get_logger().info(
                f'Tag {current_id} at {current_distance:.2f}m. Moving forward...', 
                throttle_duration_sec=1.0
            )
        else:
            # Reached target or closer: Stop
            cmd_msg.twist.linear.x = 0.0
            self.get_logger().info(
                f'Target distance reached ({current_distance:.2f}m). Stopping.', 
                throttle_duration_sec=2.0
            )

        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops on shutdown
        stop_msg = TwistStamped()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
