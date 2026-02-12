
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TagFollowerNode(Node):

    def __init__(self):
        super().__init__('tag_follower_node')
        
        # Parameters
        self.declare_parameter('target_tag_id', 2)
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_k', 0.5)
        self.declare_parameter('linear_max_speed', 0.5)
        self.declare_parameter('linear_min_speed', 0.05)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('target_frame', 'camera_link')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Timer for control loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Tag Follower Node Initialized.')

    def control_loop(self):
        target_id = self.get_parameter('target_tag_id').get_parameter_value().integer_value
        tag_family = self.get_parameter('tag_family').get_parameter_value().string_value
        target_dist = self.get_parameter('target_distance').get_parameter_value().double_value
        k_p = self.get_parameter('linear_k').get_parameter_value().double_value
        max_speed = self.get_parameter('linear_max_speed').get_parameter_value().double_value
        min_speed = self.get_parameter('linear_min_speed').get_parameter_value().double_value
        threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        camera_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # Construct frame name from parameters
        tag_frame = f"{tag_family}:{target_id}"

        try:
            # Lookup transform from camera to tag
            t = self.tf_buffer.lookup_transform(
                camera_frame,
                tag_frame,
                rclpy.time.Time())
            
            # Since camera looks backward (+Z relative to camera link is outward?)
            # Wait, standard camera optical frame has Z as depth.
            # But in the Gazebo bridge setup, we use camera_link.
            # Let's check coordinates.
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            
            # Euclidean distance
            current_distance = math.sqrt(tx**2 + ty**2 + tz**2)
            
            # Error calculation
            error = current_distance - target_dist
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            if abs(error) > threshold:
                # Proportional control
                # The camera is now on the front, looking forward (+X).
                # If current_distance > target_dist, error is positive.
                # We want to move FORWARD to get closer.
                
                cmd_velocity = error * k_p
                
                # Limit speed
                if abs(cmd_velocity) > max_speed:
                    cmd_velocity = math.copysign(max_speed, cmd_velocity)
                elif abs(cmd_velocity) < min_speed:
                    cmd_velocity = math.copysign(min_speed, cmd_velocity)

                # Positive command moves robot forward towards the tag
                msg.twist.linear.x = cmd_velocity
                self.get_logger().info(f'Tag {target_id} at {current_distance:.2f}m. Moving X: {cmd_velocity:.2f}', throttle_duration_sec=1.0)
            else:
                msg.twist.linear.x = 0.0
                self.get_logger().info(f'Target distance reached: {current_distance:.2f}m', throttle_duration_sec=2.0)

            self.publisher_.publish(msg)

        except TransformException as ex:
            # Stop if tag lost
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            self.get_logger().info(f'Searching for tag {target_id}...', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = TagFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
