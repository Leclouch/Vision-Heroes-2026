import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped # Using PointStamped to send ID, Dist, Yaw
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ApriltagDistanceNode(Node):

    def __init__(self):
        super().__init__('apriltag_distance_node')
        
        self.target_frame = self.declare_parameter(
            'target_frame', 'camera_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for tag information (ID, Distance, Yaw)
        self.publisher = self.create_publisher(PointStamped, '/tag_info', 10)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        
        self.get_logger().info('AprilTag Distance & Yaw Node Initialized.')

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counter-clockwise)
        pitch is rotation around y in radians (counter-clockwise)
        yaw is rotation around z in radians (counter-clockwise)
        """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return yaw_z # We only need yaw for this task

    def listener_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_family = detection.family
            tag_id = detection.id
            tag_frame = f"{tag_family}:{tag_id}"
            camera_frame = msg.header.frame_id if msg.header.frame_id else 'camera_link'
            
            try:
                # Look up transform from camera to tag
                t = self.tf_buffer.lookup_transform(
                    camera_frame,
                    tag_frame,
                    rclpy.time.Time())
                
                # --- Position/Distance ---
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z
                distance = math.sqrt(tx**2 + ty**2 + tz**2)
                
                # --- Orientation/Yaw ---
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                yaw = self.euler_from_quaternion(qx, qy, qz, qw)
                
                # --- Publishing Info ---
                info_msg = PointStamped()
                info_msg.header.stamp = self.get_clock().now().to_msg()
                info_msg.header.frame_id = camera_frame
                
                # We package the info into the Point message:
                # x = Tag ID, y = Distance, z = Yaw
                info_msg.point.x = float(tag_id)
                info_msg.point.y = distance
                info_msg.point.z = yaw
                
                self.publisher.publish(info_msg)

                self.get_logger().info(
                    f'ID: {tag_id} | Dist: {distance:.2f}m | Yaw: {math.degrees(yaw):.1f}°'
                )

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {camera_frame} to {tag_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
