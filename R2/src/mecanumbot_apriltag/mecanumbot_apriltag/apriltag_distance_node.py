
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
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

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if not msg.detections:
            return

        for detection in msg.detections:
            tag_family = detection.family
            tag_id = detection.id
            # Standard tf frame naming for apriltag_ros: "family:id"
            tag_frame = f"{tag_family}:{tag_id}"
            
            # The detections array usually has a header with frame_id
            # This is the frame where detection happened (camera frame)
            # We want transform from this frame to the tag frame to get distance
            camera_frame = msg.header.frame_id if msg.header.frame_id else 'camera_link'
            
            try:
                # Look up transform from camera_frame to tag_frame
                # This gives us the position of the tag relative to the camera
                t = self.tf_buffer.lookup_transform(
                    camera_frame,
                    tag_frame,
                    # We can use rclpy.time.Time() for latest, or msg.header.stamp for exact sync
                    rclpy.time.Time())
                
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z
                
                distance = math.sqrt(tx**2 + ty**2 + tz**2)
                
                self.get_logger().info(f'Tag ID: {tag_id}, Distance: {distance:.2f} meters')

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {camera_frame} to {tag_frame}: {ex}')
                return

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
