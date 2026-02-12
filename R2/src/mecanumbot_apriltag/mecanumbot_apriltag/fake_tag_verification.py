
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class FakeTagNode(Node):
    def __init__(self):
        super().__init__('fake_tag_node')
        self.publisher_ = self.create_publisher(AprilTagDetectionArray, '/detections', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Fake Tag Node started. Publishing tag ID 2 at exactly 3.0 meters.')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # 1. Publish TF: camera_link -> tag36h11:2 at exactly 3m on X axis
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'tag36h11:2'
        t.transform.translation.x = 3.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Detection Array
        msg = AprilTagDetectionArray()
        msg.header.stamp = now
        msg.header.frame_id = 'camera_link'
        
        detection = AprilTagDetection()
        detection.family = 'tag36h11'
        detection.id = 2
        # (Note: Pose info is usually empty in the message, which is why we use TF)
        
        msg.detections.append(detection)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = FakeTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
