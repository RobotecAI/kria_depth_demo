# usage: 
#  python3 publish_with_know_disparity.py --ros-args -p shift_pixels:=-1
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DualImagePublisher(Node):
    def __init__(self):
        super().__init__('dual_image_publisher')

        # Declare ROS 2 parameters
        self.declare_parameter('image_file', 'left.png')
        self.declare_parameter('shift_pixels', -1)
 
        # Get parameters
        image_file = self.get_parameter('image_file').get_parameter_value().string_value
        shift_pixels = self.get_parameter('shift_pixels').get_parameter_value().integer_value

        # Configure QoS
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # Create publishers for /left and /right topics
        self.left_publisher = self.create_publisher(Image, '/left', qos_profile)
        self.right_publisher = self.create_publisher(Image, '/right', qos_profile)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load image
        self.left_image = cv2.imread(image_file)
        self.left_image = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        if self.left_image is None:
            self.get_logger().error(f"Failed to load image from {image_file}")
            return

        # Create shifted image
        self.right_image = self.shift_image(self.left_image, shift_pixels)

        # Publish images at a regular interval
        self.timer = self.create_timer(1.0, self.publish_images)

    def shift_image(self, image, pixels):
        # Shift the image left or right by the given number of pixels
        if pixels > 0:
            # Shift right
            shifted_image = np.roll(image, pixels, axis=1)
        else:
            # Shift left
            shifted_image = np.roll(image, pixels, axis=1)

        return shifted_image

    def publish_images(self):
        # Convert OpenCV images to ROS 2 Image messages
        left_image_msg = self.bridge.cv2_to_imgmsg(self.left_image, encoding='mono8')
        right_image_msg = self.bridge.cv2_to_imgmsg(self.right_image, encoding='mono8')

        # Publish the images
        self.left_publisher.publish(left_image_msg)
        self.right_publisher.publish(right_image_msg)

        self.get_logger().info('Published images on /left and /right')

def main(args=None):
    rclpy.init(args=args)
    node = DualImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
