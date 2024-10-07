import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time  # Import time module to track frame rate

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')

        # Initialize CvBridge
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0  # Number of messages to store for reliability
        )

        # Subscribe to the depth image topic (change topic name as needed)
        self.subscription = self.create_subscription(
            Image,
            '/disparity_map',  # Replace with your depth topic name
            self.depth_callback,
            qos_profile
        )

        # Load the watermark with alpha channel (RGBA)
        self.watermark = cv2.imread('logo.png', cv2.IMREAD_UNCHANGED)

        # Variables for FPS calculation
        self.prev_time = time.time()
        self.fps = []

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            disparity = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            max = np.max(disparity)
            min = np.min(disparity)
            median = np.median(disparity)
            print(f'Max: {max}, Min: {min} Med : {median}')
          
            base_line = 0.025
            focal = 869.1168823242188
            max_depth = 30
            min_depth = 0.5
            
            depth_metric = base_line * focal / disparity
      
      
            depth_scaled = np.clip((depth_metric - min_depth) / (max_depth-min_depth), 0.0, 1.0)

            # Convert to 8-bit image for colormap
            depth_8u = 255-np.uint8(depth_scaled * 255)
            #depth_8u = np.uint8(cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)).astype(np.uint8)
            
            
            
            # Apply the colormap
            depth_colormap = cv2.applyColorMap(depth_8u, cv2.COLORMAP_BONE)
            
            maskYellow = disparity > 7
            depth_colormap[maskYellow] = [0,255,255]
            
            maskRed = disparity > 17
            depth_colormap[maskRed] = [0, 0, 255]

            self.add_watermark(depth_colormap)

            depth_colormap = cv2.resize(depth_colormap, (int(depth_colormap.shape[1] * 0.5), int(depth_colormap.shape[0] * 0.5)))
            
            
            # Calculate FPS
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            if elapsed_time > 0:
                fps = 1.0 / elapsed_time
                self.fps.append(fps)
                if len(self.fps) > 100:
                    self.fps.pop(0)
            self.prev_time = current_time

            fps_mean = np.mean(self.fps)
            # Display FPS on the image
            cv2.putText(depth_colormap, f'FPS: {fps_mean:.2f}', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)


          
            # Overlay the watermark
         
            # Display the resulting image
            cv2.imshow('Depth Colormap', depth_colormap)
            
            cv2.waitKey(1)  # Needed to display the image properly

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def add_watermark(self, depth_colormap):
        if self.watermark is not None:
            """Adds the watermark to the entire depth_colormap image."""
            # Assuming the watermark is the same size as the image
            # Split the watermark into its color and alpha channels
            watermark_rgb = self.watermark[:, :, :3]  # Color part
            alpha_channel = self.watermark[:, :, 3] / 255.0  # Normalize alpha to [0, 1]

            # Expand the alpha channel to match the shape of the RGB image
            alpha_channel = np.dstack([alpha_channel] * 3)

            # Blend the watermark using NumPy's vectorized operations
            depth_colormap[:] = (depth_colormap * (1 - alpha_channel) +
                                watermark_rgb * alpha_channel).astype(np.uint8)
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

