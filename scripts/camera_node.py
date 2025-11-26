#!/usr/bin/env python3
"""
ROS2 Camera Node for Modulr Agent
Publishes camera images to /camera/image_raw topic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CameraNode(Node):
    """ROS2 node that publishes camera images to /camera/image_raw"""
    
    def __init__(self, camera_index=0, frame_rate=30):
        super().__init__('camera_publisher')
        
        # Create publisher for camera images with Reliable QoS
        # This ensures compatibility with rosbridge_server which expects Reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Open camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_index}')
            sys.exit(1)
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)
        
        # Get actual frame rate
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Camera opened successfully. FPS: {actual_fps}')
        
        # Create timer to publish frames
        timer_period = 1.0 / frame_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera node started. Publishing to /camera/image_raw')
    
    def timer_callback(self):
        """Callback function that publishes camera frames"""
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert OpenCV image to ROS2 Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'camera_frame'
                
                # Publish the image
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
        else:
            self.get_logger().warn('Failed to read frame from camera')
    
    def destroy_node(self):
        """Clean up resources"""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    """Main function to run the camera node"""
    rclpy.init(args=args)
    
    # Parse command line arguments for camera index
    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"Invalid camera index: {sys.argv[1]}. Using default: 0")
    
    # Create and run the node
    node = CameraNode(camera_index=camera_index)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

