#!/usr/bin/env python3
"""
Simple camera node for ROS2
This node can be used for custom camera processing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('camera_name', 'robot_camera')
        
        # Get parameters
        device = self.get_parameter('device').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fps = self.get_parameter('framerate').value
        camera_name = self.get_parameter('camera_name').value
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Image, 
            f'{camera_name}/image_raw', 
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Open camera
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device: {device}')
            return
        
        self.get_logger().info(f'Camera opened successfully: {device}')
        self.get_logger().info(f'Resolution: {width}x{height} @ {fps}fps')
        
        # Timer for capturing frames
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            
            # Publish
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().warning('Failed to capture frame')
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
