#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Parameters
        # self.declare_parameter('video_path', '/media/mahmoud/DATA/calibration_11_11/video_output/calibration_11_11.mp4')
        self.declare_parameter('video_path', '/home/mohamed/aruco_pool.mp4')
        # self.declare_parameter('video_path', '/media/mahmoud/DATA/output(2).mp4')

        self.declare_parameter('camera_topic', 'bluerov2/camera/image')
        self.declare_parameter('fps', 40.0)

        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        # Publisher
        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Open video
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video: {self.video_path}")
            raise RuntimeError(f"Cannot open video: {self.video_path}")

        # Timer to publish frames
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.get_logger().info(f"Publishing video {self.video_path} to {self.camera_topic} at {self.fps} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # Loop video
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read video frame")
                return

        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
