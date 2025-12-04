#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class BlueROV2DummyCameraNode(Node):
    """Dummy camera node that captures webcam frames and publishes them as ROS2 Images"""

    def __init__(self):
        super().__init__('bluerov2_dummy_camera_node')

        # Publisher for camera topic
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.cv_bridge = CvBridge()

        # Open webcam (index 0 = default camera)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam!")
            raise RuntimeError("Cannot access webcam")

        # Publish frames at 10 Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_frame)

        self.get_logger().info("Dummy camera node started — publishing on /camera")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from webcam")
            return

        # Publish as ROS Image
        img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)

        # Show the live feed (OpenCV window)
        cv2.imshow("BlueROV2 Dummy Camera", frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC key to quit
            self.get_logger().info("ESC pressed — shutting down node...")
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = BlueROV2DummyCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Dummy Camera Node...")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
