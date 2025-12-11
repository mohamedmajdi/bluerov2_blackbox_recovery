#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import sys

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Define the package name where the config is stored
        # Update this if your package name is different
        package_name = 'bluerov2_util'

        try:
            # Get the package share directory
            pkg_share = get_package_share_directory(package_name)
            default_file_path = os.path.join(pkg_share, 'config', 'camera_calb_20_11.npz')
        except Exception as e:
            # Fallback for testing without installing the package
            self.get_logger().warn(f"Could not find package '{package_name}': {e}. Falling back to local 'config' folder.")
            default_file_path = os.path.join(os.getcwd(), 'config', 'camera_calibration_19_11.npz')

        # Declare parameters so you can change them from command line without editing code
        self.declare_parameter('file_path', default_file_path)
        self.declare_parameter('topic_name', 'camera_parameter')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('width', 640)  # Default width (change via CLI)
        self.declare_parameter('height', 480) # Default height (change via CLI)
        self.declare_parameter('frequency', 1.0) # Hz

        # Get parameters
        file_path = self.get_parameter('file_path').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_width = self.get_parameter('width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('height').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value

        # Initialize the publisher
        self.publisher_ = self.create_publisher(CameraInfo, topic_name, 10)
        
        # Load the data
        self.camera_info_msg = self.load_camera_info(file_path)

        if self.camera_info_msg:
            # Create a timer to publish periodically
            self.timer = self.create_timer(1.0 / frequency, self.timer_callback)
            self.get_logger().info(f'Node started. Publishing to "{topic_name}" from {file_path}')
        else:
            self.get_logger().error('Failed to start publisher due to loading errors.')
            sys.exit(1)

    def load_camera_info(self, file_path):
        """Unpacks the NPZ file and populates the CameraInfo message."""
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return None

        try:
            # Load the npz file
            data = np.load(file_path)
            self.get_logger().info(f"Keys found in file: {list(data.files)}")

            # Extract matrices
            # Based on your file description: 
            # 'camera_matrix' -> Intrinsic matrix K (3x3)
            # 'dist_coeffs'   -> Distortion coefficients D (1x5)
            
            if 'camera_matrix' not in data or 'dist_coeffs' not in data:
                self.get_logger().error("NPZ file missing 'camera_matrix' or 'dist_coeffs' keys.")
                return None

            K_matrix = data['camera_matrix']
            D_matrix = data['dist_coeffs']

            # Create the message
            msg = CameraInfo()

            # 1. Image Dimensions
            msg.width = self.image_width
            msg.height = self.image_height

            # 2. Distortion Model
            msg.distortion_model = "plumb_bob" # Standard for 5-param k1,k2,t1,t2,k3

            # 3. Distortion Coefficients (D)
            # Ensure it's a flat list of floats
            msg.d = D_matrix.flatten().tolist()

            # 4. Intrinsic Matrix (K) - 3x3
            msg.k = K_matrix.flatten().tolist()

            # 5. Rectification Matrix (R) - 3x3
            # For monocular cameras, this is usually the Identity matrix
            msg.r = np.eye(3, dtype=np.float64).flatten().tolist()

            # 6. Projection Matrix (P) - 3x4
            # For monocular, P is K with an extra column of zeros [K|0]
            # P = [fx 0 cx 0]
            #     [0 fy cy 0]
            #     [0  0  1 0]
            P_matrix = np.zeros((3, 4), dtype=np.float64)
            P_matrix[0:3, 0:3] = K_matrix # Copy K into the top-left 3x3
            msg.p = P_matrix.flatten().tolist()

            return msg

        except Exception as e:
            self.get_logger().error(f"Error reading NPZ file: {str(e)}")
            return None

    def timer_callback(self):
        # Update timestamp for every published message
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_msg.header.frame_id = self.frame_id
        
        self.publisher_.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()