#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange

from bluerov2_interface.msg import Aruco


class ArucoDetectionNode(Node):
    """ArUco Marker Detection Node with dynamic ID selection"""

    def __init__(self):
        super().__init__('aruco_detection_node')
        
        # Initialize cv_bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Declare ROS parameters
        self._declare_parameters()
        
        # Get initial parameter values
        self._update_parameters()
        
        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Initialize ArUco detector
        self._initialize_aruco_detector()
        
        # Create subscriber for camera input
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10  # QoS queue size
        )
        
        # Create publisher for ArUco detections
        self.aruco_publisher = self.create_publisher(
            Aruco,
            'aruco_detection',
            10
        )
        
        self.get_logger().info("ArUco Detection Node initialized")
        self.get_logger().info(f"Target ArUco ID: {self.target_id}")
        self.get_logger().info(f"ArUco Dictionary: {self.aruco_dict_type}")

    def _declare_parameters(self):
        """Declare ROS parameters"""
        # Target ArUco marker ID
        target_id_descriptor = ParameterDescriptor(
            description='Target ArUco marker ID to detect (0-249)',
            integer_range=[IntegerRange(from_value=0, to_value=249, step=1)]
        )
        self.declare_parameter('target_id', 0, target_id_descriptor)
        
        # ArUco dictionary type (0 = DICT_4X4_50, 1 = DICT_5X5_100)
        dict_type_descriptor = ParameterDescriptor(
            description='ArUco dictionary type: 0=DICT_4X4_50, 1=DICT_5X5_100, 2=DICT_6X6_250',
            integer_range=[IntegerRange(from_value=0, to_value=2, step=1)]
        )
        self.declare_parameter('aruco_dict_type', 0, dict_type_descriptor)
        
        self.get_logger().info("ROS parameters declared")

    def _update_parameters(self):
        """Get current parameter values"""
        self.target_id = self.get_parameter('target_id').value
        dict_type_param = self.get_parameter('aruco_dict_type').value
        
        # Map parameter to OpenCV ArUco dictionary
        dict_map = {
            0: cv2.aruco.DICT_4X4_50,
            1: cv2.aruco.DICT_5X5_100,
            2: cv2.aruco.DICT_6X6_250
        }
        self.aruco_dict_type = dict_map.get(dict_type_param, cv2.aruco.DICT_4X4_50)
        
        self.get_logger().info(
            f"Parameters updated - target_id: {self.target_id}, "
            f"aruco_dict_type: {dict_type_param}"
        )

    def parameters_callback(self, params):
        """Callback for parameter changes"""
        reinit_detector = False
        
        for param in params:
            if param.name == 'target_id':
                self.target_id = param.value
                self.get_logger().info(f"Updated target_id to {self.target_id}")
            elif param.name == 'aruco_dict_type':
                dict_map = {
                    0: cv2.aruco.DICT_4X4_50,
                    1: cv2.aruco.DICT_5X5_100,
                    2: cv2.aruco.DICT_6X6_250
                }
                self.aruco_dict_type = dict_map.get(param.value, cv2.aruco.DICT_4X4_50)
                self.get_logger().info(f"Updated aruco_dict_type to {param.value}")
                reinit_detector = True
        
        # Reinitialize detector if dictionary changed
        if reinit_detector:
            self._initialize_aruco_detector()
            self.get_logger().info("ArUco detector reinitialized with new dictionary")
        
        return SetParametersResult(successful=True)

    def _initialize_aruco_detector(self):
        """Initialize ArUco detector with selected dictionary"""
        # Get ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        
        # Create detector parameters
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Create detector
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.get_logger().info("ArUco detector initialized")

    def camera_callback(self, msg: Image):
        """Process incoming camera frames and detect ArUco markers"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(cv_image)
            
            # Create annotated frame for visualization
            annotated_frame = cv_image.copy()
            
            # Filter for target ID and extract detection
            target_detection = None
            if ids is not None and len(ids) > 0:
                target_detection = self._filter_target_marker(corners, ids)
            
            # Visualize detections
            if target_detection is not None:
                self._draw_detection(annotated_frame, target_detection)
                cv2.putText(
                    annotated_frame,
                    f"ArUco ID {self.target_id} detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    annotated_frame,
                    f"No ArUco ID {self.target_id}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )
            
            # Show the annotated frame
            cv2.imshow("ArUco Detection", annotated_frame)
            cv2.waitKey(1)
            
            # Publish detection message
            aruco_msg = self._create_detection_message(target_detection)
            self.aruco_publisher.publish(aruco_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def _filter_target_marker(self, corners, ids):
        """Filter for target marker ID and return largest if multiple found"""
        target_detections = []
        
        for i, marker_id in enumerate(ids):
            if marker_id[0] == self.target_id:
                # Calculate area of this marker
                corner_points = corners[i][0]
                area = cv2.contourArea(corner_points)
                target_detections.append({
                    'corners': corner_points,
                    'area': area,
                    'id': marker_id[0]
                })
        
        if not target_detections:
            return None
        
        # Return the largest marker if multiple found
        largest_detection = max(target_detections, key=lambda x: x['area'])
        return largest_detection

    def _draw_detection(self, image, detection):
        """Draw detected ArUco marker on image"""
        corners = detection['corners']
        marker_id = detection['id']
        
        # Draw marker outline
        corners_int = corners.astype(int)
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)
        
        # Draw corner circles
        for corner in corners_int:
            cv2.circle(image, tuple(corner), 5, (0, 0, 255), -1)
        
        # Calculate center and draw
        center_x = int(np.mean(corners[:, 0]))
        center_y = int(np.mean(corners[:, 1]))
        cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)
        
        # Draw ID label
        cv2.putText(
            image,
            f"ID: {marker_id}",
            (corners_int[0][0], corners_int[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    def _create_detection_message(self, detection) -> Aruco:
        """Create ArUco detection message"""
        aruco_msg = Aruco()
        
        if detection is None:
            # No detection - set all to -1
            aruco_msg.aruco_xmin = -1
            aruco_msg.aruco_xmax = -1
            aruco_msg.aruco_xcenter = -1
            aruco_msg.aruco_ymin = -1
            aruco_msg.aruco_ymax = -1
            aruco_msg.aruco_ycenter = -1
        else:
            # Extract bounding box from corners
            corners = detection['corners']
            x_coords = corners[:, 0]
            y_coords = corners[:, 1]
            
            aruco_msg.aruco_xmin = int(np.min(x_coords))
            aruco_msg.aruco_xmax = int(np.max(x_coords))
            aruco_msg.aruco_xcenter = int(np.mean(x_coords))
            aruco_msg.aruco_ymin = int(np.min(y_coords))
            aruco_msg.aruco_ymax = int(np.max(y_coords))
            aruco_msg.aruco_ycenter = int(np.mean(y_coords))
        
        return aruco_msg


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ArucoDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
