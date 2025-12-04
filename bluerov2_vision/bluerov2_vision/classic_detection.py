#!/usr/bin/env python3

import os
import yaml
import cv2
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from bluerov2_interface.msg import Detection


class YOLOv11DetectionNode(Node):
    """YOLOv11 Segmentation Detection Node for BlueROV2"""

    def __init__(self):
        super().__init__('vision_node')
        
        # Initialize cv_bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Load configuration
        self.config = self._load_config()
        self.get_logger().info(f"Configuration loaded: {self.config}")
        
        # Load YOLOv11 model
        self.model = self._load_model()
        self.get_logger().info("YOLOv11 model loaded successfully")
        
        # Create subscriber for camera input
        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image',
            self.camera_callback,
            10  # QoS queue size
        )
        
        # Create publisher for detections
        self.detection_publisher = self.create_publisher(
            Detection,
            'detections',
            10
        )
        
        self.get_logger().info("BlueROV2 Detection Node initialized")

    def _load_config(self) -> dict:
        """Load configuration from YAML file"""
        try:
            # Get the package share directory
            script_dir = get_package_share_directory('bluerov2_vision')

            config_path = os.path.join(script_dir, 'config', 'detection_config.yaml')

            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            return config
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found at {config_path}")
            raise
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")
            raise

    def _load_model(self) -> YOLO:
        """Load YOLOv11 model from weights file"""
        try:
            # Get the directory of this script
            script_dir = get_package_share_directory('bluerov2_vision')
            
            # Construct path to weights
            weights_path = os.path.join(
                script_dir,
                self.config['model']['weights_path']
            )
            
            if not os.path.exists(weights_path):
                raise FileNotFoundError(f"Model weights not found at {weights_path}")
            
            # Load YOLO model
            model = YOLO(weights_path)
            
            # Set device
            device = self.config['model']['device']
            model.to(device)
            
            self.get_logger().info(f"Model loaded from {weights_path} on device {device}")
            return model
            
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            raise

    def camera_callback(self, msg: Image) -> None:
        """Process incoming camera frames and show detections"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv11 inference
            # Write the frame to a temporary file and process it
            tmp_path = '/tmp/tmp_yolo_frame.jpg'
            cv2.imwrite(tmp_path, cv_image)

            results = self.model.predict(
                source=tmp_path,
                conf=self.config['confidence'],
                iou=self.config['threshold'],
                show=False,
                stream=False,
                verbose=False
            )


            # Draw detections on image
            result = results[0]
            annotated_frame = cv_image.copy()
            class_names = {0: 'blackbox', 1: 'handle'}

            if result.boxes is not None and len(result.boxes) > 0:
                for box in result.boxes:
                    x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
                    x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = class_names.get(class_id, 'unknown')

                    # Draw bounding box
                    color = (0, 255, 0) if class_name == 'blackbox' else (0, 0, 255)
                    cv2.rectangle(annotated_frame, (x_min, y_min), (x_max, y_max), color, 2)

                    # Label text
                    label = f"{class_name}"
                    cv2.putText(
                        annotated_frame,
                        label,
                        (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )

            else:
                # If no detection, display text
                cv2.putText(
                    annotated_frame,
                    "No detections",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )

            # Show the annotated frame
            cv2.imshow("YOLOv11 Detections", annotated_frame)
            cv2.waitKey(1)

            # Extract detections and publish message
            detection_msg = self._extract_detections(result, cv_image.shape)
            self.detection_publisher.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")


    def _extract_detections(self, result, image_shape) -> Detection:
        """Extract detection coordinates from YOLOv11 results"""
        detection_msg = Detection()
        
        # Initialize all values to -1 (no detection)
        detection_msg.blackbox_xmin = -1
        detection_msg.blackbox_xmax = -1
        detection_msg.blackbox_xcenter = -1
        detection_msg.blackbox_ymin = -1
        detection_msg.blackbox_ymax = -1
        detection_msg.blackbox_ycenter = -1
        
        detection_msg.handle_xmin = -1
        detection_msg.handle_xmax = -1
        detection_msg.handle_xcenter = -1
        detection_msg.handle_ymin = -1
        detection_msg.handle_ymax = -1
        detection_msg.handle_ycenter = -1
        
        # Class names mapping
        class_names = {0: 'blackbox', 1: 'handle'}
        
        # Process detections
        if result.boxes is not None:
            for box in result.boxes:
                # Get bounding box coordinates
                x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                
                # Calculate center
                x_center = (x_min + x_max) // 2
                y_center = (y_min + y_max) // 2
                
                # Get class ID
                class_id = int(box.cls[0].cpu().numpy())
                class_name = class_names.get(class_id, 'unknown')
                
                # Update message based on class
                if class_name == 'blackbox':
                    detection_msg.blackbox_xmin = x_min
                    detection_msg.blackbox_xmax = x_max
                    detection_msg.blackbox_xcenter = x_center
                    detection_msg.blackbox_ymin = y_min
                    detection_msg.blackbox_ymax = y_max
                    detection_msg.blackbox_ycenter = y_center
                    
                elif class_name == 'handle':
                    detection_msg.handle_xmin = x_min
                    detection_msg.handle_xmax = x_max
                    detection_msg.handle_xcenter = x_center
                    detection_msg.handle_ymin = y_min
                    detection_msg.handle_ymax = y_max
                    detection_msg.handle_ycenter = y_center
        
        return detection_msg


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = YOLOv11DetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()