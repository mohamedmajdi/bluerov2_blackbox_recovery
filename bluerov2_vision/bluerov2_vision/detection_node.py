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
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

from bluerov2_interface.msg import Detection


class YOLOv11DetectionNode(Node):
    """YOLOv11 Segmentation Detection Node for BlueROV2"""

    def __init__(self):
        super().__init__('vision_node')
        
        # Initialize cv_bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Load configuration from YAML
        self.config = self._load_config()
        self.get_logger().info(f"Configuration loaded: {self.config}")
        
        # Declare ROS parameters from YAML config with constraints
        self._declare_parameters()
        
        # Get initial parameter values
        self._update_parameters()
        
        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Load YOLOv11 model
        self.model = self._load_model()
        self.get_logger().info("YOLOv11 model loaded successfully")
        
        # Create subscriber for camera input
        self.camera_subscription = self.create_subscription(
            Image,
            '/bluerov/camera/image_color',
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

    def _declare_parameters(self):
        """Declare ROS parameters from config with range constraints"""
        
        # Confidence parameter with range [0.0, 1.0]
        confidence_descriptor = ParameterDescriptor(
            description='General confidence threshold for detection (0.0 - 1.0)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]
        )
        self.declare_parameter('confidence', self.config['confidence'], confidence_descriptor)
        
        # Handle confidence parameter with range [0.0, 1.0]
        handle_confidence_descriptor = ParameterDescriptor(
            description='Confidence threshold specifically for handle detection (0.0 - 1.0)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]
        )
        self.declare_parameter('handle_confidence', self.config['handle_confidence'], handle_confidence_descriptor)
        
        # IOU threshold parameter with range [0.0, 1.0]
        threshold_descriptor = ParameterDescriptor(
            description='IOU threshold for Non-Maximum Suppression (0.0 - 1.0)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]
        )
        self.declare_parameter('threshold', self.config['threshold'], threshold_descriptor)
        
        # Handle detection boolean parameter
        handle_detection_descriptor = ParameterDescriptor(
            description='Enable or disable handle detection (true/false)'
        )
        self.declare_parameter('handle_detection', self.config['handle_detection'], handle_detection_descriptor)
        
        # Device parameter (integer or string)
        device_descriptor = ParameterDescriptor(
            description='Device for YOLO inference: 0 for GPU, "cpu" for CPU'
        )
        self.declare_parameter('device', self.config['model']['device'], device_descriptor)
        
        self.get_logger().info("ROS parameters declared with range constraints")

    def _update_parameters(self):
        """Get current parameter values"""
        self.confidence = self.get_parameter('confidence').value
        self.handle_confidence = self.get_parameter('handle_confidence').value
        self.threshold = self.get_parameter('threshold').value
        self.handle_detection = self.get_parameter('handle_detection').value
        self.device = self.get_parameter('device').value
        
        self.get_logger().info(
            f"Parameters updated - confidence: {self.confidence}, "
            f"handle_confidence: {self.handle_confidence}, "
            f"threshold: {self.threshold}, "
            f"handle_detection: {self.handle_detection}, "
            f"device: {self.device}"
        )

    def parameters_callback(self, params):
        """Callback for parameter changes with validation"""
        for param in params:
            # Validate range for float parameters
            if param.name in ['confidence', 'handle_confidence', 'threshold']:
                if not (0.0 <= param.value <= 1.0):
                    self.get_logger().error(
                        f"Parameter {param.name} value {param.value} out of range [0.0, 1.0]"
                    )
                    return SetParametersResult(successful=False)
            
            if param.name == 'confidence':
                self.confidence = param.value
                self.get_logger().info(f"Updated confidence to {self.confidence}")
            elif param.name == 'handle_confidence':
                self.handle_confidence = param.value
                self.get_logger().info(f"Updated handle_confidence to {self.handle_confidence}")
            elif param.name == 'threshold':
                self.threshold = param.value
                self.get_logger().info(f"Updated threshold to {self.threshold}")
            elif param.name == 'handle_detection':
                self.handle_detection = param.value
                self.get_logger().info(f"Updated handle_detection to {self.handle_detection}")
            elif param.name == 'device':
                self.device = param.value
                self.get_logger().info(f"Updated device to {self.device}")
                # Note: changing device requires reloading the model
                self.get_logger().warn("Device change requires node restart to take effect")
        
        return SetParametersResult(successful=True)

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
            model.to(self.device)
            
            self.get_logger().info(f"Model loaded from {weights_path} on device {self.device}")
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
                conf=0.0,  # Set to 0 to get all detections, we'll filter manually
                iou=self.threshold,
                show=False,
                stream=False,
                verbose=False
            )

            # Draw detections on image
            result = results[0]
            annotated_frame = cv_image.copy()
            class_names = {0: 'blackbox', 1: 'handle'}

            # Filter detections based on custom confidence thresholds
            filtered_detections = self._filter_detections(result, class_names)

            if filtered_detections:
                for detection in filtered_detections:
                    x_min, y_min, x_max, y_max = detection['bbox']
                    class_name = detection['class_name']
                    conf = detection['confidence']

                    # Draw bounding box
                    color = (0, 255, 0) if class_name == 'blackbox' else (0, 0, 255)
                    cv2.rectangle(annotated_frame, (x_min, y_min), (x_max, y_max), color, 2)

                    # Label text
                    label = f"{class_name}: {conf:.2f}"
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
            detection_msg = self._extract_detections(filtered_detections, cv_image.shape)
            self.detection_publisher.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def _filter_detections(self, result, class_names):
        """Filter detections based on confidence thresholds and handle_detection flag"""
        if result.boxes is None or len(result.boxes) == 0:
            return []

        # Collect all detections with their information
        all_detections = []
        
        for box in result.boxes:
            x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            
            class_id = int(box.cls[0].cpu().numpy())
            class_name = class_names.get(class_id, 'unknown')
            confidence = float(box.conf[0].cpu().numpy())
            
            # Apply confidence threshold based on class
            if class_name == 'handle':
                # Check if handle detection is enabled
                if not self.handle_detection:
                    continue  # Skip handle detection
                # Use handle_confidence threshold
                if confidence < self.handle_confidence:
                    continue
            else:
                # Use general confidence threshold for other objects
                if confidence < self.confidence:
                    continue
            
            all_detections.append({
                'bbox': (x_min, y_min, x_max, y_max),
                'class_id': class_id,
                'class_name': class_name,
                'confidence': confidence
            })

        # Keep only the highest confidence detection per class
        filtered_detections = {}
        for det in all_detections:
            class_name = det['class_name']
            if class_name not in filtered_detections:
                filtered_detections[class_name] = det
            else:
                # Keep the one with higher confidence
                if det['confidence'] > filtered_detections[class_name]['confidence']:
                    filtered_detections[class_name] = det

        return list(filtered_detections.values())

    def _extract_detections(self, filtered_detections, image_shape) -> Detection:
        """Extract detection coordinates from filtered detections"""
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
        
        # Process filtered detections
        for detection in filtered_detections:
            x_min, y_min, x_max, y_max = detection['bbox']
            class_name = detection['class_name']
            
            # Calculate center
            x_center = (x_min + x_max) // 2
            y_center = (y_min + y_max) // 2
            
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
