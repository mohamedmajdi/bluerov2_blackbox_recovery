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

        #show camera window parameter
        self.show_camera_window = self.config.get('show_camera_window')
        
        # Get initial parameter values
        self._update_parameters()
        
        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Load YOLO model
        self.model = self._load_model()
        self.get_logger().info("YOLOv11 model loaded successfully")
        
        # Create subscriber for camera input
        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image',
            self.camera_callback,
            10  # QoS queue size
        )
        
        self.camera_publisher = self.create_publisher(
            Image,
            'camera_detections',
            10
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
        
        #set camera window parameter
        show_camera_window_descriptor = ParameterDescriptor(
            description='Show camera window with detections (true/false)'
        )
        self.declare_parameter('show_camera_window', self.config['show_camera_window'], show_camera_window_descriptor)
        
        # Validate Handle Position (Boolean)
        validate_handle_descriptor = ParameterDescriptor(
            description='If true, checks if handle is physically close/inside the blackbox'
        )
        # using .get() with default False to prevent crash if not in YAML
        self.declare_parameter('validate_handle_position', 
                               self.config.get('validate_handle_position', False), 
                               validate_handle_descriptor)

        # Handle Position Tolerance (Float percentage, e.g., 0.2 = 20% expanison)
        tolerance_pos_descriptor = ParameterDescriptor(
            description='Percentage expansion of blackbox area to search for handle (0.0 - 1.0)',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.05)]
        )
        self.declare_parameter('handle_position_tolerance', 
                               self.config.get('handle_position_tolerance', 0.2), 
                               tolerance_pos_descriptor)

        self.get_logger().info("ROS parameters declared with range constraints")

    def _update_parameters(self):
        """Get current parameter values"""
        self.confidence = self.get_parameter('confidence').value
        self.handle_confidence = self.get_parameter('handle_confidence').value
        self.threshold = self.get_parameter('threshold').value
        self.handle_detection = self.get_parameter('handle_detection').value
        self.device = self.get_parameter('device').value
        self.validate_handle_position = self.get_parameter('validate_handle_position').value
        self.handle_position_tolerance = self.get_parameter('handle_position_tolerance').value


        self.get_logger().info(
            f"Parameters updated - confidence: {self.confidence}, "
            f"handle_confidence: {self.handle_confidence}, "
            f"threshold: {self.threshold}, "
            f"handle_detection: {self.handle_detection}, "
            f"device: {self.device}"
            f"Spatial Constraints - Validate: {self.validate_handle_position}, "
            f"Tolerance: {self.handle_position_tolerance}"
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
            elif param.name == 'show_camera_window':
                self.show_camera_window = param.value
                self.get_logger().info(f"Updated show_camera_window to {self.show_camera_window}")
            elif param.name == 'device':
                self.device = param.value
                self.get_logger().info(f"Updated device to {self.device}")
                # Note: changing device requires reloading the model
                self.get_logger().warn("Device change requires node restart to take effect")
            elif param.name == 'validate_handle_position':
                self.validate_handle_position = param.value
                self.get_logger().info(f"Updated validate_handle_position to {self.validate_handle_position}")
            elif param.name == 'handle_position_tolerance':
                self.handle_position_tolerance = param.value
                self.get_logger().info(f"Updated handle_position_tolerance to {self.handle_position_tolerance}")
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
                conf=0.3,  # Set to 0 to get all detections, we'll filter manually
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
            if self.show_camera_window == True:
                cv2.imshow("YOLOv11 Detections", annotated_frame)
                cv2.waitKey(1)
            
            #convert annotated frame to ROS Image message and publish
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.camera_publisher.publish(annotated_msg)
            
            # Extract detections and publish message
            detection_msg = self._extract_detections(filtered_detections, cv_image.shape)
            self.detection_publisher.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def _filter_detections(self, result, class_names):
            """
            Filter detections based on:
            1. Confidence thresholds
            2. Handle detection enabled flag
            3. (NEW) Spatial relationship: Handle must be inside/near Blackbox
            """
            if result.boxes is None or len(result.boxes) == 0:
                return []

            # Temporary lists to hold raw candidates before final selection
            blackbox_candidates = []
            handle_candidates = []

            # 1. Parse all boxes and separate them by class
            for box in result.boxes:
                x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                
                class_id = int(box.cls[0].cpu().numpy())
                class_name = class_names.get(class_id, 'unknown')
                confidence = float(box.conf[0].cpu().numpy())
                
                detection_data = {
                    'bbox': (x_min, y_min, x_max, y_max),
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': confidence
                }

                if class_name == 'blackbox':
                    if confidence >= self.confidence:
                        blackbox_candidates.append(detection_data)
                
                elif class_name == 'handle':
                    if self.handle_detection and confidence >= self.handle_confidence:
                        handle_candidates.append(detection_data)

            # 2. Select the BEST Blackbox
            # We assume the highest confidence blackbox is the "True" one
            best_blackbox = None
            if blackbox_candidates:
                # Sort by confidence descending and pick the first
                best_blackbox = sorted(blackbox_candidates, key=lambda x: x['confidence'], reverse=True)[0]
            
            # 3. Apply Spatial Constraints to Handles
            final_detections = []
            
            # If we have a blackbox, add it to final results
            if best_blackbox:
                final_detections.append(best_blackbox)

            # If spatial validation is ON
            if self.validate_handle_position:
                # If NO blackbox detected, we cannot validate handles -> discard all (Option A)
                if not best_blackbox:
                    return [] 
                
                # Define Valid Region (Blackbox + Tolerance)
                bb_xmin, bb_ymin, bb_xmax, bb_ymax = best_blackbox['bbox']
                bb_width = bb_xmax - bb_xmin
                bb_height = bb_ymax - bb_ymin
                
                # Calculate expansion amount
                margin_x = bb_width * self.handle_position_tolerance
                margin_y = bb_height * self.handle_position_tolerance
                
                # The allowed region for the handle CENTER
                valid_min_x = bb_xmin - margin_x
                valid_max_x = bb_xmax + margin_x
                valid_min_y = bb_ymin - margin_y
                valid_max_y = bb_ymax + margin_y

                valid_handles = []
                for handle in handle_candidates:
                    h_xmin, h_ymin, h_xmax, h_ymax = handle['bbox']
                    # Calculate handle center
                    h_center_x = (h_xmin + h_xmax) / 2
                    h_center_y = (h_ymin + h_ymax) / 2
                    
                    # CHECK: Is handle center inside the expanded blackbox?
                    if (valid_min_x <= h_center_x <= valid_max_x) and \
                    (valid_min_y <= h_center_y <= valid_max_y):
                        valid_handles.append(handle)
                
                # Update candidate list to only include spatially valid ones
                handle_candidates = valid_handles

            # 4. Select the BEST Handle from remaining candidates
            if handle_candidates:
                best_handle = sorted(handle_candidates, key=lambda x: x['confidence'], reverse=True)[0]
                final_detections.append(best_handle)

            return final_detections

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