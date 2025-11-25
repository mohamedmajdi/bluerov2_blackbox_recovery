#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

from bluerov2_interface.msg import Detection


class BlackboxOrientationNode(Node):
    """Node to determine blackbox orientation based on width/height ratio"""

    def __init__(self):
        super().__init__('orientation_node')
        
        # Declare ROS parameter for ratio threshold
        self._declare_parameters()
        
        # Get initial parameter value
        self._update_parameters()
        
        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create subscriber for detections
        self.detection_subscription = self.create_subscription(
            Detection,
            'detections',
            self.detection_callback,
            10
        )
        
        # Create publisher for orientation
        self.orientation_publisher = self.create_publisher(
            String,
            'blackbox_orientation',
            10
        )
        
        self.get_logger().info("Blackbox Orientation Node initialized")
        self.get_logger().info(f"Ratio threshold: {self.ratio_threshold}")

    def _declare_parameters(self):
        """Declare ROS parameters with constraints"""
        ratio_threshold_descriptor = ParameterDescriptor(
            description='Width/Height ratio threshold for orientation detection (> 1.0)',
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=5.0, step=0.1)]
        )
        self.declare_parameter('ratio_threshold', 1.2, ratio_threshold_descriptor)
        
        self.get_logger().info("ROS parameters declared")

    def _update_parameters(self):
        """Get current parameter values"""
        self.ratio_threshold = self.get_parameter('ratio_threshold').value
        
        self.get_logger().info(f"Parameters updated - ratio_threshold: {self.ratio_threshold}")

    def parameters_callback(self, params):
        """Callback for parameter changes with validation"""
        for param in params:
            if param.name == 'ratio_threshold':
                if param.value < 1.0:
                    self.get_logger().error(
                        f"Parameter ratio_threshold value {param.value} must be >= 1.0"
                    )
                    return SetParametersResult(successful=False)
                
                self.ratio_threshold = param.value
                self.get_logger().info(f"Updated ratio_threshold to {self.ratio_threshold}")
        
        return SetParametersResult(successful=True)

    def detection_callback(self, msg: Detection):
        """Process detection message and determine orientation"""
        try:
            # Check if blackbox is detected
            if msg.blackbox_xmin == -1:
                # No detection
                orientation = "no_detection"
                self.get_logger().debug("No blackbox detected")
            else:
                # Calculate blackbox dimensions
                width = msg.blackbox_xmax - msg.blackbox_xmin
                height = msg.blackbox_ymax - msg.blackbox_ymin
                
                # Calculate orientation based on ratio
                orientation = self._calculate_orientation(width, height)
                
                self.get_logger().debug(
                    f"Blackbox dimensions - Width: {width}, Height: {height}, "
                    f"Ratio: {width/height:.2f}, Orientation: {orientation}"
                )
            
            # Publish orientation
            orientation_msg = String()
            orientation_msg.data = orientation
            self.orientation_publisher.publish(orientation_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing detection: {e}")

    def _calculate_orientation(self, width: int, height: int) -> str:
        """Calculate orientation based on width/height ratio"""
        if height == 0:
            self.get_logger().warn("Height is zero, cannot calculate ratio")
            return "no_detection"
        
        ratio = width / height
        
        # Determine orientation
        if ratio > self.ratio_threshold:
            # Width is significantly larger than height
            return "wide"
        elif (1.0 / ratio) > self.ratio_threshold:
            # Height is significantly larger than width
            return "standing"
        else:
            # Width and height are roughly equal
            return "straight"


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = BlackboxOrientationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
