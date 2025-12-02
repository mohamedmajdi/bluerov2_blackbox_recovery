#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from mavros_msgs.msg import OverrideRCIn, Mavlink
from bluerov2_interface.msg import Detection
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
import tf_transformations


# camera parameters
u0 = 480
v0 = 270
lx = 455
ly = 455
kud = 0.00683 
kdu = -0.01424     
    
# convert a pixel coordinate to meters given linear calibration parameters
def convert2meter(pt, u0, v0, lx, ly):
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a pixel coordinate to meters using default calibration parameters
def convertOnePoint2meter(pt):
    global u0, v0, lx, ly
    return (float(pt[0])-u0)/lx, (float(pt[1])-v0)/ly

# convert a list of pixels coordinates to meters using default calibration parameters
def convertListPoint2meter(points):
    global u0, v0, lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n, 2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt, u0, v0, lx, ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter
    

class Controller(Node):
    def __init__(self):
        super().__init__('Vision_Controller')

        # Initialize with None to be set from image dimensions
        self.desired_point = None
        self.image_center_set = False

        self.track_points = None
        self.prev_gray = None

        self.roi_width = 0
        self.roi_height = 0
        self.frames_since_reinit = 0
 
        self.last_handle_bbox = None
        
        # VO-based handle tracking (fallback)
        self.use_of_tracking = False
        self.last_known_handle_pos_cam = None  # Handle position in camera frame when last seen
        self.vo_pose = None  # Current VO pose
        self.vo_pose_at_last_detection = None  # VO pose when handle was last detected
        self.handle_lost_frames = 0
        self.handle_lost_threshold = 10  # Number of frames before switching to VO tracking
        
        # Tracking mode indicator
        self.tracking_mode = "NONE"  # Can be: "YOLO", "FEATURES", "BOX"

        # ============== DECLARE DYNAMIC PARAMETERS ==============
        
        # Desired point parameters (will be auto-set from image center on first frame)
        self.declare_parameter('desired_point_x', -1.0,
            ParameterDescriptor(description='Desired X pixel coordinate (-1 = auto image center)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_y', -1.0,
            ParameterDescriptor(description='Desired Y pixel coordinate (-1 = auto image center)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_z', 0.5,
            ParameterDescriptor(description='Desired distance (depth) in meters',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Control gains for [surge, sway, heave, yaw]
        self.declare_parameter('gain_surge', 0.6,
            ParameterDescriptor(description='Visual servoing gain for surge (forward/back)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_sway', 0.5,
            ParameterDescriptor(description='Visual servoing gain for sway (left/right)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_heave', 0.5,
            ParameterDescriptor(description='Visual servoing gain for heave (up/down)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('gain_yaw', 0.5,
            ParameterDescriptor(description='Visual servoing gain for yaw (rotation)',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Velocity limits
        self.declare_parameter('v_linear_max', 0.3,
            ParameterDescriptor(description='Maximum linear velocity (m/s)',
                              type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('v_angular_max', 0.5,
            ParameterDescriptor(description='Maximum angular velocity (rad/s)',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Floatability compensation
        self.declare_parameter('floatability', -0.18,
            ParameterDescriptor(description='Heave compensation for buoyancy/floatability',
                              type=ParameterType.PARAMETER_DOUBLE))
        
        # Sign inversion flags for each DOF
        self.declare_parameter('invert_surge', False,
            ParameterDescriptor(description='Invert surge control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_sway', False,
            ParameterDescriptor(description='Invert sway control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_heave', True,
            ParameterDescriptor(description='Invert heave control direction',
                              type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('invert_yaw', True,
            ParameterDescriptor(description='Invert yaw control direction',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Control enable/disable
        self.declare_parameter('enable_visual_servoing', False,
            ParameterDescriptor(description='Enable/disable visual servoing control',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Track handle center instead of box center
        self.declare_parameter('track_handle', False,
            ParameterDescriptor(description='Track handle instead of box center',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Enable optical flow tracking as fallback
        self.declare_parameter('enable_of_tracking', True,
            ParameterDescriptor(description='Enable optical flow tracking when handle feature tracking fails',
                              type=ParameterType.PARAMETER_BOOL))
        
        # Send to bringup topic instead of direct control
        self.declare_parameter('send_to_bringup', False,
            ParameterDescriptor(description='Send commands to bringup topic instead of direct robot control',
                            type=ParameterType.PARAMETER_BOOL))
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self._on_set_parameters)
        
        # Initialize parameters
        self.update_parameters()

        self.distance = 1.0
        self.blackbox_xmin = 0
        self.blackbox_xmax = 0
        self.blackbox_ymin = 0
        self.blackbox_ymax = 0
        self.handle_xmin = 0

        # Visual servo parameters
        self.Camera_cooriction_surge = 1500
        self.Camera_cooriction_sway = 1500
        self.Camera_cooriction_heave = 1500
        self.Camera_cooriction_yaw = 1500
        self.Camera_cooriction_roll = 1500
        self.Camera_cooriction_pitch = 1500

        # Initialize error tracking
        self.s_error = np.zeros(2)
        self.z_error = 0.0

        # ---------------- SUBSCRIBERS ----------------

        #sim topic:   /bluerov/camera/image_color
        # real robot topic: camera/image
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        # OpenCV Bridge
        self.bridge = CvBridge()

        self.boxDetections = self.create_subscription(
            Detection,
            'detections',
            self.color_video_tracking_callback,
            10
        )
        self.boxDetections 

        # ---------------- PUBLISHER ----------------
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)

        self.pub_pwm_vs = self.create_publisher(Int16MultiArray, "controller/pwm_vs", 10)


        timer_period = 0.05  # 50 msec - 20 Hz   (or 0.5 ? )
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Tracker Controller Active with Dynamic Parameters")
        self.log_current_parameters()

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'desired_point_x':
                self.desired_point[0] = p.value
            elif p.name == 'desired_point_y':
                self.desired_point[1] = p.value
            elif p.name == 'desired_point_z':
                self.desired_point[2] = p.value
            elif p.name == 'gain_surge':
                self.gains[0] = p.value
            elif p.name == 'gain_sway':
                self.gains[1] = p.value
            elif p.name == 'gain_heave':
                self.gains[2] = p.value
            elif p.name == 'gain_yaw':
                self.gains[3] = p.value
            elif p.name == 'v_linear_max':
                self.v_linear_max = p.value
            elif p.name == 'v_angular_max':
                self.v_angular_max = p.value
            elif p.name == 'floatability':
                self.floatability = p.value
            elif p.name == 'invert_surge':
                self.invert_surge = p.value
            elif p.name == 'invert_sway':
                self.invert_sway = p.value
            elif p.name == 'invert_heave':
                self.invert_heave = p.value
            elif p.name == 'invert_yaw':
                self.invert_yaw = p.value
            elif p.name == 'enable_visual_servoing':
                self.enable_visual_servoing = p.value
                self.get_logger().info(f"Visual servoing {'ENABLED' if p.value else 'DISABLED'}")
            elif p.name == 'track_handle':
                self.track_handle = p.value
                self.get_logger().info(f"Target switched to {'HANDLE' if p.value else 'BOX CENTER'}")
            elif p.name == 'enable_of_tracking':
                self.use_of_tracking = p.value
                self.get_logger().info(f"OF-based tracking fallback {'ENABLED' if p.value else 'DISABLED'}")
            elif p.name == 'send_to_bringup':
                self.send_to_bringup = p.value
                self.get_logger().info(f"Send to bringup: {'ENABLED' if p.value else 'DISABLED'}")
        return SetParametersResult(successful=True)

    def update_parameters(self):
        """Update internal variables from ROS2 parameters"""
        # Get desired point
        desired_x = self.get_parameter('desired_point_x').value
        desired_y = self.get_parameter('desired_point_y').value
        desired_z = self.get_parameter('desired_point_z').value
        
        # Initialize desired_point array if needed
        if self.desired_point is None:
            self.desired_point = np.array([desired_x, desired_y, desired_z])
        else:
            # Only update from parameters if they're not -1 (auto mode)
            if desired_x >= 0:
                self.desired_point[0] = desired_x
            if desired_y >= 0:
                self.desired_point[1] = desired_y
            self.desired_point[2] = desired_z
        
        # Get control gains
        self.gains = np.array([
            self.get_parameter('gain_surge').value,
            self.get_parameter('gain_sway').value,
            self.get_parameter('gain_heave').value,
            self.get_parameter('gain_yaw').value
        ])
        
        # Get velocity limits
        self.v_linear_max = self.get_parameter('v_linear_max').value
        self.v_angular_max = self.get_parameter('v_angular_max').value
        
        # Get floatability compensation
        self.floatability = self.get_parameter('floatability').value
        
        # Get inversion flags
        self.invert_surge = self.get_parameter('invert_surge').value
        self.invert_sway = self.get_parameter('invert_sway').value
        self.invert_heave = self.get_parameter('invert_heave').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        
        # Get control enable flag
        self.enable_visual_servoing = self.get_parameter('enable_visual_servoing').value

        # Get target selection
        self.track_handle = self.get_parameter('track_handle').value

        # Get OF tracking enable flag
        self.enable_of_tracking = self.get_parameter('enable_of_tracking').value

        # Get bringup mode flag
        self.send_to_bringup = self.get_parameter('send_to_bringup').value

    def log_current_parameters(self):
        """Log all current parameter values"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Current Parameters:")
        self.get_logger().info(f"  Desired Point: [{self.desired_point[0]:.1f}, {self.desired_point[1]:.1f}, {self.desired_point[2]:.2f}]")
        self.get_logger().info(f"  Control Gains: [{self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[2]:.2f}, {self.gains[3]:.2f}]")
        self.get_logger().info(f"  Linear Max Vel: {self.v_linear_max:.2f} m/s")
        self.get_logger().info(f"  Angular Max Vel: {self.v_angular_max:.2f} rad/s")
        self.get_logger().info(f"  Floatability: {self.floatability:.2f}")
        self.get_logger().info(f"  Inversions: Surge={self.invert_surge}, Sway={self.invert_sway}, "
                             f"Heave={self.invert_heave}, Yaw={self.invert_yaw}")
        self.get_logger().info(f"  Visual Servoing: {'ENABLED' if self.enable_visual_servoing else 'DISABLED'}")
        try:
            use_sim = self.get_parameter('use_sim_time').value
            self.get_logger().info(f"  Use Sim Time: {use_sim}")
        except:
            pass
        self.get_logger().info(f"  Track Handle: {self.track_handle}")
        self.get_logger().info(f"  Send to Bringup: {self.send_to_bringup}")
        self.get_logger().info("=" * 60)

    def timer_callback(self):
        '''
        Time step at a fixed rate to execute control logic.
        '''
            
        if self.enable_visual_servoing:
            if self.send_to_bringup:
                # Send to bringup topic instead of direct robot control
                self.publish_to_bringup()
            else:
                # Direct robot control
                self.setOverrideRCIN(self.Camera_cooriction_pitch, self.Camera_cooriction_roll, 
                                self.Camera_cooriction_heave, self.Camera_cooriction_yaw, 
                                self.Camera_cooriction_surge, self.Camera_cooriction_sway)
        else:
            # When disabled, do nothing
            return

    def image_callback(self, msg):
        try:
            self.image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')    
            image_height, image_width, image_channels = self.image_np.shape
            
            # Set desired point to image center on first frame if not manually set
            if not self.image_center_set:
                param_x = self.get_parameter('desired_point_x').value
                param_y = self.get_parameter('desired_point_y').value
                
                # If parameters are still -1 (auto mode), set to image center
                if param_x < 0 or param_y < 0:
                    self.desired_point = np.array([
                        image_width // 2,
                        image_height // 2,
                        self.get_parameter('desired_point_z').value
                    ])
                    self.get_logger().info(
                        f"Auto-set desired point to image center: "
                        f"[{self.desired_point[0]:.1f}, {self.desired_point[1]:.1f}]"
                    )
                self.image_center_set = True
            
            # Extract width and height of the blackbox from the detected bounding box
            (width_px, height_px) = (self.blackbox_xmax - self.blackbox_xmin, 
                                     self.blackbox_ymax - self.blackbox_ymin)
            # Determine which dimension is the width ( choose max or min?)
            blackbox_width_px = min(width_px, height_px)
            
            #blackbox_width_px=self.blackbox_xmax - self.blackbox_xmin
            #future improvement: use handle width or height when very close?
            
            if blackbox_width_px > 0:
                known_width_m = 0.14  # Known width of the blackbox in meters (handle width is 10 cm)
                focal_length = 455
                known_length_m=0.30
                self.distance = (known_width_m * focal_length) / blackbox_width_px
                #self.distance = (known_length_m * focal_length) / blackbox_width_px
            else:
                # No valid detection - stop and rotate to search (only if servoing is enabled)
                if self.enable_visual_servoing:
                    
                    self.Camera_cooriction_surge = 1500
                    self.Camera_cooriction_sway = 1500
                    self.Camera_cooriction_heave = 1500
                    self.Camera_cooriction_yaw = 1500
                    #self.Camera_cooriction_surge = self.mapValueScalSat(0.0)
                    #self.Camera_cooriction_sway = self.mapValueScalSat(0.0)
                    # self.Camera_cooriction_heave = self.mapValueScalSat(-0.25)

                    #self.Camera_cooriction_heave = self.mapValueScalSat(self.floatability)
                    
                    # search_rotation_velocity = 0.1  
                    # search_rotation_velocity = 0.0
                    # self.Camera_cooriction_yaw = self.mapValueScalSat(search_rotation_velocity)
                    
                    # self.get_logger().warn("No detection - searching (rotating)...", 
                    #                     throttle_duration_sec=1.0)
                    self.get_logger().warn("No detection ..", 
                                        throttle_duration_sec=1.0)
                else:
                    # Visual servoing disabled - maintain neutral position
                    self.Camera_cooriction_surge = 1500
                    self.Camera_cooriction_sway = 1500
                    self.Camera_cooriction_heave = 1500
                    self.Camera_cooriction_yaw = 1500
                    return

            self.draw_visualization()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def mapValueScalSat(self, value):
        '''Map the value of the joystick analog from -1 to 1 to a pwm value from 1100 to 1900'''
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)
    
    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, 
                        channel_forward, channel_lateral):
        '''Override RC channels for motor control'''
        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)
        msg_override.channels[1] = np.uint(channel_roll)
        msg_override.channels[2] = np.uint(channel_throttle)
        msg_override.channels[3] = np.uint(channel_yaw)
        msg_override.channels[4] = np.uint(channel_forward)
        msg_override.channels[5] = np.uint(channel_lateral)
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500

        self.pub_msg_override.publish(msg_override)

    def publish_to_bringup(self):
        '''Publish PWM commands to bringup topic'''
        msg = Int16MultiArray()
        msg.data = [
            int(self.Camera_cooriction_pitch),
            int(self.Camera_cooriction_roll),
            int(self.Camera_cooriction_heave),
            int(self.Camera_cooriction_yaw),
            int(self.Camera_cooriction_surge),
            int(self.Camera_cooriction_sway)
        ]
        self.pub_pwm_vs.publish(msg)

    def color_video_tracking_callback(self, msg):
        """
        Visual servoing callback for BlueROV control using surge, sway, heave, and yaw.
        Now with multi-level handle tracking: YOLO -> Features -> OF -> Box fallback
        """
        # If visual servoing is disabled, set neutral commands and return
        if not self.enable_visual_servoing:
            self.Camera_cooriction_surge = 1500
            self.Camera_cooriction_sway = 1500
            self.Camera_cooriction_heave = 1500
            self.Camera_cooriction_yaw = 1500
            self.tracking_mode = "DISABLED"
            return
        
        # Extract detection data
        self.blackbox_xmin = msg.blackbox_xmin
        self.blackbox_xmax = msg.blackbox_xmax
        self.blackbox_xcenter = msg.blackbox_xcenter
        self.blackbox_ymin = msg.blackbox_ymin
        self.blackbox_ymax = msg.blackbox_ymax
        self.blackbox_ycenter = msg.blackbox_ycenter
        
        self.handle_xmin = msg.handle_xmin
        self.handle_xmax = msg.handle_xmax
        self.handle_xcenter = msg.handle_xcenter
        self.handle_ymin = msg.handle_ymin
        self.handle_ymax = msg.handle_ymax
        self.handle_ycenter = msg.handle_ycenter
        
        # Depth estimation
        Z = max(self.distance, 0.1)
        
        # Determine if handle is currently detected by YOLO
        handle_detected = (self.handle_xcenter > 0 and self.handle_xmax > 0)
        
         # ============ MULTI-LEVEL TRACKING LOGIC ============
    
        if self.track_handle and handle_detected:
            # Level 1: YOLO Detection (PRIMARY)
            self.handle_lost_frames = 0
            
            # Store handle bounding box for feature tracking
            self.last_handle_bbox = (self.handle_xmin, self.handle_ymin, 
                                    self.handle_xmax, self.handle_ymax)
            
            # Store the handle position in camera frame
            x_handle, y_handle = convertOnePoint2meter([self.handle_xcenter, self.handle_ycenter])
            self.last_known_handle_pos_cam = np.array([x_handle * Z, y_handle * Z, Z])
            
            # Store the handle position in pixels
            self.last_known_handle_pos_pixels = np.array([self.handle_xcenter, self.handle_ycenter])
            
            # Track the detected handle
            x, y = x_handle, y_handle
            self.tracking_mode = "YOLO-HANDLE"
            
            
        elif self.track_handle and not handle_detected:
            # Handle was requested but not detected by YOLO
            self.handle_lost_frames += 1
            
            # Level 2: Feature Tracking (SECONDARY)
            if self.handle_lost_frames > 2 and hasattr(self, 'image_np') and self.image_np is not None:
                projected_pos = self.track_handle_features(
                    self.image_np, 
                    self.last_handle_bbox
                )
                
                # Validate the tracked position
                valid_position = False
                if projected_pos is not None:
                    # Check if position makes sense relative to box
                    if self.blackbox_ymin > 0:
                        # Handle should be above box top (with tolerance)
                        if projected_pos[1] < self.blackbox_ymin + 150:  # 150px tolerance
                            valid_position = True
                        else:
                            self.get_logger().warn(
                                f"Feature tracking invalid: y={projected_pos[1]:.1f} below box_top={self.blackbox_ymin:.1f}",
                                throttle_duration_sec=1.0
                            )
                    else:
                        # No box detection to validate against, accept the position
                        valid_position = True
                
                if valid_position and projected_pos is not None:
                    # Feature tracking successful and validated
                    x, y = convertOnePoint2meter(projected_pos)
                    self.tracking_mode = "FEATURE-HANDLE"
                    self.get_logger().info(
                        f"Feature tracking: handle at ({projected_pos[0]:.1f}, {projected_pos[1]:.1f})",
                        throttle_duration_sec=1.0
                    )
                else:
                    # Level 3: Box Center Fallback (LAST RESORT)
                    x, y = convertOnePoint2meter([self.blackbox_xcenter, self.blackbox_ymin])
                    self.tracking_mode = "BOX-FALLBACK"
                    self.get_logger().warn(
                        "Feature tracking failed, using box top center",
                        throttle_duration_sec=1.0
                    )
            else:
                # Not enough frames lost yet, use box top center
                x, y = convertOnePoint2meter([self.blackbox_xcenter, self.blackbox_ymin])
                self.tracking_mode = "BOX-EARLY"
        else:
            # Track top center of box (default or when track_handle is disabled)
            box_top_center_x = self.blackbox_xcenter
            box_top_center_y = self.blackbox_ymin  
            x, y = convertOnePoint2meter([box_top_center_x, box_top_center_y])
            self.tracking_mode = "BOX-TOP"
            self.handle_lost_frames = 0

        
        # ============ VISUAL SERVOING CONTROL ============
        
        xd, yd = convertOnePoint2meter(self.desired_point[:2])
        zd = self.desired_point[2]
        
        # Feature error
        self.s_error = np.array([x - xd, y - yd])
        self.z_error = Z - zd
        
        # Interaction matrix
        fx = 1.0
        fy = 1.0
        
        L_full = np.array([
            [-fx/Z, 0, fx*x/Z, fx*x*y, -fx*(1+x**2), fx*y],
            [0, -fy/Z, fy*y/Z, fy*(1+y**2), -fy*x*y, -fy*x]
        ])
        
        L_reduced = L_full[:, [0, 1, 2, 5]]
        L_depth = np.array([[0, 0, -1, 0]])
        L = np.vstack([L_reduced, L_depth])
        
        error_combined = np.array([self.s_error[0], self.s_error[1], self.z_error])
        
        # Compute velocity using DYNAMIC GAINS
        L_inv = np.linalg.pinv(L)
        v_cam_4d = -self.gains * (L_inv @ error_combined)
        # print("v_cam_4d shape:", v_cam_4d.shape)
        
        # Transform to ROV frame
        H = self.camera_to_rov_transform()
        v_rov_4d = self.transform_velocity_4dof(v_cam_4d, H)
        
        # Apply DYNAMIC VELOCITY LIMITS
        v_rov_4d[0:3] = np.clip(v_rov_4d[0:3], -self.v_linear_max, self.v_linear_max)
        v_rov_4d[3] = np.clip(v_rov_4d[3], -self.v_angular_max, self.v_angular_max)
        
        # Apply inversion flags
        surge_sign = -1.0 if self.invert_surge else 1.0
        sway_sign = -1.0 if self.invert_sway else 1.0
        heave_sign = -1.0 if self.invert_heave else 1.0
        yaw_sign = -1.0 if self.invert_yaw else 1.0
        
        # Store for RC override with inversion and floatability
        self.Camera_cooriction_surge = self.mapValueScalSat(surge_sign * v_rov_4d[0])
        self.Camera_cooriction_sway = self.mapValueScalSat(sway_sign * v_rov_4d[1])
        self.Camera_cooriction_heave = self.mapValueScalSat(heave_sign * v_rov_4d[2] + self.floatability)
        self.Camera_cooriction_yaw = self.mapValueScalSat(yaw_sign * v_rov_4d[3])


    def transform_velocity_4dof(self, v_cam, H):
        """Transform 4DOF velocity from camera frame to ROV body frame"""
        R = H[0:3, 0:3]
        v_linear_cam = v_cam[0:3]
        v_linear_rov = R @ v_linear_cam
        yaw_rov = v_cam[3]
        return np.array([v_linear_rov[0], v_linear_rov[1], v_linear_rov[2], yaw_rov])

    def camera_to_rov_transform(self):
        """Returns transformation matrix from camera frame to ROV body frame"""
        R = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        t = np.array([0.2, 0.0, 0.0])
        H = np.eye(4)
        H[0:3, 0:3] = R
        H[0:3, 3] = t
        return H
    
    def overlay_points(self, image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
        cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
        position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

    def draw_visualization(self):
        """Draw detection visualization with bounding boxes and tracking points"""
        try:
            if not hasattr(self, 'image_np') or self.image_np is None:
                return
            
            desired_x = int(self.desired_point[0])
            desired_y = int(self.desired_point[1])
            
            self.overlay_points(self.image_np, [desired_x, desired_y], 255, 0, 0, 
                            'desired point', scale=0.7, offsetx=10, offsety=-10)
            
            if self.blackbox_xmin > 0 and self.blackbox_xmax > 0:
                # Draw box detection (yellow rectangle)
                cv2.rectangle(self.image_np,
                            (int(self.blackbox_xmin), int(self.blackbox_ymin)),
                            (int(self.blackbox_xmax), int(self.blackbox_ymax)),
                            (0, 255, 255), 2)
                
                if "BOX" in self.tracking_mode:
                    # We track BOX TOP CENTER, not box center
                    detected_x = int(self.blackbox_xcenter)
                    detected_y = int(self.blackbox_ymin)  # Changed from blackbox_ycenter!
                    label = 'box top center'
                else:
                    # For other modes, show actual box center for reference
                    detected_x = int(self.blackbox_xcenter)
                    detected_y = int(self.blackbox_ycenter)
                    label = 'box center'
                            
                self.overlay_points(self.image_np, [detected_x, detected_y], 0, 255, 0,
                                'box center', scale=0.7, offsetx=10, offsety=10)
                
                error_x = detected_x - desired_x
                error_y = detected_y - desired_y
                
                info_y = 30
                line_height = 30
                
                error_text = f"Pixel Error in x,y: ({error_x:.0f}, {error_y:.0f})"
                cv2.putText(self.image_np, error_text,
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, error_text,
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                depth_text = f"Estimated Distance to box: {self.distance:.2f} m"
                cv2.putText(self.image_np, depth_text,
                        (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, depth_text,
                        (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                box_width = self.blackbox_xmax - self.blackbox_xmin
                box_height = self.blackbox_ymax - self.blackbox_ymin
                box_text = f"Box size: {box_width:.0f} x {box_height:.0f} px"
                cv2.putText(self.image_np, box_text,
                        (10, info_y + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, box_text,
                        (10, info_y + 2*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                vel_text = (f"PWM - Surge: {self.Camera_cooriction_surge}, "
                            f"Sway: {self.Camera_cooriction_sway}, "
                            f"Heave: {self.Camera_cooriction_heave}, "
                            f"Yaw: {self.Camera_cooriction_yaw}")
                cv2.putText(self.image_np, vel_text,
                        (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, vel_text,
                        (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1) 
                
                img_error_text = f"Image Error: ({self.s_error[0]:.4f}, {self.s_error[1]:.4f}), Depth Error: {self.z_error:.4f}"
                cv2.putText(self.image_np, img_error_text,
                        (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, img_error_text,
                        (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Display current gains and floatability
                gains_text = f"Gains [S,Sw,H,Y]: [{self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[2]:.2f}, {self.gains[3]:.2f}] | Float: {self.floatability:.2f}"
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Display tracking mode
                status_text = (f"VS: {'ON' if self.enable_visual_servoing else 'OFF'} | "
                            f"Mode: {self.tracking_mode} | Lost: {self.handle_lost_frames} | "
                            f"Inv: S={self.invert_surge} Sw={self.invert_sway} H={self.invert_heave} Y={self.invert_yaw}")
                cv2.putText(self.image_np, status_text,
                        (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, status_text,
                        (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # ============ HANDLE VISUALIZATION ============
            
            # Draw YOLO-detected handle (if available)
            if self.handle_xmin > 0 and self.handle_xmax > 0:
                # Magenta rectangle for handle detection
                cv2.rectangle(self.image_np,
                            (int(self.handle_xmin), int(self.handle_ymin)),
                            (int(self.handle_xmax), int(self.handle_ymax)),
                            (255, 0, 255), 2)
                
                handle_x = int(self.handle_xcenter)
                handle_y = int(self.handle_ycenter)
                
                # Draw handle center point
                cv2.circle(self.image_np, (handle_x, handle_y), 8, (255, 0, 255), -1)
                cv2.putText(self.image_np, 'YOLO Handle', 
                        (handle_x + 15, handle_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # ============ TRACKING METHOD VISUALIZATION ============
            
            # Draw indicator in top-left corner with color coding
            tracking_indicator_y = 30
            
            if self.tracking_mode == "YOLO-HANDLE":
                # Magenta indicator for YOLO tracking
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (255, 0, 255), -1)
                cv2.putText(self.image_np, "YOLO", (860, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Draw active tracking point (magenta circle on handle)
                if self.handle_xmin > 0:
                    handle_x = int(self.handle_xcenter)
                    handle_y = int(self.handle_ycenter)
                    cv2.circle(self.image_np, (handle_x, handle_y), 12, (255, 0, 255), 3)
            
            elif self.tracking_mode == "FEATURE-HANDLE":
                # Green indicator for feature tracking
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (0, 255, 0), -1)
                cv2.putText(self.image_np, "FEAT", (860, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                # Draw last known handle bbox if available
                if self.last_handle_bbox is not None:
                    xmin, ymin, xmax, ymax = self.last_handle_bbox
                    cv2.rectangle(self.image_np,
                                (int(xmin), int(ymin)),
                                (int(xmax), int(ymax)),
                                (0, 255, 0), 2)
                    cv2.putText(self.image_np, 'Last Known ROI', 
                            (int(xmin), int(ymin) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Try to get current tracked position
                projected_pos = self.track_handle_features(self.image_np, self.last_handle_bbox)
                if projected_pos is not None:
                    proj_x, proj_y = int(projected_pos[0]), int(projected_pos[1])
                    # Draw tracked point (green)
                    cv2.circle(self.image_np, (proj_x, proj_y), 12, (0, 255, 0), 3)
                    cv2.circle(self.image_np, (proj_x, proj_y), 5, (0, 255, 0), -1)
                    cv2.putText(self.image_np, 'Tracked', 
                            (proj_x + 15, proj_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Draw line from desired point to tracked point
                    cv2.line(self.image_np, 
                            (int(self.desired_point[0]), int(self.desired_point[1])),
                            (proj_x, proj_y),
                            (0, 255, 0), 2)
        
                        
            elif "BOX" in self.tracking_mode:
                # Yellow indicator for box tracking
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (0, 255, 255), -1)
                cv2.putText(self.image_np, "BOX", (865, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                # Highlight box center being tracked
                if self.blackbox_xmin > 0:
                    box_x = int(self.blackbox_xcenter)
                    box_y = int(self.blackbox_ycenter)
                    cv2.circle(self.image_np, (box_x, box_y), 12, (0, 255, 255), 3)
            
            else:
                # Gray indicator for disabled/unknown
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (128, 128, 128), -1)
                cv2.putText(self.image_np, "OFF", (865, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)            
            
            cv2.imshow("Visual Servoing", self.image_np)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
    

   
    
    def track_handle_features(self, current_frame, handle_bbox):
        """
        Track handle using optical flow on good features with ROI update and robust outlier rejection
        Returns: (projected_x, projected_y) or None
        """
        if current_frame is None:
            return None
        
        try:
            gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # Initialize tracking points when handle is detected by YOLO
            if handle_bbox is not None:
                xmin, ymin, xmax, ymax = handle_bbox
                
                # Extract handle region
                handle_region = gray[int(ymin):int(ymax), int(xmin):int(xmax)]
                
                if handle_region.size > 0:
                    # Detect good features to track in handle region
                    corners = cv2.goodFeaturesToTrack(
                        handle_region, 
                        maxCorners=20,
                        qualityLevel=0.01,
                        minDistance=10
                    )
                    
                    if corners is not None:
                        # Convert to full image coordinates
                        self.track_points = corners + np.array([xmin, ymin])
                        self.prev_gray = gray.copy()
                        
                        # Store initial ROI size for reference
                        self.roi_width = xmax - xmin
                        self.roi_height = ymax - ymin
                        
                        # Calculate center of tracking points
                        center = np.mean(self.track_points, axis=0)[0]
                        
                        # Update last_handle_bbox with current position
                        self.last_handle_bbox = (xmin, ymin, xmax, ymax)
                        
                        # Store last known good center for outlier detection
                        self.last_good_center = np.array([center[0], center[1]])
                        
                        # Reset frame counter
                        self.frames_since_reinit = 0
                        
                        return center[0], center[1]
                
                return None
            
            # Update tracking using optical flow (when handle_bbox is None)
            if hasattr(self, 'track_points') and self.track_points is not None:
                new_points, status, error = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray,
                    gray,
                    self.track_points,
                    None,
                    winSize=(21, 21),
                    maxLevel=3
                )
                
                # Keep only good points
                if status is not None:
                    good_new = new_points[status.flatten() == 1]
                    
                    if len(good_new) >= 5:  # Need at least 5 points
                        
                        # **SAFEGUARD 1: Check tracking quality based on optical flow error**
                        good_errors = error[status.flatten() == 1].flatten()
                        median_error = np.median(good_errors)
                        
                        # If tracking error is too high, abort (indicates poor tracking quality)
                        if median_error > 30.0:  # Threshold for acceptable tracking error
                            self.get_logger().warn(
                                f"High tracking error: {median_error:.1f} - rejecting frame",
                                throttle_duration_sec=1.0
                            )
                            self.track_points = None
                            return None
                        
                        # **OUTLIER REJECTION 1: Remove points far from median**
                        median_pos = np.median(good_new, axis=0)
                        distances = np.linalg.norm(good_new - median_pos, axis=1)
                        mad = np.median(distances)
                        
                        # Adaptive threshold based on distance to target
                        # When close (large handle), use tighter threshold
                        base_threshold = 20
                        if hasattr(self, 'distance') and self.distance < 0.5:
                            threshold_multiplier = 2.0  # Tighter when close
                        else:
                            threshold_multiplier = 3.0  # More lenient when far
                        
                        threshold = max(threshold_multiplier * mad, base_threshold)
                        
                        inlier_mask = distances < threshold
                        filtered_points = good_new[inlier_mask]
                        
                        if len(filtered_points) < 5:
                            self.get_logger().warn(
                                f"Too many distance outliers: {len(good_new) - len(filtered_points)}/{len(good_new)}",
                                throttle_duration_sec=1.0
                            )
                            self.track_points = None
                            return None
                        
                        # **OUTLIER REJECTION 2: Spatial constraints relative to box**
                        if self.blackbox_xmin > 0 and self.blackbox_ymin > 0:
                            # Adaptive margins based on distance
                            if hasattr(self, 'distance') and self.distance < 0.5:
                                # Very close - use tighter spatial constraints
                                width_margin = 50
                                height_margin = 150
                            else:
                                # Further away - more lenient
                                width_margin = 30
                                height_margin = 100
                            
                            valid_xmin = self.blackbox_xmin - width_margin
                            valid_xmax = self.blackbox_xmax + width_margin
                            valid_ymin = 0
                            valid_ymax = self.blackbox_ymin + height_margin
                            
                            spatial_mask = (
                                (filtered_points[:, 0] >= valid_xmin) &
                                (filtered_points[:, 0] <= valid_xmax) &
                                (filtered_points[:, 1] >= valid_ymin) &
                                (filtered_points[:, 1] <= valid_ymax)
                            )
                            
                            spatially_filtered = filtered_points[spatial_mask]
                            num_spatial_outliers = len(filtered_points) - len(spatially_filtered)
                            
                            if len(spatially_filtered) < 5:
                                self.get_logger().warn(
                                    f"Too many spatial outliers: {num_spatial_outliers}/{len(filtered_points)}",
                                    throttle_duration_sec=1.0
                                )
                                self.track_points = None
                                return None
                            
                            filtered_points = spatially_filtered
                        
                        # Calculate center of filtered points
                        center = np.mean(filtered_points, axis=0)
                        center_x, center_y = center[0], center[1]
                        
                        # **SAFEGUARD 2: Check for unrealistic velocity (jump detection)**
                        if hasattr(self, 'last_good_center') and self.last_good_center is not None:
                            jump_distance = np.linalg.norm(center - self.last_good_center)
                            
                            # Adaptive max jump based on distance
                            if hasattr(self, 'distance') and self.distance < 0.5:
                                max_jump = 30  # Stricter when close
                            else:
                                max_jump = 50  # More lenient when far
                            
                            if jump_distance > max_jump:
                                self.get_logger().warn(
                                    f"Center jumped {jump_distance:.1f}px (max: {max_jump}px) at dist={self.distance:.2f}m",
                                    throttle_duration_sec=1.0
                                )
                                self.track_points = None
                                return None
                        
                        # **SAFEGUARD 3: Verify center is geometrically valid relative to box**
                        if self.blackbox_ymin > 0:
                            # Adaptive height check
                            if hasattr(self, 'distance') and self.distance < 0.5:
                                allowed_below = 150  # When close, handle might appear lower
                            else:
                                allowed_below = 100
                            
                            if center_y > self.blackbox_ymin + allowed_below:
                                self.get_logger().warn(
                                    f"Handle center too low: y={center_y:.1f} > box_top={self.blackbox_ymin:.1f}",
                                    throttle_duration_sec=1.0
                                )
                                self.track_points = None
                                return None
                        
                        # **SAFEGUARD 4: Check percentage of points retained**
                        retention_rate = len(filtered_points) / len(good_new)
                        if retention_rate < 0.5:  # Lost more than 50% of points
                            self.get_logger().warn(
                                f"Low point retention: {retention_rate*100:.1f}% - tracking may be unreliable",
                                throttle_duration_sec=1.0
                            )
                            # Don't abort, but flag as suspicious
                        
                        # Update tracking points with filtered inliers
                        self.track_points = filtered_points.reshape(-1, 1, 2)
                        self.prev_gray = gray.copy()
                        
                        # Store this as last good center
                        self.last_good_center = np.array([center_x, center_y])
                        
                        # **Update ROI based on tracked center**
                        half_w = self.roi_width / 2
                        half_h = self.roi_height / 2
                        
                        new_xmin = center_x - half_w
                        new_ymin = center_y - half_h
                        new_xmax = center_x + half_w
                        new_ymax = center_y + half_h
                        
                        # Clamp ROI to image bounds
                        h, w = gray.shape
                        new_xmin = max(0, new_xmin)
                        new_ymin = max(0, new_ymin)
                        new_xmax = min(w, new_xmax)
                        new_ymax = min(h, new_ymax)
                        
                        self.last_handle_bbox = (new_xmin, new_ymin, new_xmax, new_ymax)
                        
                        # **CRITICAL: More aggressive feature re-initialization when close**
                        if not hasattr(self, 'frames_since_reinit'):
                            self.frames_since_reinit = 0
                        
                        self.frames_since_reinit += 1
                        
                        # Adaptive re-initialization interval
                        if hasattr(self, 'distance'):
                            if self.distance < 0.5:
                                reinit_interval = 15  # Re-init more often when close
                            elif self.distance < 1.0:
                                reinit_interval = 25
                            else:
                                reinit_interval = 40  # Less often when far
                        else:
                            reinit_interval = 30
                        
                        if self.frames_since_reinit > reinit_interval:
                            if new_xmax > new_xmin and new_ymax > new_ymin:
                                handle_region = gray[int(new_ymin):int(new_ymax), 
                                                    int(new_xmin):int(new_xmax)]
                                
                                if handle_region.size > 0:
                                    corners = cv2.goodFeaturesToTrack(
                                        handle_region,
                                        maxCorners=20,
                                        qualityLevel=0.01,
                                        minDistance=10
                                    )
                                    
                                    if corners is not None:
                                        self.track_points = corners + np.array([new_xmin, new_ymin])
                                        self.frames_since_reinit = 0
                                        self.get_logger().info(
                                            f"Re-initialized features at dist={self.distance:.2f}m",
                                            throttle_duration_sec=2.0
                                        )
                        
                        num_total_outliers = len(good_new) - len(filtered_points)
                        self.get_logger().debug(
                            f"OF: {len(filtered_points)} pts at ({center_x:.1f}, {center_y:.1f}), "
                            f"err={median_error:.1f}, outliers={num_total_outliers}, dist={self.distance:.2f}m",
                            throttle_duration_sec=1.0
                        )
                        
                        return center_x, center_y
                    else:
                        self.get_logger().debug(
                            "Not enough tracking points",
                            throttle_duration_sec=1.0
                        )
                        self.track_points = None
                        return None
            
            return None
                
        except Exception as e:
            self.get_logger().error(f"Error in optical flow tracking: {e}")
            self.track_points = None
            return None

    
    
    

  

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()