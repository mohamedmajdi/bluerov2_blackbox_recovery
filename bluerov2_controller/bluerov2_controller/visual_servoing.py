#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import Int16MultiArray, UInt16MultiArray, String
from sensor_msgs.msg import Image
from bluerov2_interface.msg import Detection
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

# --- Helper: Map to PWM with Deadzone Compensation ---
def map_to_pwm(value: float, deadzone_offset: int = 25) -> int:
    """
    Map -1..1 to 1100..1900.
    Adds a 'deadzone_offset' jump to ensure thrusters actually spin 
    for small non-zero values (T200 usually needs ~25us offset).
    """
    if abs(value) < 0.001:
        return 1500
    
    # Base PWM calculation
    pw_offset = value * 400.0
    
    # Add deadzone jump
    if pw_offset > 0:
        pw_offset += deadzone_offset
    elif pw_offset < 0:
        pw_offset -= deadzone_offset
        
    pw = 1500.0 + pw_offset
    pw = max(1100.0, min(1900.0, pw))
    return int(pw)

class VisionController(LifecycleNode):
    def __init__(self):
        super().__init__('vision_controller')

        self.attachment_start_time = None
        self.fast_surge = False
        
        # Integral State for Surge
        self.z_integral = 0.0
        self.last_time = time.time()

        # --- default parameters ---
        self.declare_parameter('calib_file', 'camera_params.npz',
                               ParameterDescriptor(description='npz with camera calibration', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('desired_point_x', -1.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_y', 250.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_z', 0.8, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('handel_offset',0.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('fast_surge', False,
            ParameterDescriptor(description='Enable fast surge when close to the handle',
                            type=ParameterType.PARAMETER_BOOL))

        for name, default in (
            ('gain_surge', 1.0),
            ('gain_surge_integral', 0.2), # Integral gain for surge
            ('integral_limit', 0.5),      # Limit for integral term (anti-windup)
            ('gain_sway', 0.04),
            ('gain_heave', 5.0),
            ('gain_yaw', 25.0),
            ('v_linear_max', 0.15),
            ('v_angular_max', 0.1),
            ('floatability', 0.0),
            ('invert_surge', False),
            ('invert_sway', True),
            ('invert_heave', True),
            ('invert_yaw', False),
            ('enable_visual_servoing', True),
            ('track_handle', False),
            ('enable_of_tracking', True),
             ('image_topic', 'camera/image'),
            ('detections_topic', 'detections'),
            ('pwm_topic', 'controller/pwm_servoing'),
            ('control_rate', 20.0),
            ('rotating_yaw_factor',0.4),
            ('rotating_sway_factor',1.0),
            ('aligned_threshold', 0.3),
            ('aligned_distance', 1.0),
            ('attachment_speed',0.5),
        ):
            tp = ParameterType.PARAMETER_BOOL if isinstance(default, bool) else \
                 ParameterType.PARAMETER_DOUBLE if isinstance(default, float) else \
                 ParameterType.PARAMETER_STRING
            self.declare_parameter(name, default, ParameterDescriptor(type=tp))

        # param callback
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # runtime state (initialized later)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.legacy_cam = {} 

        self.desired_point = None
        self.gains = np.zeros(4)
        self.gain_surge_integral = 0.0
        self.integral_limit = 0.5
        
        self.v_linear_max = 0.3
        self.v_angular_max = 0.5
        self.floatability = -0.18
        self.invert_surge = False
        self.invert_sway = False
        self.invert_heave = True
        self.invert_yaw = False
        self.enable_visual_servoing = True
        self.track_handle = False
        self.enable_of_tracking = False
        self.aligned_threshold=0.05
        # bounding boxes / tracking
        self.blackbox_xmin = self.blackbox_xmax = self.blackbox_ymin = self.blackbox_ymax = 0
        self.blackbox_xcenter = self.blackbox_ycenter = 0
        self.handle_xmin = self.handle_xmax = self.handle_xcenter = 0
        self.handle_ymin = self.handle_ymax = self.handle_ycenter = 0

        # OF tracker
        self.track_points = None
        self.prev_gray = None
        self.roi_w = self.roi_h = 0
        self.frames_since_reinit = 0
        self.last_handle_bbox = None
        self.last_good_center = None
        self.handle_lost_frames = 0

        # visualization & outputs
        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)
        self.s_error = np.zeros(2)
        self.z_error = 0.0
        self.distance = 1.0

        # publishers/subscribers (created in configure/activate)
        self.pub_pwm_vs = None           
        self.sub_image = None
        self.sub_detect = None
        self.sub_approaching = None      # Subscriber for approaching trigger
        self._timer = None
        self.rate = 20.0
        self.tracking_mode = None
        self.known_w = 0.14

        self.aligned = False

        self.rotating_yaw_factor = 1.0
        self.rotating_sway_factor = 1.0
        self.aligned_distance = 2.0
        self.handel_offset = 0.0
        self.attachment_speed = 0.5
        self.get_logger().info("VisionController constructed")

    # ---------- lifecycle callbacks ----------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring VisionController...")

        # load calibration (flexible)
        calib = self.get_parameter('calib_file').value
        try:
            data = np.load(calib)
            if 'camera_matrix' in data.files and 'dist_coeffs' in data.files:
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                self.get_logger().info(f"Loaded camera_matrix/dist_coeffs from {calib}")
            else:
                for k in ('u0', 'v0', 'lx', 'ly', 'kud', 'kdu'):
                    if k in data.files:
                        self.legacy_cam[k] = float(data[k])
                if self.legacy_cam:
                    self.get_logger().info(f"Loaded legacy camera params from {calib}: {list(self.legacy_cam.keys())}")
                else:
                    self.get_logger().warn(f"No expected keys in {calib}, will use defaults")
        except Exception as e:
            self.get_logger().warn(f"Could not load calib '{calib}': {e} — using defaults")

        # update parameters into local state
        self._update_params_from_ros()

        pwm_topic = self.get_parameter('pwm_topic').value
        self.pub_pwm_vs = self.create_publisher(UInt16MultiArray, pwm_topic, 10)

        self.get_logger().info(f"Configured topics: image={self.get_parameter('image_topic').value}, detections={self.get_parameter('detections_topic').value}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating VisionController...")
        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)
        
        # Reset integral on activation
        self.z_integral = 0.0
        self.last_time = time.time()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.sub_image = self.create_subscription(Image,self.get_parameter('image_topic').value, self.image_callback, qos_profile=qos)
        self.sub_detect = self.create_subscription(Detection, self.get_parameter('detections_topic').value, self.color_video_tracking_callback, 10)
        # Subscribe to approaching topic
        self.sub_approaching = self.create_subscription(String, "visual_servoing/approaching", self.approaching_callback, 10)

        self.rate = float(self.get_parameter('control_rate').value)
        self._timer = self.create_timer(1.0 / self.rate, self._control_loop)

        self.get_logger().info("Activated VisionController")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)
        self.blackbox_xmin = self.blackbox_xmax = self.blackbox_ymin = self.blackbox_ymax = 0
        self.blackbox_xcenter = self.blackbox_ycenter = 0
        self.handle_xmin = self.handle_xmax = self.handle_xcenter = 0
        self.handle_ymin = self.handle_ymax = self.handle_ycenter = 0
        self.aligned = False
        
        # Reset State
        self.track_points = None
        self.prev_gray = None
        self.roi_w = self.roi_h = 0
        self.frames_since_reinit = 0
        self.last_handle_bbox = None
        self.last_good_center = None
        self.handle_lost_frames = 0
        self.z_integral = 0.0 # Reset Integral
        
        self.get_logger().info("Deactivating VisionController...")
        if self._timer:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if self.sub_image:
            self.destroy_subscription(self.sub_image); self.sub_image = None
        if self.sub_detect:
            self.destroy_subscription(self.sub_detect); self.sub_detect = None
        if self.sub_approaching:
            self.destroy_subscription(self.sub_approaching); self.sub_approaching = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up VisionController...")
        if self.pub_pwm_vs:
            self.destroy_publisher(self.pub_pwm_vs); self.pub_pwm_vs = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down VisionController...")
        return TransitionCallbackReturn.SUCCESS
    
    # ---------- Approaching Callback ----------
    def approaching_callback(self, msg: String):
        """Handle approaching signal from joystick"""
        if msg.data == "allowed":
            if not self.fast_surge:
                self.fast_surge = True
                self.get_logger().info("Approaching allowed: Fast surge ENABLED")
        elif msg.data == "denied":
            if self.fast_surge:
                self.fast_surge = False
                self.get_logger().info("Approaching denied: Fast surge DISABLED")

    # ---------- parameters ----------
    def _on_set_parameters(self, params):
        for p in params:
            try:
                name, val = p.name, p.value

                if name.startswith('desired_point_'):
                    if self.desired_point is None:
                        x = self.get_parameter('desired_point_x').value
                        y = self.get_parameter('desired_point_y').value
                        z = self.get_parameter('desired_point_z').value
                        self.desired_point = np.array([x, y, z])
                    if name == 'desired_point_x': self.desired_point[0] = val
                    elif name == 'desired_point_y': self.desired_point[1] = val
                    elif name == 'desired_point_z': self.desired_point[2] = val

                # Gains
                elif name == 'gain_surge': self.gains[2] = val
                elif name == 'gain_surge_integral': self.gain_surge_integral = val
                elif name == 'integral_limit': self.integral_limit = val
                elif name == 'gain_sway': self.gains[0] = val
                elif name == 'gain_heave': self.gains[1] = val
                elif name == 'gain_yaw': self.gains[3] = val

                elif name == 'floatability': self.floatability = val
                elif name == 'enable_visual_servoing': self.enable_visual_servoing = val
                elif name == 'track_handle': self.track_handle = val
                elif name == 'fast_surge': self.fast_surge = p.value
                elif name == 'enable_of_tracking': self.enable_of_tracking = val
                elif name == 'v_linear_max': self.v_linear_max = val
                elif name == 'v_angular_max': self.v_angular_max = val
                elif name == 'invert_surge': self.invert_surge = val
                elif name == 'invert_sway': self.invert_sway = val
                elif name == 'invert_heave': self.invert_heave = val
                elif name == 'invert_yaw': self.invert_yaw = val
                elif name == 'rotating_yaw_factor': self.rotating_yaw_factor = val 
                elif name == 'rotating_sway_factor': self.rotating_sway_factor = val 
                elif name == 'aligned_threshold': self.aligned_threshold = val 
                elif name == 'aligned_distance': self.aligned_distance = val 
                elif name == 'handel_offset': self.handel_offset = val
                elif name == 'attachment_speed': self.attachment_speed = val
                elif name == 'calib_file':
                    try:
                        data = np.load(val)
                        if 'camera_matrix' in data.files and 'dist_coeffs' in data.files:
                            self.camera_matrix = data['camera_matrix']
                            self.dist_coeffs = data['dist_coeffs']
                            self.legacy_cam = {}
                        else:
                            self.legacy_cam = {k: float(data[k]) for k in data.files if k in ('u0','v0','lx','ly','kud','kdu')}
                    except Exception as e:
                        self.get_logger().error(f"Failed to reload calib '{val}': {e}")

            except Exception as e:
                self.get_logger().error(f"Error applying parameter {p.name}: {e}")

        return SetParametersResult(successful=True)


    def _update_params_from_ros(self):
        """Centralized parameter pulling."""
        dp_x = self.get_parameter('desired_point_x').value
        dp_y = self.get_parameter('desired_point_y').value
        dp_z = self.get_parameter('desired_point_z').value
        if self.desired_point is None:
            self.desired_point = np.array([dp_x, dp_y, dp_z])
        else:
            if dp_x >= 0: self.desired_point[0] = dp_x
            if dp_y >= 0: self.desired_point[1] = dp_y
            self.desired_point[2] = dp_z

        self.gains = np.array([
            self.get_parameter('gain_sway').value,
            self.get_parameter('gain_heave').value,
            self.get_parameter('gain_surge').value,
            self.get_parameter('gain_yaw').value])
        
        self.gain_surge_integral = self.get_parameter('gain_surge_integral').value
        self.integral_limit = self.get_parameter('integral_limit').value

        self.v_linear_max = self.get_parameter('v_linear_max').value
        self.v_angular_max = self.get_parameter('v_angular_max').value
        self.floatability = self.get_parameter('floatability').value
        self.invert_surge = self.get_parameter('invert_surge').value
        self.invert_sway = self.get_parameter('invert_sway').value
        self.invert_heave = self.get_parameter('invert_heave').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        self.enable_visual_servoing = self.get_parameter('enable_visual_servoing').value
        self.track_handle = self.get_parameter('track_handle').value
        self.enable_of_tracking = self.get_parameter('enable_of_tracking').value
        self.fast_surge = self.get_parameter('fast_surge').value
        self.rotating_sway_factor = self.get_parameter('rotating_sway_factor').value
        self.rotating_yaw_factor = self.get_parameter('rotating_yaw_factor').value
        self.aligned_threshold = self.get_parameter('aligned_threshold').value
        self.aligned_distance = self.get_parameter('aligned_distance').value
        self.handel_offset = self.get_parameter('handel_offset').value
        self.attachment_speed = self.get_parameter('attachment_speed').value
    # ---------- control loop ----------
    def _control_loop(self):
        """Publishes PWM messages based on current Camera_pwm state."""
        if not self.enable_visual_servoing:
            for k in self.Camera_pwm: self.Camera_pwm[k] = 1500

        if self.pub_pwm_vs:
            msg = UInt16MultiArray()
            arr = [int(self.Camera_pwm['surge']), int(self.Camera_pwm['sway']),
                   int(self.Camera_pwm['heave']),  int(self.Camera_pwm['roll']),
                   int(self.Camera_pwm['pitch']), int(self.Camera_pwm['yaw'])
                   ]
            msg.data = arr
            self.pub_pwm_vs.publish(msg)

    # ---------- image & detections (streamlined) ----------
    def image_callback(self, msg: Image):
        try:
            self.image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = self.image_np.shape[:2]
            
            if not getattr(self, 'image_center_set', False):
                dp_x = self.get_parameter('desired_point_x').value
                dp_y = self.get_parameter('desired_point_y').value
                dp_z = self.get_parameter('desired_point_z').value
                if dp_x < 0 or dp_x >= w: dp_x = w // 2
                if dp_y < 0 or dp_y >= h: dp_y = h // 2
                self.desired_point = np.array([dp_x, dp_y, dp_z])
                self.image_center_set = True

            bw = max(0, self.blackbox_xmax - self.blackbox_xmin)
            bh = max(0, self.blackbox_ymax - self.blackbox_ymin)
            box_px = min(bw, bh) if bw and bh else bw
            if box_px > 0:
                if self.camera_matrix is not None:
                    f = (self.camera_matrix[0,0] + self.camera_matrix[1,1]) / 2.0
                else:
                    f = self.legacy_cam.get('lx', 455.0)
                self.distance = (self.known_w * f) / float(box_px)
            
            self.draw_visualization()

        except Exception as e:
            self.get_logger().error(f"image_callback error: {e}")

    def color_video_tracking_callback(self, msg):
        """
        Visual servoing callback with PI control, deadzone comp, and fast surge.
        """
        # Calculate DT for integration
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        surge_sign = -1.0 if self.invert_surge else 1.0
        sway_sign = -1.0 if self.invert_sway else 1.0
        heave_sign = -1.0 if self.invert_heave else 1.0
        yaw_sign = -1.0 if self.invert_yaw else 1.0

        if not self.enable_visual_servoing:
            self.Camera_pwm.update({k:1500 for k in self.Camera_pwm})
            self.tracking_mode = "DISABLED"
            self.z_integral = 0.0 # Reset
            return
        
        # --- Fast Surge Logic ---
        if self.fast_surge:
            if self.attachment_start_time is None:
                self.attachment_start_time = time.time()           
                self.gains[2] = 1.0 
                self.attachment_duration = 5.0
                # self.attachment_speed = 0.5 

            elapsed = time.time() - self.attachment_start_time

            if elapsed < self.attachment_duration:
                self.Camera_pwm['surge'] = map_to_pwm(self.attachment_speed)
                self.Camera_pwm['sway'] = 1500
                self.Camera_pwm['heave'] = 1500
                self.Camera_pwm['yaw'] = 1500
                self.tracking_mode = "ATTACHING"
                return
            else:
                self.attachment_start_time = None
                self.fast_surge = False 
                self.gains[2] = self.get_parameter('gain_surge').value 
        else:
            if self.attachment_start_time is not None:
                self.attachment_start_time = None
                self.gains[2] = self.get_parameter('gain_surge').value
                self.get_logger().warn("Fast surge interrupted")

        # --- Detection Data ---
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

        handle_detected = (self.handle_xcenter > 0 and self.handle_xmax > 0)
        
        width = np.abs(self.blackbox_xmax - self.blackbox_xmin)
        height = np.abs(self.blackbox_ymax - self.blackbox_ymin)
        
        # --- LOSS OF TRACKING RESET ---
        if width == 0 or height == 0:
            self.get_logger().info("Box lost - Resetting Integral")
            self.z_integral = 0.0 # Reset
            return

        ratio = width / height
        ref_width = ratio * self.known_w 
        f = (self.camera_matrix[0,0] + self.camera_matrix[1,1]) / 2.0
        bw = max(0, self.blackbox_xmax - self.blackbox_xmin)
        bh = max(0, self.blackbox_ymax - self.blackbox_ymin)
        box_px = min(bw, bh) if bw and bh else bw
        z = ref_width * f / float(box_px)
        self.distance = z
        Z = max(self.distance, 0.1)

        # --- Alignment Rotation ---
        if not self.aligned and np.abs(width/height - 16/14) > self.aligned_threshold:
            self.get_logger().info(f"Rotating to align. Ratio:{ratio:.2f}")
            if z > self.aligned_distance:
                self.Camera_pwm['surge'] = 1550
                self.Camera_pwm['sway']  = 1500
                self.Camera_pwm['heave'] = 1500
                self.Camera_pwm['yaw']   = 1500
            else:
                rotate_speed = 0.2
                self.Camera_pwm['surge'] = 1500
                self.Camera_pwm['sway']  = map_to_pwm(self.rotating_sway_factor * rotate_speed)
                self.Camera_pwm['heave'] = map_to_pwm(self.floatability)
                self.Camera_pwm['yaw']   = map_to_pwm(yaw_sign* self.rotating_yaw_factor * rotate_speed)
            
            # Reset integral during rotation as we aren't actively surging to target
            self.z_integral = 0.0 
            return
        else:
            self.aligned = True
        
         # ============ MULTI-LEVEL TRACKING LOGIC ============
        prev_mode = self.tracking_mode
        
        if self.track_handle and handle_detected:
            self.handle_lost_frames = 0
            self.last_handle_bbox = (self.handle_xmin, self.handle_ymin, 
                                    self.handle_xmax, self.handle_ymax)
            x_handle, y_handle = self._px2norm([self.handle_xcenter, self.handle_ycenter])
            self.last_known_handle_pos_cam = np.array([x_handle * Z, y_handle * Z, Z])
            self.last_known_handle_pos_pixels = np.array([self.handle_xcenter, self.handle_ycenter])
            x, y = x_handle, y_handle
            self.tracking_mode = "YOLO-HANDLE"
            
        elif self.track_handle and not handle_detected:
            self.handle_lost_frames += 1
            if self.handle_lost_frames > 2 and hasattr(self, 'image_np') and self.image_np is not None:
                projected_pos = self.track_handle_features(self.image_np, self.last_handle_bbox)
                
                if projected_pos is not None:
                    x, y = self._px2norm(projected_pos)
                    self.tracking_mode = "FEATURE-HANDLE"
                else:
                    x, y = self._px2norm([self.blackbox_xcenter, self.blackbox_ymax])
                    self.tracking_mode = "BOX-FALLBACK"
            else:
                x, y = self._px2norm([self.blackbox_xcenter, self.blackbox_ymax])
                self.tracking_mode = "BOX-EARLY"
        else:
            x, y = self._px2norm([self.blackbox_xcenter, self.blackbox_ymax])
            self.tracking_mode = "BOX-BOTTOM"
            self.handle_lost_frames = 0
            
        # Reset Integral if tracking mode drastically changes (e.g. handle lost) to avoid sudden jumps
        if prev_mode != self.tracking_mode and "HANDLE" in str(prev_mode) and "BOX" in str(self.tracking_mode):
             self.z_integral = 0.0

        # ============ VISUAL SERVOING CONTROL ============
        xd, yd = self._px2norm(self.desired_point[:2])
        zd = self.desired_point[2]
        
        self.s_error = np.array([x - xd, y - yd])
        self.z_error = Z - zd
        
        # --- Integral Update ---
        # Accumulate error for surge (Z-axis)
        if abs(self.z_error) < 1.0: # Only integrate when reasonably close
             self.z_integral += self.z_error * dt
             # Anti-windup
             self.z_integral = np.clip(self.z_integral, -self.integral_limit, self.integral_limit)
        else:
             self.z_integral = 0.0
        
        # Interaction matrix
        fx = 1.0; fy = 1.0
        L_full = np.array([
            [-fx/Z, 0, fx*x/Z, fx*x*y, -fx*(1+x**2), fx*y],
            [0, -fy/Z, fy*y/Z, fy*(1+y**2), -fy*x*y, -fy*x]
        ])
        L_reduced = L_full[:, [0, 1, 2, 5]]
        L_depth = np.array([[0, 0, -1, 0]])
        L = np.vstack([L_reduced, L_depth])
        
        error_combined = np.array([self.s_error[0], self.s_error[1], self.z_error])
        L_inv = np.linalg.pinv(L)
        v_cam_4d = -self.gains * (L_inv @ error_combined)
        
        # Apply Integral term to Surge
        # Subtract because v = -K * error
        v_cam_4d[2] -= self.gain_surge_integral * self.z_integral
        
        H = self.camera_to_rov_transform()
        v_rov_4d = self.transform_velocity_4dof(v_cam_4d, H)
        
        v_rov_4d[0:3] = np.clip(v_rov_4d[0:3], -self.v_linear_max, self.v_linear_max)
        v_rov_4d[3] = np.clip(v_rov_4d[3], -self.v_angular_max, self.v_angular_max)

        if handle_detected:
            sway_speed = (self.blackbox_xcenter - (self.handle_xcenter - self.handel_offset)) * self.gains[0]
        else:
            sway_speed = 0.0
            
        self.Camera_pwm['surge'] = map_to_pwm(surge_sign * v_rov_4d[0])
        self.Camera_pwm['sway']  = map_to_pwm(sway_sign * sway_speed)
        self.Camera_pwm['heave'] = map_to_pwm(heave_sign * v_rov_4d[2] + self.floatability)
        self.Camera_pwm['yaw']   = map_to_pwm(yaw_sign * v_rov_4d[3])
        self.Camera_pwm['pitch'] = 1500
        self.Camera_pwm['roll']  = 1500

    # ---------- small helpers ----------
    def _px2norm(self, pt):
        try:
            if self.camera_matrix is not None:
                cx = self.camera_matrix[0, 2]; cy = self.camera_matrix[1, 2]
                fx = self.camera_matrix[0, 0]; fy = self.camera_matrix[1, 1]
                return (float(pt[0]) - cx) / fx, (float(pt[1]) - cy) / fy
            elif self.legacy_cam:
                u0 = self.legacy_cam.get('u0', 480.0); v0 = self.legacy_cam.get('v0', 270.0)
                lx = self.legacy_cam.get('lx', 455.0); ly = self.legacy_cam.get('ly', 455.0)
                return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly
            else:
                return (float(pt[0]) - 480.0) / 455.0, (float(pt[1]) - 270.0) / 455.0
        except Exception as e:
            self.get_logger().warn(f"_px2norm failed: {e}")
            return 0.0, 0.0

    def camera_to_rov_transform(self):
        R = np.array([[0,0,1],[1,0,0],[0,1,0]])
        t = np.array([0.2, 0.0, 0.0])
        H = np.eye(4); H[0:3,0:3] = R; H[0:3,3] = t
        return H

    def transform_velocity_4dof(self, v_cam, H):
        R = H[0:3,0:3]
        v_lin_rov = R @ v_cam[0:3]
        return np.array([v_lin_rov[0], v_lin_rov[1], v_lin_rov[2], v_cam[3]])
    
    def overlay_points(self, image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
        cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
        position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

    def draw_visualization(self):
        try:
            if not hasattr(self, 'image_np') or self.image_np is None:
                return
            
            desired_x = int(self.desired_point[0])
            desired_y = int(self.desired_point[1])
            self.overlay_points(self.image_np, [desired_x, desired_y], 255, 0, 0, 'desired', scale=0.7)
            
            if self.blackbox_xmin > 0:
                cv2.rectangle(self.image_np, (int(self.blackbox_xmin), int(self.blackbox_ymin)), 
                              (int(self.blackbox_xmax), int(self.blackbox_ymax)), (0, 255, 255), 2)
                
                info_y = 30; line_height = 30
                cv2.putText(self.image_np, f"Dist: {self.distance:.2f} m", (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(self.image_np, f"Mode: {self.tracking_mode}", (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Show Integral Accumulation
                cv2.putText(self.image_np, f"Z-Integral: {self.z_integral:.3f}", (10, info_y + 7*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("Visual Servoing", self.image_np)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")

    def track_handle_features(self, current_frame, handle_bbox):
        if current_frame is None or handle_bbox is None: return None
        try:
            gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            xmin, ymin, xmax, ymax = handle_bbox
            if xmin >= xmax or ymin >= ymax: return None
            region = gray[int(ymin):int(ymax), int(xmin):int(xmax)]
            if region.size == 0: return None
            corners = cv2.goodFeaturesToTrack(region, maxCorners=20, qualityLevel=0.01, minDistance=8)
            if corners is None: return None
            pts = corners + np.array([xmin, ymin])
            self.track_points = pts; self.prev_gray = gray.copy()
            center = np.mean(pts, axis=0)[0]
            return (center[0], center[1])
        except Exception:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VisionController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()