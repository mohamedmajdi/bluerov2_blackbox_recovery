#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import Int16MultiArray, UInt16MultiArray, String,Float64
from sensor_msgs.msg import Image
from bluerov2_interface.msg import Detection
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

def map_to_pwm(value: float) -> int:
    """Map -1..1 to 1100..1900 and clamp to integers."""
    pw = value * 400.0 + 1500.0
    pw = max(1100.0, min(1900.0, pw))
    return int(pw)

class VisionController(LifecycleNode):
    def __init__(self):
        super().__init__('vision_controller')

        self.attachment_start_time = None
        self.fast_surge = False
        
        self.z_integral = 0.0
        self.last_time = time.time()

        self.declare_parameter('calib_file', 'camera_params.npz',
                               ParameterDescriptor(description='npz with camera calibration', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('desired_point_x', -1.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_y', 350.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_z', 0.35, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('handel_offset',-7.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('fast_surge', False,
            ParameterDescriptor(description='Enable fast surge when close to the handle',
                            type=ParameterType.PARAMETER_BOOL))

        for name, default in (
            ('gain_surge', 1.0),
            ('gain_surge_integral', 0.0),
            ('integral_limit', 0.5),
            ('gain_sway', 0.03),
            ('gain_heave', 6.0),
            ('gain_yaw', 0.001),
            ('turbo_mode', False),
            ('v_linear_max', 0.15),
            ('v_surge_max', 0.1),
            ('v_sway_max', 0.1),
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
        ):
            tp = ParameterType.PARAMETER_BOOL if isinstance(default, bool) else \
                 ParameterType.PARAMETER_DOUBLE if isinstance(default, float) else \
                 ParameterType.PARAMETER_STRING
            self.declare_parameter(name, default, ParameterDescriptor(type=tp))

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.legacy_cam = {} 

        self.desired_point = None
        self.gains = np.zeros(4)
        self.gain_surge_integral = 0.0
        self.integral_limit = 0.5
        
        self.v_linear_max = 0.15
        self.v_surge_max = 0.08
        self.v_sway_max = 0.15
        
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
        self.blackbox_xmin = self.blackbox_xmax = self.blackbox_ymin = self.blackbox_ymax = 0
        self.blackbox_xcenter = self.blackbox_ycenter = 0
        self.handle_xmin = self.handle_xmax = self.handle_xcenter = 0
        self.handle_ymin = self.handle_ymax = self.handle_ycenter = 0
        self.track_points = None
        self.prev_gray = None
        self.roi_w = self.roi_h = 0
        self.frames_since_reinit = 0
        self.last_handle_bbox = None
        self.last_good_center = None
        self.handle_lost_frames = 0
        
        # --- Optical Flow Variables for Box ---
        self.prev_gray_box = None
        self.box_p0 = None
        self.last_box_dims = (0.0, 0.0) # width, height
        self.box_lost_frames = 0
        self.internal_box_center = (0.0, 0.0) # To keep track during the 2-frame gap
        # --------------------------------------

        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)
        self.s_error = np.zeros(2)
        self.z_error = 0.0
        self.distance = 1.0

        self.pub_pwm_vs = None           
        self.sub_image = None
        self.sub_detect = None
        self.sub_approaching = None 
        self.depth_sub = None
        self._timer = None
        self.rate = 20.0
        self.tracking_mode = None
        self.known_w = 0.14

        self.aligned = False
        self.box_or = ""
        self.handle_or = ""

        self.rotating_yaw_factor = 1.0
        self.rotating_sway_factor = 1.0
        self.aligned_distance = 2.0
        self.handel_offset = 0.0
        self.depth =0.0
        self.turbo_mode = False
        self.ready = False
        self.pub_servo_image = None
        self.get_logger().info("VisionController constructed")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring VisionController...")

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

        self._update_params_from_ros()

        pwm_topic = self.get_parameter('pwm_topic').value
        self.pub_pwm_vs = self.create_publisher(UInt16MultiArray, pwm_topic, 10)
        self.pub_servo_image = self.create_publisher(Image, 'visual_servo_image', 10)

        self.get_logger().info(f"Configured topics: image={self.get_parameter('image_topic').value}, detections={self.get_parameter('detections_topic').value}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating VisionController...")
        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)
        
        self.z_integral = 0.0
        self.last_time = time.time()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos2 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.sub_image = self.create_subscription(Image,self.get_parameter('image_topic').value, self.image_callback, qos_profile=qos)
        self.sub_detect = self.create_subscription(Detection, self.get_parameter('detections_topic').value, self.color_video_tracking_callback, 10)
        self.depth_sub = self.create_subscription(
            Float64,
            "global_position/rel_alt",
            self.rel_alt_callback,
            qos_profile=qos2
        )
        self.sub_approaching = self.create_subscription(String, "visual_servoing/approaching", self.approaching_callback, 10)

        self.sub_box_orientation = self.create_subscription(
            String,
            "blackbox_orientation",
            self.box_orientation_callback,
            10
        )

        self.sub_handle_orientation = self.create_subscription(
            String,
            "handle_orientation",
            self.handle_orientation_callback,
            10
        )

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
        self.track_points = None
        self.prev_gray = None
        self.roi_w = self.roi_h = 0
        self.frames_since_reinit = 0
        self.last_handle_bbox = None
        self.last_good_center = None
        self.handle_lost_frames = 0
        self.z_integral = 0.0
        
        # --- Reset OF Box ---
        self.prev_gray_box = None
        self.box_p0 = None
        self.last_box_dims = (0.0, 0.0)
        self.box_lost_frames = 0
        self.internal_box_center = (0.0, 0.0)
        # --------------------
        
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
        if self.depth_sub:
            self.destroy_subscription(self.depth_sub); self.depth_sub = None

        if self.sub_box_orientation:
            self.destroy_subscription(self.sub_box_orientation); self.sub_box_orientation = None
        if self.sub_handle_orientation:
            self.destroy_subscription(self.sub_handle_orientation); self.sub_handle_orientation = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up VisionController...")
        if self.pub_pwm_vs:
            self.destroy_publisher(self.pub_pwm_vs); self.pub_pwm_vs = None
        if self.pub_servo_image:
            self.destroy_publisher(self.pub_servo_image); self.pub_servo_image = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down VisionController...")
        return TransitionCallbackReturn.SUCCESS

    def rel_alt_callback(self,msg:Float64):
        self.depth = -msg.data
    def approaching_callback(self, msg: String):
        if msg.data == "allowed":
            if not self.fast_surge:
                self.fast_surge = True
                self.get_logger().info("Approaching allowed: Fast surge ENABLED")
        elif msg.data == "denied":
            if self.fast_surge:
                self.fast_surge = False
                self.get_logger().info("Approaching denied: Fast surge DISABLED")

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
                elif name == 'v_surge_max': self.v_surge_max = val
                elif name == 'v_sway_max': self.v_sway_max = val

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
                elif name == 'turbo_mode':
                    self.turbo_mode = val
                    if self.turbo_mode:
                        self.get_logger().info("Turbo mode ENABLED")
                    else:
                        self.get_logger().info("Turbo mode DISABLED")
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
        self.v_surge_max = self.get_parameter('v_surge_max').value
        self.v_sway_max = self.get_parameter('v_sway_max').value
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
        self.turbo_mode = self.get_parameter('turbo_mode').value

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

    def box_orientation_callback(self, msg: String):
        self.box_or = msg.data
    def handle_orientation_callback(self, msg: String):
        self.handle_or = msg.data

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
        Visual servoing callback for BlueROV control.
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Convert to gray for OF
        gray = None
        if hasattr(self, 'image_np') and self.image_np is not None:
            gray = cv2.cvtColor(self.image_np, cv2.COLOR_BGR2GRAY)

        surge_sign = -1.0 if self.invert_surge else 1.0
        sway_sign = -1.0 if self.invert_sway else 1.0
        heave_sign = -1.0 if self.invert_heave else 1.0
        yaw_sign = -1.0 if self.invert_yaw else 1.0

        if not self.enable_visual_servoing:
            self.Camera_pwm['surge'] = 1500
            self.Camera_pwm['sway']  = 1500
            self.Camera_pwm['heave'] = 1500
            self.Camera_pwm['yaw']   = 1500
            self.Camera_pwm['pitch'] = 1500
            self.Camera_pwm['roll']  = 1500
            self.tracking_mode = "DISABLED"
            self.z_integral = 0.0
            return
        
        if self.fast_surge:
            if self.attachment_start_time is None:
                self.attachment_start_time = time.time()           
                self.gains[2] = 1.0 
                self.attachment_duration = 5.0
                self.attachment_speed = 0.2

            elapsed = time.time() - self.attachment_start_time

            if elapsed < self.attachment_duration:
                self.Camera_pwm['surge'] = map_to_pwm(self.attachment_speed)
                self.Camera_pwm['sway'] = 1500
                self.Camera_pwm['heave'] = map_to_pwm(self.floatability)
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
                self.get_logger().warn("Fast surge interrupted, resuming normal tracking")

        # -------------------- Box Tracking Logic --------------------
        yolo_box_detected = (msg.blackbox_xmax - msg.blackbox_xmin) > 0 and (msg.blackbox_ymax - msg.blackbox_ymin) > 0

        # Always update internal OF state if possible, to bridge the gap if YOLO is lost
        of_shift_x = 0.0
        of_shift_y = 0.0
        of_valid = False

        if self.enable_of_tracking and self.prev_gray_box is not None and self.box_p0 is not None and gray is not None:
            try:
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray_box, gray, self.box_p0, None, 
                                                       winSize=(15, 15), maxLevel=2,
                                                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                if p1 is not None:
                    good_new = p1[st==1]
                    good_old = self.box_p0[st==1]
                    if len(good_new) >= 2:
                        of_shift_x = np.mean(good_new[:,0] - good_old[:,0])
                        of_shift_y = np.mean(good_new[:,1] - good_old[:,1])
                        # Update points for next step
                        self.box_p0 = good_new.reshape(-1, 1, 2)
                        of_valid = True
                    else:
                        self.box_p0 = None # Lost flow
            except Exception as e:
                self.get_logger().warn(f"Box OF Error: {e}")
                self.box_p0 = None
        
        # Now decide which data to use
        if yolo_box_detected:
            # --- YOLO DETECTED ---
            self.box_lost_frames = 0
            # FIX: Explicitly set mode to BOTTOM/YOLO so we don't get stuck in FLOW mode
            self.tracking_mode = "BOX-BOTTOM" 
            
            # Trust YOLO completely
            self.blackbox_xmin = msg.blackbox_xmin
            self.blackbox_xmax = msg.blackbox_xmax
            self.blackbox_xcenter = msg.blackbox_xcenter
            self.blackbox_ymin = msg.blackbox_ymin
            self.blackbox_ymax = msg.blackbox_ymax
            self.blackbox_ycenter = msg.blackbox_ycenter

            # Update internal state for backup
            self.last_box_dims = (self.blackbox_xmax - self.blackbox_xmin, 
                                  self.blackbox_ymax - self.blackbox_ymin)
            self.internal_box_center = (self.blackbox_xcenter, self.blackbox_ycenter)
            
            # Re-init features inside YOLO box for future failures
            if gray is not None:
                mask = np.zeros_like(gray)
                try:
                    mask[int(self.blackbox_ymin):int(self.blackbox_ymax), 
                         int(self.blackbox_xmin):int(self.blackbox_xmax)] = 255
                    self.box_p0 = cv2.goodFeaturesToTrack(gray, mask=mask, maxCorners=40, 
                                                          qualityLevel=0.1, minDistance=5, blockSize=5)
                    self.prev_gray_box = gray.copy()
                except Exception as e:
                    self.get_logger().warn(f"Error init box features: {e}")

        else:
            # --- YOLO LOST ---
            self.box_lost_frames += 1
            
            # Apply calculated flow to internal center (if valid) so we don't lose position during the <2 frame gap
            if of_valid:
                cx, cy = self.internal_box_center
                self.internal_box_center = (cx + of_shift_x, cy + of_shift_y)
                # Keep prev_gray updated so OF continues next frame
                self.prev_gray_box = gray.copy()

            if self.box_lost_frames > 2 and self.enable_of_tracking and of_valid:
                # > 2 Frames Lost: ACTIVATE PROJECTED BOX
                w_b, h_b = self.last_box_dims
                icx, icy = self.internal_box_center
                
                # Project the box based on internal center and last known dimensions
                self.blackbox_xcenter = icx
                self.blackbox_ycenter = icy
                self.blackbox_xmin = icx - w_b / 2
                self.blackbox_xmax = icx + w_b / 2
                self.blackbox_ymin = icy - h_b / 2
                self.blackbox_ymax = icy + h_b / 2 # This is the "bottom" point used in control
                
                self.tracking_mode = "BOX-FLOW"
            else:
                # < 2 Frames or Flow failed: Report "Zero" (Lost)
                # This respects the requirement: "only if ... lost two frames ... start project"
                self.blackbox_xmin = 0
                self.blackbox_xmax = 0
                self.blackbox_xcenter = 0
                self.blackbox_ymin = 0
                self.blackbox_ymax = 0
                self.blackbox_ycenter = 0
        
        # -----------------------------------------------------------

        self.handle_xmin = msg.handle_xmin
        self.handle_xmax = msg.handle_xmax
        self.handle_xcenter = msg.handle_xcenter
        self.handle_ymin = msg.handle_ymin
        self.handle_ymax = msg.handle_ymax
        self.handle_ycenter = msg.handle_ycenter

        handle_detected = (self.handle_xcenter > 0 and self.handle_xmax > 0)
        
        width = np.abs(self.blackbox_xmax - self.blackbox_xmin)
        height = np.abs(self.blackbox_ymax - self.blackbox_ymin)

        if width == 0 or height == 0 and (self.box_lost_frames <=2 and not of_valid):
            # If we are here, it means YOLO is lost AND (frames <= 2 OR OF failed/disabled)
            self.get_logger().info("box not detected")
            self.z_integral = 0.0
            return

        ratio = width / height
        ref_width = ratio * self.known_w 
        f = (self.camera_matrix[0,0] + self.camera_matrix[1,1]) / 2.0
        bw = max(0, self.blackbox_xmax - self.blackbox_xmin)
        bh = max(0, self.blackbox_ymax - self.blackbox_ymin)
        box_px = min(bw, bh) if bw and bh else bw
        z = ref_width * f / float(box_px)
        th = self.aligned_threshold
        rotate_speed = 0.2
        self.distance = z
        Z = max(self.distance, 0.1)

        
        prev_mode = self.tracking_mode
        
        # Determine tracking mode
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
            if self.tracking_mode != "BOX-FLOW":
                self.tracking_mode = "BOX-BOTTOM"
            self.handle_lost_frames = 0
        
        if prev_mode != self.tracking_mode and "HANDLE" in str(prev_mode) and "BOX" in str(self.tracking_mode):
             self.z_integral = 0.0

        xd, yd = self._px2norm(self.desired_point[:2])
        zd = self.desired_point[2]
        
        self.s_error = np.array([x - xd, y - yd])
        self.z_error = Z - zd
        
        if abs(self.z_error) < 1.0:
             self.z_integral += self.z_error * dt
             self.z_integral = np.clip(self.z_integral, -self.integral_limit, self.integral_limit)
        else:
             self.z_integral = 0.0

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
        
        L_inv = np.linalg.pinv(L)
        v_cam_4d = -self.gains * (L_inv @ error_combined)
        self.get_logger().info(f"gain: {self.gains}, error: {error_combined}, v_cam_4d: {v_cam_4d}", throttle_duration_sec=1.0)
        
        v_cam_4d[2] -= self.gain_surge_integral * self.z_integral

        H = self.camera_to_rov_transform()
        v_rov_4d = self.transform_velocity_4dof(v_cam_4d, H)
        

        if handle_detected:
            sway_speed = (self.blackbox_xcenter - (self.handle_xcenter - self.handel_offset)) * self.gains[0]
        else:
            sway_speed = 0.0
        yaw_speed = (self.blackbox_xcenter - self.desired_point[0]) * self.gains[3]

        if self.turbo_mode and Z > 1.5:
            v_rov_4d[1:3] = np.clip(v_rov_4d[1:3], -1.65 * self.v_linear_max, 1.65 * self.v_linear_max)
            v_rov_4d[0] = np.clip(v_rov_4d[0], -1.5 * self.v_surge_max, 1.5 * self.v_surge_max)
            v_rov_4d[3] = np.clip(v_rov_4d[3], -1.5 * self.v_angular_max, 1.5 * self.v_angular_max)
            sway_speed = np.clip(sway_speed, -1.5 * self.v_sway_max,1.5 * self.v_sway_max)
        elif self.depth < 3.8:
            v_rov_4d[1:3] = np.clip(v_rov_4d[1:3], -1.35 * self.v_linear_max, 1.35 * self.v_linear_max)
            v_rov_4d[0] = np.clip(v_rov_4d[0], -self.v_surge_max, self.v_surge_max)
            v_rov_4d[3] = np.clip(v_rov_4d[3], -self.v_angular_max, self.v_angular_max)
            sway_speed = np.clip(sway_speed, -self.v_sway_max, self.v_sway_max)
        else:
            v_rov_4d[1:3] = np.clip(v_rov_4d[1:3], -self.v_linear_max, self.v_linear_max)
            v_rov_4d[0] = np.clip(v_rov_4d[0], -self.v_surge_max, self.v_surge_max)
            v_rov_4d[3] = np.clip(v_rov_4d[3], -self.v_angular_max, self.v_angular_max)
            sway_speed = np.clip(sway_speed, -self.v_sway_max, self.v_sway_max)

        
        self.Camera_pwm['surge'] = map_to_pwm(surge_sign * v_rov_4d[0])
        self.Camera_pwm['sway']  = map_to_pwm(sway_sign * sway_speed)
        
        self.Camera_pwm['heave'] = map_to_pwm(heave_sign * v_rov_4d[2] + self.floatability)
        self.Camera_pwm['yaw']   = map_to_pwm(yaw_sign * yaw_speed)
        self.Camera_pwm['pitch'] = 1500
        self.Camera_pwm['roll']  = 1500
        err_heave = int(self.blackbox_ymax) - int(self.desired_point[1])
        err_surge = int(Z) - int(self.desired_point[2])
        err_yaw = int(self.blackbox_xcenter) - int(self.desired_point[0]) 
        err_sway = int(self.blackbox_xcenter) - int((self.handle_xcenter - self.handel_offset))
        if abs(err_heave) < 15 and abs(err_surge) < 0.04 and abs(err_yaw) < 30 and abs(err_sway) < 5:
            self.ready = True
        else:
            self.ready = False

    def _px2norm(self, pt):
        """Pixel to normalized camera coords. Falls back to legacy linear model."""
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
        """Draw detection visualization with bounding boxes and tracking points"""
        try:
            if not hasattr(self, 'image_np') or self.image_np is None:
                return
            
            desired_x = int(self.desired_point[0])
            desired_y = int(self.desired_point[1])
            
            self.overlay_points(self.image_np, [desired_x, desired_y], 255, 0, 0, 
                            'desired point', scale=0.7, offsetx=10, offsety=-10)
            
            if self.blackbox_xmin > 0 and self.blackbox_xmax > 0:
                cv2.rectangle(self.image_np,
                            (int(self.blackbox_xmin), int(self.blackbox_ymin)),
                            (int(self.blackbox_xmax), int(self.blackbox_ymax)),
                            (0, 255, 255), 2)
                
                if "BOX" in self.tracking_mode:
                    detected_x = int(self.blackbox_xcenter)
                    detected_y = int(self.blackbox_ymax)
                    label = 'box bottom center'
                else:
                    detected_x = int(self.blackbox_xcenter)
                    detected_y = int(self.blackbox_ycenter)
                    label = 'box center'
                            
                self.overlay_points(self.image_np, [detected_x, detected_y], 0, 255, 0,
                                label, scale=0.7, offsetx=10, offsety=10)
                
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
                
                vel_text = (f"PWM - Surge: {self.Camera_pwm['surge']}, "
                            f"Sway: {self.Camera_pwm['sway']}, "
                            f"Heave: {self.Camera_pwm['heave']}, "
                            f"Yaw: {self.Camera_pwm['yaw']}")
                cv2.putText(self.image_np, vel_text,
                        (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, vel_text,
                        (10, info_y + 3*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1) 
                
                img_error_text = f"Image Error: ({self.s_error[0]:.4f}, {self.s_error[1]:.4f}), Depth Error: {self.z_error:.4f}"
                cv2.putText(self.image_np, img_error_text,
                        (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, img_error_text,
                        (10, info_y + 4*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                gains_text = f"Gains [S,Sw,H,Y]: [{self.gains[2]:.2f}, {self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[3]:.2f}] | Float: {self.floatability:.3f}"
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                status_text = (f"VS: {'ON' if self.enable_visual_servoing else 'OFF'} | "
                            f"Mode: {self.tracking_mode} | Lost: {self.handle_lost_frames} ")
                cv2.putText(self.image_np, status_text,
                        (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, status_text,
                        (10, info_y + 6*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # ready_text = (f"Ready: {'YES' if self.ready else 'NO'} | ")
                # cv2.putText(self.image_np, ready_text,
                #         (10, info_y + 7*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                # cv2.putText(self.image_np, ready_text,
                #         (10, info_y + 7*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                box_or_text = f"Box Orientation: {self.box_or}"
                cv2.putText(self.image_np, box_or_text,
                        (10, info_y + 7*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, box_or_text,
                        (10, info_y + 7*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                handle_or_text = f"Handle Orientation: {self.handle_or}"
                cv2.putText(self.image_np, handle_or_text,
                        (10, info_y + 8*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, handle_or_text,
                        (10, info_y + 8*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if self.handle_xmin > 0 and self.handle_xmax > 0:
                cv2.rectangle(self.image_np,
                            (int(self.handle_xmin), int(self.handle_ymin)),
                            (int(self.handle_xmax), int(self.handle_ymax)),
                            (255, 0, 255), 2)
                
                handle_x = int(self.handle_xcenter)
                handle_y = int(self.handle_ycenter)
                
                cv2.circle(self.image_np, (handle_x, handle_y), 8, (255, 0, 255), -1)
                cv2.putText(self.image_np, 'YOLO Handle', 
                        (handle_x + 15, handle_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            tracking_indicator_y = 30
            
            if self.tracking_mode == "YOLO-HANDLE":
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (255, 0, 255), -1)
                cv2.putText(self.image_np, "YOLO", (860, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if self.handle_xmin > 0:
                    handle_x = int(self.handle_xcenter)
                    handle_y = int(self.handle_ycenter)
                    cv2.circle(self.image_np, (handle_x, handle_y), 12, (255, 0, 255), 3)
            
            elif self.tracking_mode == "FEATURE-HANDLE":
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (0, 255, 0), -1)
                cv2.putText(self.image_np, "FEAT", (860, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                if self.last_handle_bbox is not None:
                    xmin, ymin, xmax, ymax = self.last_handle_bbox
                    cv2.rectangle(self.image_np,
                                (int(xmin), int(ymin)),
                                (int(xmax), int(ymax)),
                                (0, 255, 0), 2)
                    cv2.putText(self.image_np, 'Last Known ROI', 
                            (int(xmin), int(ymin) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                projected_pos = self.track_handle_features(self.image_np, self.last_handle_bbox)
                if projected_pos is not None:
                    proj_x, proj_y = int(projected_pos[0]), int(projected_pos[1])
                    cv2.circle(self.image_np, (proj_x, proj_y), 12, (0, 255, 0), 3)
                    cv2.circle(self.image_np, (proj_x, proj_y), 5, (0, 255, 0), -1)
                    cv2.putText(self.image_np, 'Tracked', 
                            (proj_x + 15, proj_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.line(self.image_np, 
                            (int(self.desired_point[0]), int(self.desired_point[1])),
                            (proj_x, proj_y),
                            (0, 255, 0), 2)
        
            elif "BOX" in self.tracking_mode:
                if self.tracking_mode == "BOX-FLOW":
                    color = (255, 100, 0) # Blue-ish for Flow
                    txt = "FLOW"
                else:
                    color = (0, 255, 255) # Yellow for YOLO
                    txt = "BOX"

                cv2.rectangle(self.image_np, (850, 10), (950, 50), color, -1)
                cv2.putText(self.image_np, txt, (865, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                if self.blackbox_xmin > 0:
                    box_x = int(self.blackbox_xcenter)
                    box_y = int(self.blackbox_ycenter)
                    cv2.circle(self.image_np, (box_x, box_y), 12, color, 3)
                    
                if self.tracking_mode == "BOX-FLOW" and self.box_p0 is not None:
                    # visualize flow points
                    for p in self.box_p0:
                         x,y = p.ravel()
                         cv2.circle(self.image_np, (int(x),int(y)), 3, (0,0,255), -1)
            
            else:
                cv2.rectangle(self.image_np, (850, 10), (950, 50), (128, 128, 128), -1)
                cv2.putText(self.image_np, "OFF", (865, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)     


            if self.ready:
                cv2.rectangle(self.image_np, (850, 60), (950, 100), (0, 255, 0), -1)
                cv2.putText(self.image_np, "READY", (860, 85), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            else:
                cv2.rectangle(self.image_np, (850, 60), (950, 100), (0, 0, 255), -1)
                cv2.putText(self.image_np, "NOT RDY", (855, 85), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
            cv2.imshow("Visual Servoing", self.image_np)
            cv2.waitKey(1)
            
            if self.pub_servo_image:
                img_msg = self.bridge.cv2_to_imgmsg(self.image_np, encoding='bgr8')
                self.pub_servo_image.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
    def track_handle_features(self, current_frame, handle_bbox):
        if current_frame is None or handle_bbox is None:
            return None
        try:
            gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            xmin, ymin, xmax, ymax = handle_bbox
            if xmin >= xmax or ymin >= ymax:
                return None
            region = gray[int(ymin):int(ymax), int(xmin):int(xmax)]
            if region.size == 0:
                return None
            corners = cv2.goodFeaturesToTrack(region, maxCorners=20, qualityLevel=0.01, minDistance=8)
            if corners is None:
                return None
            pts = corners + np.array([xmin, ymin])
            self.track_points = pts; self.prev_gray = gray.copy()
            self.roi_w, self.roi_h = xmax - xmin, ymax - ymin
            center = np.mean(pts, axis=0)[0]
            self.last_good_center = center
            return (center[0], center[1])
        except Exception as e:
            self.get_logger().error(f"track_handle_features error: {e}")
            self.track_points = None
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