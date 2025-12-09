#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import Int16MultiArray, UInt16MultiArray
from sensor_msgs.msg import Image
from bluerov2_interface.msg import Detection
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

# --- minimal helper: clamp & map to pwm ---
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

        # --- default parameters ---
        self.declare_parameter('calib_file', 'camera_params.npz',
                               ParameterDescriptor(description='npz with camera calibration', type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('desired_point_x', -1.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_y', 400.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('desired_point_z', 0.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))

        self.declare_parameter('fast_surge', False,
            ParameterDescriptor(description='Enable fast surge when close to the handle',
                            type=ParameterType.PARAMETER_BOOL))

        for name, default in (
            ('gain_surge', 0.05),
            ('gain_sway', 0.3),
            ('gain_heave', 3.3),
            ('gain_yaw', 14.2),
            ('v_linear_max', 0.15),
            ('v_angular_max', 0.6),
            ('floatability', -0.125),
            ('invert_surge', False),
            ('invert_sway', False),
            ('invert_heave', True),
            ('invert_yaw', False),
            ('enable_visual_servoing', True),
            ('track_handle', False),
            ('enable_of_tracking', True),
            # ('send_to_bringup', False),
            ('image_topic', 'camera/image'),
            # ('image_topic', '/bluerov/camera/image_color'),
            ('detections_topic', 'detections'),
            ('pwm_topic', 'controller/pwm_servoing'),
            # ('bringup_topic', 'controller/pwm_servoing'),
            ('control_rate', 20.0)
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
        self.legacy_cam = {}  # store u0,v0,lx,ly if present

        self.desired_point = None
        self.gains = np.zeros(4)
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
        # self.send_to_bringup = False

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
        self.pub_pwm_vs = None           # Int16MultiArray (controller/pwm_vs)
        # self.pub_bringup = None          # UInt16MultiArray (controller/vs_pwm)
        self.sub_image = None
        self.sub_detect = None
        self._timer = None
        self.rate = 20.0
        self.tracking_mode = None

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
                # support legacy scalar keys if present
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

        # publishers (both bringup and direct pwm kept separated)
        pwm_topic = self.get_parameter('pwm_topic').value
        # bringup_topic = self.get_parameter('bringup_topic').value
        self.pub_pwm_vs = self.create_publisher(UInt16MultiArray, pwm_topic, 10)
        # self.pub_bringup = self.create_publisher(UInt16MultiArray, bringup_topic, 10)

        self.get_logger().info(f"Configured topics: image={self.get_parameter('image_topic').value}, detections={self.get_parameter('detections_topic').value}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating VisionController...")
        self.Camera_pwm = dict(pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.sub_image = self.create_subscription(Image,self.get_parameter('image_topic').value, self.image_callback, qos_profile=qos)
        self.sub_detect = self.create_subscription(Detection, self.get_parameter('detections_topic').value, self.color_video_tracking_callback, 10)

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

        # OF tracker
        self.track_points = None
        self.prev_gray = None
        self.roi_w = self.roi_h = 0
        self.frames_since_reinit = 0
        self.last_handle_bbox = None
        self.last_good_center = None
        self.handle_lost_frames = 0
        self.get_logger().info("Deactivating VisionController...")
        if self._timer:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if self.sub_image:
            self.destroy_subscription(self.sub_image); self.sub_image = None
        if self.sub_detect:
            self.destroy_subscription(self.sub_detect); self.sub_detect = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up VisionController...")
        if self.pub_pwm_vs:
            self.destroy_publisher(self.pub_pwm_vs); self.pub_pwm_vs = None
        # if self.pub_bringup:
        #     self.destroy_publisher(self.pub_bringup); self.pub_bringup = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down VisionController...")
        return TransitionCallbackReturn.SUCCESS

    # ---------- parameters ----------
        # Parameter update callback for runtime tuning
    def _on_set_parameters(self, params):
        for p in params:
            try:
                name, val = p.name, p.value

                # Update desired_point safely
                if name.startswith('desired_point_'):
                    if self.desired_point is None:
                        x = self.get_parameter('desired_point_x').value
                        y = self.get_parameter('desired_point_y').value
                        z = self.get_parameter('desired_point_z').value
                        self.desired_point = np.array([x, y, z])
                    if name == 'desired_point_x':
                        self.desired_point[0] = val
                        self.get_logger().info(f"desired_point_x updated to {val}")
                    elif name == 'desired_point_y':
                        self.desired_point[1] = val
                        self.get_logger().info(f"desired_point_y updated to {val}")
                    elif name == 'desired_point_z':
                        self.desired_point[2] = val
                        self.get_logger().info(f"desired_point_z updated to {val}")

                # Update gains directly
                elif name == 'gain_surge':
                    self.gains[2] = val
                    self.get_logger().info(f"gain_surge updated to {val}")
                elif name == 'gain_sway':
                    self.gains[0] = val
                    self.get_logger().info(f"gain_sway updated to {val}")
                elif name == 'gain_heave':
                    self.gains[1] = val
                    self.get_logger().info(f"gain_heave updated to {val}")
                elif name == 'gain_yaw':
                    self.gains[3] = val
                    self.get_logger().info(f"gain_yaw updated to {val}")

                # Update simple numeric/boolean parameters
                elif name == 'floatability':
                    self.floatability = val
                    self.get_logger().info(f"floatability updated to {val}")
                elif name == 'enable_visual_servoing':
                    self.enable_visual_servoing = val
                    self.get_logger().info(f"enable_visual_servoing updated to {val}")
                elif name == 'track_handle':
                    self.track_handle = val
                    self.get_logger().info(f"track_handle updated to {val}")
                elif name == 'fast_surge':
                    self.fast_surge = p.value
                    self.get_logger().info(f"Fast surge when close to handle {'ENABLED' if p.value else 'DISABLED'}")
                elif name == 'enable_of_tracking':
                    self.enable_of_tracking = val
                    self.get_logger().info(f"enable_of_tracking updated to {val}")
                # elif name == 'control_rate':
                #     self.rate = float(val)
                #     if self._timer:
                #         self._timer.cancel()
                #         self.destroy_timer(self._timer)
                #     self._timer = self.create_timer(1.0 / self.rate, self._control_loop)
                #     self.get_logger().info(f"control_rate updated to {val} Hz")
                elif name == 'v_linear_max':
                    self.v_linear_max = val
                    self.get_logger().info(f"v_linear_max updated to {val}")
                elif name == 'v_angular_max':
                    self.v_angular_max = val
                    self.get_logger().info(f"v_angular_max updated to {val}")
                elif name == 'invert_surge':
                    self.invert_surge = val
                    self.get_logger().info(f"invert_surge updated to {val}")
                elif name == 'invert_sway':
                    self.invert_sway = val
                    self.get_logger().info(f"invert_sway updated to {val}")
                elif name == 'invert_heave':
                    self.invert_heave = val
                    self.get_logger().info(f"invert_heave updated to {val}")
                elif name == 'invert_yaw':
                    self.invert_yaw = val
                    self.get_logger().info(f"invert_yaw updated to {val}")
                # Reload calibration file on the fly
                elif name == 'calib_file':
                    try:
                        data = np.load(val)
                        if 'camera_matrix' in data.files and 'dist_coeffs' in data.files:
                            self.camera_matrix = data['camera_matrix']
                            self.dist_coeffs = data['dist_coeffs']
                            self.legacy_cam = {}
                            self.get_logger().info(f"Reloaded camera_matrix/dist_coeffs from {val}")
                        else:
                            self.legacy_cam = {k: float(data[k]) for k in data.files if k in ('u0','v0','lx','ly','kud','kdu')}
                            self.get_logger().info(f"Reloaded legacy camera params from {val}")
                    except Exception as e:
                        self.get_logger().error(f"Failed to reload calib '{val}': {e}")

            except Exception as e:
                self.get_logger().error(f"Error applying parameter {p.name}: {e}")

        return SetParametersResult(successful=True)


    def _update_params_from_ros(self):
        """Centralized parameter pulling (called from on_configure or when param sets are simple)."""
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

        # self.send_to_bringup = self.get_parameter('send_to_bringup').value

    # ---------- control loop ----------
    def _control_loop(self):
        """Publishes PWM messages based on current Camera_pwm state."""
        if not self.enable_visual_servoing:
            # neutral
            for k in self.Camera_pwm: self.Camera_pwm[k] = 1500

        # publish to bringup topic as UInt16MultiArray if requested, else publish Int16MultiArray to pwm_vs
        # if self.send_to_bringup and self.pub_bringup:
        #     msg = UInt16MultiArray()
        #     # ensure values fit 0..65535 (we keep 1100..1900 -> safe)
        #     arr = [np.uint16(self.Camera_pwm['pitch']), np.uint16(self.Camera_pwm['roll']),
        #            np.uint16(self.Camera_pwm['heave']), np.uint16(self.Camera_pwm['yaw']),
        #            np.uint16(self.Camera_pwm['surge']), np.uint16(self.Camera_pwm['sway'])]
        #     msg.data = arr
        #     self.pub_bringup.publish(msg)
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
        # self.get_logger().info("Received image frame")
        try:
            self.image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = self.image_np.shape[:2]
            
            # Initialize desired point if not set or invalid
            if not getattr(self, 'image_center_set', False):
                dp_x = self.get_parameter('desired_point_x').value
                dp_y = self.get_parameter('desired_point_y').value
                dp_z = self.get_parameter('desired_point_z').value
                
                # Auto-center if -1, otherwise validate bounds
                if dp_x < 0 or dp_x >= w:
                    dp_x = w // 2
                    self.get_logger().warn(f"desired_point_x out of bounds, using center: {dp_x}")
                
                if dp_y < 0 or dp_y >= h:
                    dp_y = h // 2
                    self.get_logger().warn(f"desired_point_y out of bounds, using center: {dp_y}")
                
                self.desired_point = np.array([dp_x, dp_y, dp_z])
                self.get_logger().info(f"Set desired point to [{dp_x}, {dp_y}, {dp_z}]")
                self.image_center_set = True

            # compute distance if box available
            bw = max(0, self.blackbox_xmax - self.blackbox_xmin)
            bh = max(0, self.blackbox_ymax - self.blackbox_ymin)
            box_px = min(bw, bh) if bw and bh else bw
            if box_px > 0:
                if self.camera_matrix is not None:
                    f = (self.camera_matrix[0,0] + self.camera_matrix[1,1]) / 2.0
                else:
                    f = self.legacy_cam.get('lx', 455.0)
                known_w = 0.14
                self.distance = (known_w * f) / float(box_px)
                # self.get_logger().info(f"Distance estimated at {self.distance:.2f}m from box size {box_px}px")
            else:
                # neutral outputs kept by control loop
                pass

            # draw visualization (non-blocking)
            self.draw_visualization()

        except Exception as e:
            self.get_logger().error(f"image_callback error: {e}")

    def color_video_tracking_callback(self, msg):
        """
        Visual servoing callback for BlueROV control using surge, sway, heave, and yaw.
        Now with multi-level handle tracking: YOLO -> Features -> OF -> Box fallback
        """
        # If visual servoing is disabled, set neutral commands and return
        if not self.enable_visual_servoing:
            self.Camera_pwm['surge'] = 1500
            self.Camera_pwm['sway']  = 1500
            self.Camera_pwm['heave'] = 1500
            self.Camera_pwm['yaw']   = 1500
            # pitch & roll kept neutral (or could be derived if required)
            self.Camera_pwm['pitch'] = 1500
            self.Camera_pwm['roll']  = 1500
            self.tracking_mode = "DISABLED"
            return
        
        # fast surge when close to the handle
        if self.fast_surge and self.distance <= 0.3:

            # Initialize attachment start time only once
            if self.attachment_start_time is None:
                self.attachment_start_time = time.time()           
                self.gains[2] = 1.0  # Increase surge gain
                self.attachment_duration = 1.5  # seconds
                self.attachment_speed = 0.5  # m/s

            elapsed = time.time() - self.attachment_start_time

            if elapsed < self.attachment_duration:
                # Fast forward surge
                self.Camera_pwm['surge'] = map_to_pwm(self.attachment_speed)
                self.Camera_pwm['sway'] = 1500
                self.Camera_pwm['heave'] = map_to_pwm(self.floatability)
                self.Camera_pwm['yaw'] = 1500
                self.tracking_mode = "ATTACHING"
                return
            else:
                # Attachment duration complete, reset
                self.attachment_start_time = None
                #self.set_parameters([rclpy.parameter.Parameter('fast_surge', rclpy.Parameter.Type.BOOL, False)])
                self.fast_surge = False  # Disable fast surge after completion
                self.gains[2] = self.get_parameter('gain_surge').value  # Reset to normal gain
        else:
            # Conditions not met, reset timer if it was running
            if self.attachment_start_time is not None:
                self.attachment_start_time = None
                self.gains[2] = self.get_parameter('gain_surge').value  # Reset to normal gain
                self.get_logger().warn("Fast surge interrupted, resuming normal tracking")

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
            x_handle, y_handle = self._px2norm([self.handle_xcenter, self.handle_ycenter])
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
                valid_position = True
                # if projected_pos is not None:
                #     # Check if position makes sense relative to box
                #     if self.blackbox_ymin > 0:
                #         # Handle should be above box top (with tolerance)
                #         if projected_pos[1] < self.blackbox_ymin + 150:  # 150px tolerance
                #             valid_position = True
                #         else:
                #             self.get_logger().warn(
                #                 f"Feature tracking invalid: y={projected_pos[1]:.1f} below box_top={self.blackbox_ymin:.1f}",
                #                 throttle_duration_sec=1.0
                #             )
                #     else:
                #         # No box detection to validate against, accept the position
                #         valid_position = True
                
                if valid_position and projected_pos is not None:
                    # Feature tracking successful and validated
                    x, y = self._px2norm(projected_pos)
                    self.tracking_mode = "FEATURE-HANDLE"
                    self.get_logger().info(
                        f"Feature tracking: handle at ({projected_pos[0]:.1f}, {projected_pos[1]:.1f})",
                        throttle_duration_sec=1.0
                    )
                else:
                    # Level 3: Box BOTTOM Center Fallback (LAST RESORT)
                    x, y = self._px2norm([self.blackbox_xcenter, self.blackbox_ymax])
                    self.tracking_mode = "BOX-FALLBACK"
                    self.get_logger().warn(
                        "Feature tracking failed, using box top center",
                        throttle_duration_sec=1.0
                    )
            else:
                # Not enough frames lost yet, use box BOTTOM center
                x, y = self._px2norm([self.blackbox_xcenter, self.blackbox_ymax])
                self.tracking_mode = "BOX-EARLY"
        else:
            # Track BOTTOM center of box (default or when track_handle is disabled)
            box_top_center_x = self.blackbox_xcenter
            box_top_center_y = self.blackbox_ymin  
            x, y = self._px2norm([box_top_center_x, self.blackbox_ymax])
            self.tracking_mode = "BOX-BOTTOM"
            self.handle_lost_frames = 0

        
        # ============ VISUAL SERVOING CONTROL ============
        
        xd, yd = self._px2norm(self.desired_point[:2])
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
        self.get_logger().info(f"gain: {self.gains}, error: {error_combined}, v_cam_4d: {v_cam_4d}", throttle_duration_sec=1.0)
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
        self.Camera_pwm['surge'] = map_to_pwm(surge_sign * v_rov_4d[0])
        self.Camera_pwm['sway']  = map_to_pwm(sway_sign * v_rov_4d[1])
        self.Camera_pwm['heave'] = map_to_pwm(heave_sign * v_rov_4d[2] + self.floatability)
        self.Camera_pwm['yaw']   = map_to_pwm(yaw_sign * v_rov_4d[3])
        self.Camera_pwm['pitch'] = 1500
        self.Camera_pwm['roll']  = 1500

    # ---------- small helpers ----------
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
                # fallback defaults
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
                # Draw box detection (yellow rectangle)
                cv2.rectangle(self.image_np,
                            (int(self.blackbox_xmin), int(self.blackbox_ymin)),
                            (int(self.blackbox_xmax), int(self.blackbox_ymax)),
                            (0, 255, 255), 2)
                
                if "BOX" in self.tracking_mode:
                    # We track BOX TOP CENTER, not box center
                    detected_x = int(self.blackbox_xcenter)
                    detected_y = int(self.blackbox_ymax)  # Changed from blackbox_ycenter!
                    label = 'box bottom center'
                else:
                    # For other modes, show actual box center for reference
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
                
                # Display current gains and floatability
                gains_text = f"Gains [S,Sw,H,Y]: [{self.gains[2]:.2f}, {self.gains[0]:.2f}, {self.gains[1]:.2f}, {self.gains[3]:.2f}] | Float: {self.floatability:.3f}"
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(self.image_np, gains_text,
                        (10, info_y + 5*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Display tracking mode
                status_text = (f"VS: {'ON' if self.enable_visual_servoing else 'OFF'} | "
                            f"Mode: {self.tracking_mode} | Lost: {self.handle_lost_frames} ")
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
    # ---------- optical flow tracker (kept focused) ----------
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
            # init features
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
