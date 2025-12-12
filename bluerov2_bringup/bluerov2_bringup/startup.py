import rclpy
from rclpy.node import Node
from time import sleep
from mavros_msgs.srv import CommandLong, SetMode 
from mavros_msgs.msg import MountControl,OverrideRCIn
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.msg import Transition,State
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16MultiArray, Float64
from bluerov2_util import LifecycleManager
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import math

class bluerov2_bringup(Node):
    def __init__(self):
        super().__init__("bluerov2_bringup")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #### Subscribers #### 
        #TODO:: Make the topic names variable depending on the parameters
        self.sub_manual_pwm = self.create_subscription(
            UInt16MultiArray,
            'controller/manual',
            lambda msg: setattr(self, 'RC_pwms', msg.data),
            qos_profile=qos_profile
        )

        # controller PWM
        self.sub_pwm = self.create_subscription(
            UInt16MultiArray,
            'controller/pwm',
            self.set_controller_pwm,
            qos_profile=qos_profile
        )

        self.camera_angle_sub = self.create_subscription(
            Float64,
            'camera/angle',
            self.angle_callback,
            10
        )

        self.servoing_sub = self.create_subscription(
            UInt16MultiArray,
            'controller/pwm_servoing',
            self.set_servoing_pwm,
            qos_profile=qos_profile
        )

        ##### Publishers #####
        self.pub_camera_anlge = self.create_publisher(MountControl, 'mount_control/command', qos_profile = 10)
        self.pub_msg_override = self.create_publisher(OverrideRCIn, 'rc/override', qos_profile =10)

        ###### Clients ######
        # Service client for setting MAVROS flight modes (set_mode)
        self.cli_set_mode = self.create_client(SetMode, 'set_mode')

        ###### Parameters ####
        self.declare_parameter("initialize", False)
        self.declare_parameter("lights_available", True)
        self.declare_parameter("gripper_available", True)

        self.declare_parameter("gripper_pin", 11.0)
        self.declare_parameter("light_pin", 13.0)
        self.declare_parameter("camera_pen", 16.0)

        self.declare_parameter("light_min", 1100.0)
        self.declare_parameter("light_max", 1900.0)
        self.declare_parameter("gripper_open", 1600.0)
        self.declare_parameter("gripper_close", 1300.0)
        self.declare_parameter("tilt_int", 0.0)
        self.declare_parameter("tilt_max", 85.0)
        self.declare_parameter("tilt_min", -85.0)
        self.declare_parameter("mode", "manual")
        self.declare_parameter("enable_gimbal", False)

        self.light_pin = self.get_parameter("light_pin").value
        self.camera_servo_pin = self.get_parameter("camera_pen").value
        self.gripper_pin = self.get_parameter("gripper_pin").value
        self.light_min = self.get_parameter("light_min").value
        self.light_max = self.get_parameter("light_max").value
        self.gripper_open = self.get_parameter("gripper_open").value
        self.gripper_close = self.get_parameter("gripper_close").value
        self.tilt_int = self.get_parameter("tilt_int").value
        self.camera_tilt_max = self.get_parameter("tilt_max").value
        self.camera_tilt_min = self.get_parameter("tilt_min").value
        self.lights_available = self.get_parameter("lights_available").value
        self.gripper_available = self.get_parameter("gripper_available").value
        self.mode = self.get_parameter("mode").value
        self.initialization_test_flag = self.get_parameter("initialize").value
        self.enable_gimbal = self.get_parameter("enable_gimbal").value
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        self.mode_change = False
        if self.initialization_test_flag:
            self.initialization_test()

        self.RC_pwms = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.surge = 1500
        self.sway = 1500
        self.heave = 1500
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.surge_servoing, self.sway_servoing, self.heave_servoing, self.roll_servoing, self.pitch_servoing, self.yaw_servoing = 1500, 1500, 1500, 1500, 1500, 1500

         #### Lifecycle Managers ####
        self.depth_controller_node_name = "depth_controller"
        self.yaw_controller_node_name = "yaw_controller"
        self.pitch_controller_node_name = "pitch_controller"
        self.roll_controller_node_name = "roll_controller"
        self.gimbal_node_name = "bluerov2_gimbal"
        self.serch_node_name = "search_pattern"
        self.approaching_node_name = "box_approaching"
        self.vs_node_name = "visual_servoing"

        self.depth_manager = LifecycleManager(self, self.depth_controller_node_name)
        self.yaw_manager = LifecycleManager(self, self.yaw_controller_node_name)
        self.pitch_manager = LifecycleManager(self, self.pitch_controller_node_name)
        self.roll_manager = LifecycleManager(self, self.roll_controller_node_name)

        self.gimbal_manager = LifecycleManager(self, self.gimbal_node_name)

        self.vs_manager = LifecycleManager(self, self.vs_node_name)

        self.search_manager = LifecycleManager(self, self.serch_node_name)
        self.approaching_manger = LifecycleManager(self, self.approaching_node_name)

        self.depth_manager.configure()
        self.yaw_manager.configure()
        self.pitch_manager.configure()
        self.gimbal_manager.configure()
        self.roll_manager.configure()

        self.vs_manager.configure()

        self.search_manager.configure()
        self.approaching_manger.configure()

        self.create_timer(0.2, self.timer_callback)

         # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Last angle
        self.last_angle_deg = 0.0

        self.depth_active = False
        self.yaw_active = False
        self.auto_hold = False

        # Timer to publish TF at 10 Hz
        # self.timer = self.create_timer(0.1, self.publish_tf)


   
    def parameter_update_callback(self, params):
        for param in params:
           if param.name == "enable_gimbal":
                self.enable_gimbal = param.value
                if self.enable_gimbal:
                    self.get_logger().info(f"Gimbal controller enabled")
                else:
                    self.get_logger().info("Gimbal controller disabled")

        return SetParametersResult(successful=True)
    
    def set_controller_pwm(self,msg):
        self.surge, self.sway, self.heave, self.roll, self.pitch, self.yaw = msg.data

    def set_servoing_pwm(self,msg):
        self.surge_servoing, self.sway_servoing, self.heave_servoing, self.roll_servoing, self.pitch_servoing, self.yaw_servoing = msg.data

    def timer_callback(self):
        self.get_mode("mode")
        #### Sending Thruster Commands ####
        # if self.mode == "correction" or self.mode == "approaching" or self.mode == "search":
        #     # self.RC_pwms[0] = self.pitch
        #     # self.RC_pwms[1] = self.roll
        #     # # self.RC_pwms[2] += self.heave - 1500
        #     # # self.RC_pwms[3] += self.yaw - 1500
        #     # # self.RC_pwms[4] += self.surge - 1500
        #     # # self.RC_pwms[5] += self.sway - 1500

        #     # self.RC_pwms[2] = self.heave
        #     # self.RC_pwms[3] = self.yaw
        #     # self.RC_pwms[4] = self.surge
        #     # self.RC_pwms[5] = self.sway
        #     self.get_logger().info(f"heave pwm: {self.heave}, sent pwm:{self.RC_pwms[2]}")

        if self.mode == "servoing":
            # self.get_logger().info("In servoing mode")
            self.RC_pwms[0] = self.pitch_servoing
            self.RC_pwms[1] = self.roll_servoing
            self.RC_pwms[2] = self.heave_servoing
            self.RC_pwms[3] = self.yaw_servoing
            self.RC_pwms[4] = self.surge_servoing
            self.RC_pwms[5] = self.sway_servoing

        self.setOverrideRCIN(self.RC_pwms)
        
        if self.mode_change:
            self.get_logger().info(f"Mode changed to {self.mode}")
            
            if self.mode == "correction":
                # Toggle MAVROS to Alt Hold when entering correction mode
                self.set_mav_mode("ALT_HOLD")
                self.auto_hold = True
                # self.search_manager.deactivate()
                # self.approaching_manger.deactivate()
                self.vs_manager.deactivate()
                # self.depth_manager.activate()
                # self.yaw_manager.activate()

                # self.depth_active = True
                # self.yaw_active = True
                # self.pitch_manager.activate()
                # self.roll_manager.activate()

                # if self.enable_gimbal:
                #     self.gimbal_manager.activate()
            elif self.mode == "manual":
                # Toggle MAVROS to Manual when entering manual mode
                self.set_mav_mode("MANUAL")
                self.auto_hold = False
                self.surge_servoing, self.sway_servoing, self.heave_servoing, self.roll_servoing, self.pitch_servoing, self.yaw_servoing = 1500, 1500, 1500, 1500, 1500, 1500

                # self.roll_manager.deactivate()
                # if self.depth_active:
                #     self.depth_manager.deactivate()
                # if self.yaw_active:
                #     self.yaw_manager.deactivate()
                # # self.pitch_manager.deactivate25.()
                # self.search_manager.deactivate()
                # self.approaching_manger.deactivate()
                self.vs_manager.deactivate()
                self.RC_pwms[0] = 1500
                self.RC_pwms[1] = 1500
                self.RC_pwms[2] = 1500
                self.RC_pwms[3] = 1500
                self.RC_pwms[4] = 1500
                self.RC_pwms[5] = 1500
                
            elif self.mode == "search":
                self.get_logger().info("No box found, starting search pattern")
                self.depth_manager.activate()
                self.yaw_manager.activate()
                self.pitch_manager.deactivate()
                self.search_manager.activate()
                self.approaching_manger.deactivate()
                self.roll_manager.deactivate()
                self.depth_active = True
                self.yaw_active = True
            elif self.mode == "approaching":
                self.get_logger().info("Box found, starting approaching behavior")
                self.depth_manager.activate()
                self.yaw_manager.activate()
                self.pitch_manager.deactivate()
                self.search_manager.deactivate()
                self.approaching_manger.activate()
                self.roll_manager.deactivate()

            elif self.mode == "servoing":
                if not self.auto_hold:
                    self.set_mav_mode("ALT_HOLD")
                    
                self.get_logger().info("Starting visual servoing")
                # self.depth_manager.deactivate()
                # self.yaw_manager.deactivate()
                # self.roll_manager.deactivate()
                # self.pitch_manager.deactivate()
                # self.search_manager.deactivate()
                # self.approaching_manger.deactivate()
                self.vs_manager.activate()
              

            self.mode_change = False 

    def set_mav_mode(self, custom_mode):
        """Requests MAVROS to change flight mode."""
        if self.cli_set_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = custom_mode
            req.base_mode = 0
            
            self.get_logger().info(f"Requesting MAVROS mode: {custom_mode}")
            future = self.cli_set_mode.call_async(req)
            # Optional: Add callback to check result if needed
        else:
            self.get_logger().warn("SetMode service (cmd/set_mode) not ready!")
               
        

    def initialization_test(self):
        """Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem."""
        self.get_logger().info("Starting initialization test for lights, camera servo and gripper.")
        # Flash the light
        if self.lights_available:
            self.light = self.light_min
            self.get_logger().info(f"ligth pin:{self.light_pin}, light value:{self.light}")
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)
            self.light = self.light_max
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)
            self.light = self.light_min
            self.send_servo_comand(self.light_pin, self.light)
            sleep(0.5)

        # Move the camera servo to max and min
        if self.gripper_available:
            self.gripper = self.gripper_close
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)
            self.gripper = self.gripper_open
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)
            self.gripper = self.gripper_close  
            self.send_servo_comand(self.gripper_pin, self.gripper)
            sleep(1.0)

         # Move the camera servo to max and min
        self.tilt = self.camera_tilt_max
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        self.tilt = self.camera_tilt_min
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        self.tilt = self.tilt_int
        self.set_camera_tilt(self.tilt)
        sleep(1.0)
        
        
        self.get_logger().info("Light and camera servo test completed.")

        
    def send_servo_comand(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board (13 for lights and 15 for camera servo)
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
                result = client.wait_for_service(timeout_sec=4.0)
        # Create a request object for CommandLong service
        request = CommandLong.Request()


        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0             


        # Send the service request and wait for the response
        future = client.call_async(request)

    def set_camera_tilt(self, tilt_angle):
        ''' Set the camera tilt angle in degrees'''
        msg = MountControl()
        msg.pitch = tilt_angle# tilt down
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.mode = 2        # MAVLINK_MOUNT_MODE_MAVLINK_TARGETING
        self.pub_camera_anlge.publish(msg)


    def setOverrideRCIN(self, pwms):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set '''


        msg_override = OverrideRCIn()
        msg_override.channels[0] = pwms[0]  # pulseCmd[4]--> pitch
        msg_override.channels[1] = pwms[1]   # pulseCmd[3]--> roll
        msg_override.channels[2] = pwms[2]  # pulseCmd[2]--> heave
        msg_override.channels[3] = pwms[3]   # pulseCmd[5]--> yaw
        msg_override.channels[4] = pwms[4]   # pulseCmd[0]--> surge
        msg_override.channels[5] = pwms[5]   # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500 # camera pan servo motor speed 
        msg_override.channels[7] = 1500 #camers tilt servo motro speed


        self.pub_msg_override.publish(msg_override)

    def get_mode(self, name):
        target_node = 'bluerov2_teleop'
        self.client = self.create_client(GetParameters, f'{target_node}/get_parameters')

        if not self.client.service_is_ready():
            # self.get_logger().warn(f'Parameter service not ready on {target_node}, skipping request.')
            return

        request = GetParameters.Request()
        request.names = [name]  # List of parameter names

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_callback)

    def parameter_callback(self, future):
        try:
            response = future.result()
            value = response.values[0].string_value
            if getattr(self, "mode", None) != value:
                self.mode_change = True
            else:
                self.mode_change = False
            self.mode = value
        except Exception as e:
            self.get_logger().error(f'Failed to get parameter: {e}')


    def angle_callback(self, msg: Float64):
            self.last_angle_deg = msg.data  # in degrees

    def publish_tf(self):
        # Convert angle to radians
        angle_rad = math.radians(self.last_angle_deg)

        # Compute offsets
        z = -0.2 * math.cos(angle_rad)
        y = 0.2 * math.sin(angle_rad)
        pitch = -math.pi/2+angle_rad # -90 deg + angle

        # Build quaternion (roll= -pi/2, pitch=pitch, yaw=0)
        quat = quaternion_from_euler(-math.pi/2,pitch, 0.0,axes='rzyx')

        # Create TransformStamped
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'camera'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = y
        # t.transform.translation.z = z
        # t.transform.rotation.x = quat[0]
        # t.transform.rotation.y = quat[1]
        # t.transform.rotation.z = quat[2]
        # t.transform.rotation.w = quat[3]

        # # Broadcast TF
        # self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = bluerov2_bringup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()