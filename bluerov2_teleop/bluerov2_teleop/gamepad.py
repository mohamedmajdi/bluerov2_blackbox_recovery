import rclpy
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Float64MultiArray, String, UInt16MultiArray
from sensor_msgs.msg import Joy, Imu
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink, MountControl
from mavros_msgs.srv import EndpointAdd
from geometry_msgs.msg import Twist
from time import sleep
from rclpy.parameter import Parameter

class GamepadTelop(Node):
    def __init__(self):
        super().__init__("bluerov2_gamepad_teleop")
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
        self.declare_parameter("tilt_max", 45.0)
        self.declare_parameter("tilt_min", -45.0)

        self.declare_parameter("arm_at_startup", False)
        self.declare_parameter("mode", "manual")
        self.get_logger().info('Gamepad Teleop Node has been started.')

        self.arming = self.get_parameter("arm_at_startup").value
        self.mode = self.get_parameter("mode").value
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

        self.light = self.light_min
        self.tilt = self.tilt_int
        self.gripper = self.gripper_close
        
        # Track previous state of the Logitech button
        self.prev_logitech_btn = 0

        self.pub_msg_override = self.create_publisher(UInt16MultiArray, "controller/manual", 10)
        self.pub_camera_anlge = self.create_publisher(MountControl, 'mount_control/command',10)
        self.pub_feedback_camera = self.create_publisher(Float64,"camera/angle",10)
        
        # New publisher for approaching status
        self.pub_approaching = self.create_publisher(String, "visual_servoing/approaching", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )



        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joyCallback,
            qos_profile=qos_profile)
        self.joy_subscriber  # prevent unused variable warning
        self.cmd_subscriber = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile=qos_profile)
        self.cmd_subscriber  # prevent unused variable warning

       # Subscription
        self.sub_camera_angle = self.create_subscription(
            MountControl,  # replace MountStatus
            'mount_control/status',
            self.get_camera_angle,
            qos_profile= qos_profile
        )
        self.sub_camera_angle  # prevent unused variable warning

    def get_camera_angle(self, msg):
        """
        Get current camera tilt angle from the mount feedback message.
        For MountControl, pitch is in msg.pitch
        """
        self.tilt = msg.pitch
    def joyCallback(self, data):
        ''' Map the Joystick buttons according the bluerov configuration as descriped at
        https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg'''
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button

        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button
        
        btn_camera_servo_up = data.buttons[4] # LB button 
        btn_camera_servo_down = data.buttons[5] # RB button 
        btn_camera_rest = data.buttons[9] # R3 button 

        # Logitech button is typically index 8 on F310
        btn_logitech = data.buttons[8] 

        btn_gripper_close = data.axes[2] # LT button
        btn_gripper_open = data.axes[5] # RT button

        btn_light= data.axes[7] # right arrow
        # btn_light_down = data.axes[12] # left arrow
        

        # --- Logitech Button "Approaching" Logic ---
        msg = String()
        if btn_logitech == 1:
            # While pressed, send "allowed"
            msg.data = "allowed"
            self.pub_approaching.publish(msg)
            # Optional: Log only if needed, otherwise it might spam the console
            # self.get_logger().info("Approaching: allowed")

        elif btn_logitech == 0 and self.prev_logitech_btn == 1:
            # On release (falling edge), send "denied" and switch to manual
            msg.data = "denied"
            self.pub_approaching.publish(msg)
            
            # Switch back to manual mode
            if self.mode != "manual":
                self.mode = "manual"
                self.get_logger().info("Logitech button released: Mode switched to manual")
                self.set_parameters([Parameter('mode', Parameter.Type.STRING, self.mode)])

        # Update previous button state
        self.prev_logitech_btn = btn_logitech
        # -------------------------------------------


        # Disarming when Back button is pressed
        if (btn_disarm == 1 and self.arming == True):
            self.arming = False
            self.armDisarm(self.arming)


        # Arming when Start button is pressed
        if (btn_arm == 1 and self.arming == False):
            self.arming = True
            self.armDisarm(self.arming)


        # Switch manual, auto anset_moded correction mode
        if (btn_manual_mode and not self.mode == "manual"):
            self.mode = "manual"
            self.get_logger().info("Mode manual")
        if (btn_automatic_mode and not self.mode == "servoing"):
            self.mode = "servoing"
            self.get_logger().info("Mode servoing")
        if (btn_corrected_mode and not self.mode == "correction"):
            self.mode = "correction"
            self.get_logger().info("Mode correction")
        
        self.set_parameters([Parameter('mode', Parameter.Type.STRING, self.mode)])

        
        #### Control light intensity####
        if self.lights_available:
            if (btn_light == 1 and self.light < self.light_max):
                self.light = min(self.light + 100.0, self.light_max)
                self.send_servo_comand(self.light_pin,self.light)
                self.get_logger().info(f"light PWM is: {self.light}")
                
            elif (btn_light == -1 and self.light > self.light_min):
                self.light = max(self.light_min,self.light - 100)
                self.send_servo_comand(self.light_pin,self.light)
                self.get_logger().info(f"light PWM is: {self.light}")

        ### Gripper control ###
        if self.gripper_available:
            if (btn_gripper_open == -1 and self.gripper < self.gripper_open):
                self.gripper = self.gripper_open
                self.send_servo_comand(self.gripper_pin,self.gripper)
                self.get_logger().info(f"gripper is opened")
                
            elif (btn_gripper_close == -1 and self.gripper > self.gripper_close):
                self.gripper = self.gripper_close
                self.send_servo_comand(self.gripper_pin,self.gripper)
                self.get_logger().info(f"gripper is closed")



        ### Control Camera tilt angle ###
        if (btn_camera_servo_up and not btn_camera_servo_down and self.tilt < self.camera_tilt_max):
            self.tilt = min(self.camera_tilt_max, self.tilt + 5.0)
            self.set_camera_tilt(self.tilt)
            angle_msg = Float64()
            angle_msg.data = self.tilt
            self.pub_feedback_camera.publish(angle_msg)
            self.get_logger().info(f"tilt angle: {self.tilt}")
            
        elif (btn_camera_servo_down and self.tilt > self.camera_tilt_min):
            self.tilt = max(self.camera_tilt_min, self.tilt - 5.0)
            self.set_camera_tilt(self.tilt)
            angle_msg = Float64()
            angle_msg.data = self.tilt
            self.pub_feedback_camera.publish(angle_msg)
            self.get_logger().info(f"tilt angle: {self.tilt}")
            
        elif (btn_camera_rest):
            self.tilt = self.tilt_int
            self.set_camera_tilt(self.tilt)
            angle_msg = Float64()
            angle_msg.data = self.tilt
            self.pub_feedback_camera.publish(angle_msg)
            self.get_logger().info(f"Camera tilt has been reseted")

    def armDisarm(self, armed):
        """Arms or disarms the vehicle motors using MAVROS command 400."""
        cli = self.create_client(CommandLong, 'cmd/command')  # Create MAVROS service client
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)  # Wait for service to be available
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        # Create request object for arming/disarming
        req = CommandLong.Request()
        req.broadcast = False  # Command is not broadcasted
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0  # No confirmation required
        req.param1 = 1.0 if armed else 0.0  # 1.0 = Arm, 0.0 = Disarm
        req.param2 = 0.0  
        req.param3 = 0.0  
        req.param4 = 0.0  
        req.param5 = 0.0  
        req.param6 = 0.0  
        req.param7 = 0.0 
        
        self.get_logger().info("Sending command...")
        resp = cli.call_async(req)  # Send command asynchronously
        
        # Log the result
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")
            
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


        # Check the result
        if future.result() is not None:
            self.get_logger().info('Change Completed')
        else:
            self.get_logger().error('Failed to preform the change ')


    def velCallback(self, cmd_vel):
        ''' Used in manual mode to read the values of the analog and map it pwm then send it the thrusters'''
        if (self.mode != "manual"):
            return
        else:
            self.get_logger().info("Sending...")

        
        # Extract cmd_vel message
        # roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        # pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)

        roll_left_right = 1500
        pitch_left_right = 1500

        
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        # self.forward_reverse = forward_reverse
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        # send the commands to the mthrusters 
        self.setPWM(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                                lateral_left_right)
        


    def setPWM(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set '''

        
        msg_array = UInt16MultiArray()

        # Fill it with your channel data in order
        msg_array.data = [
            int(channel_pitch),    # Channel 0
            int(channel_roll),     # Channel 1
            int(channel_throttle), # Channel 2
            int(channel_yaw),      # Channel 3
            int(channel_forward),  # Channel 4
            int(channel_lateral),  # Channel 5
            1500,                        # Channel 6 (camera pan)
            1500                         # Channel 7 (camera tilt)
        ]
        self.pub_msg_override.publish(msg_array)


    def mapValueScalSat(self, value):
        ''' Map the value of the joysteck analog form -1 to 1 to a pwm value form 1100 to 1900
            where 1500 is the stop value 1100 is maximum negative and 1900 is maximum positive'''
        pulse_width = value * 400 + 1500

        
        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100
        return int(pulse_width)
    def set_camera_tilt(self, tilt_angle):
        ''' Set the camera tilt angle in degrees'''
        msg = MountControl()
        msg.pitch = tilt_angle# tilt down
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.mode = 2        # MAVLINK_MOUNT_MODE_MAVLINK_TARGETING
        self.pub_camera_anlge.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTelop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()