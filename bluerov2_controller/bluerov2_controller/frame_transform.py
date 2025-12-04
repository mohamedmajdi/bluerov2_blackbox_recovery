#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt16MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult

class ForceController(Node):
    def __init__(self):
        super().__init__('frame_transform')
        self.get_logger().info("starting frame transformation")
        # Parameters for topic names
        self.declare_parameter('heave_topic', 'heave_force')
        self.declare_parameter('pitch_topic', 'pitch_torque')
        self.declare_parameter('yaw_topic', 'yaw_torque')
        self.declare_parameter('roll_topic', 'roll_torque')
        self.declare_parameter('surge_topic','searching/surge_force')
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('output_topic', 'forces_pwm')
        self.declare_parameter('publish_rate', 0.1)  # seconds
        self.declare_parameter('invert_surge',False)
        self.declare_parameter('invert_sway',False)
        self.declare_parameter('invert_heave',False)
        self.declare_parameter('invert_roll',False)
        self.declare_parameter('invert_pitch',False)
        self.declare_parameter('invert_yaw',False)
        self.declare_parameter('surge_scale',1.0)
        self.declare_parameter('sway_scale',1.0)

        self.add_on_set_parameters_callback(self._on_set_parameters)



        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Get parameter values
        heave_topic = self.get_parameter('heave_topic').get_parameter_value().string_value
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        yaw_topic = self.get_parameter('yaw_topic').get_parameter_value().string_value
        roll_topic = self.get_parameter('roll_topic').get_parameter_value().string_value
        surge_topic = self.get_parameter('surge_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        pwm_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Subscribers
        self.create_subscription(Float64, heave_topic, self.heave_callback, 10)
        self.create_subscription(Float64, pitch_topic, self.pitch_callback, 10)
        self.create_subscription(Float64, yaw_topic, self.yaw_callback, 10)
        self.create_subscription(Float64, surge_topic, self.surge_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile= qos_profile)
        self.create_subscription(Float64, roll_topic, self.roll_callback, 10)
        # Publisher
        self.pwm_pub = self.create_publisher(UInt16MultiArray, pwm_topic, 10)

        # Store latest values
        self.heave = 0.0
        self.surge = 0.0
        self.pitch_torque = 0.0
        self.yaw_torque = 0.0
        self.roll_torque = 0.0
        self.roll = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        self.roll_angle = 0.0
        self.rotation = None

        self.invert_surge = self.get_parameter('invert_surge').get_parameter_value().bool_value
        self.invert_sway = self.get_parameter('invert_sway').get_parameter_value().bool_value
        self.invert_heave = self.get_parameter('invert_heave').get_parameter_value().bool_value
        self.invert_roll = self.get_parameter('invert_roll').get_parameter_value().bool_value
        self.invert_pitch = self.get_parameter('invert_pitch').get_parameter_value().bool_value
        self.invert_yaw = self.get_parameter('invert_yaw').get_parameter_value().bool_value

        self.surge_scale = self.get_parameter('surge_scale').get_parameter_value().double_value
        self.sway_scale = self.get_parameter('sway_scale').get_parameter_value().double_value


        # Timer for publishing every `publish_rate` seconds
        self.create_timer(publish_rate, self.publish_forces_pwm)

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'invert_surge':
                self.invert_surge = p.value
                if self.invert_surge:
                    self.get_logger().info("surge force inverted")
                else:
                    self.get_logger().info("surge force not inverted")
            elif p.name == 'invert_sway':           
                self.invert_sway = p.value
                if self.invert_sway:
                    self.get_logger().info("sway force inverted")
                else:
                    self.get_logger().info("sway force not inverted")
            elif p.name == 'invert_heave':           
                self.invert_heave = p.value
                if self.invert_heave:
                    self.get_logger().info("heave force inverted")
                else:
                    self.get_logger().info("heave force not inverted")
            elif p.name == 'invert_roll':           
                self.invert_roll = p.value
                if self.invert_roll:
                    self.get_logger().info("roll torque inverted")
                else:
                    self.get_logger().info("roll torque not inverted")
            elif p.name == 'invert_pitch':           
                self.invert_pitch = p.value
                if self.invert_pitch:
                    self.get_logger().info("pitch torque inverted")
                else:
                    self.get_logger().info("pitch torque not inverted")
            elif p.name == 'invert_yaw':           
                self.invert_yaw = p.value
                if self.invert_yaw:
                    self.get_logger().info("yaw torque inverted")
                else:
                    self.get_logger().info("yaw torque not inverted")

            elif p.name == 'surge_scale':
                self.surge_scale = p.value
                self.get_logger().info(f"surge scale updated to: {self.surge_scale}")
            elif p.name == 'sway_scale':
                self.sway_scale = p.value
                self.get_logger().info(f"sway scale updated to: {self.sway_scale}")

        return SetParametersResult(successful=True)


    def heave_callback(self, msg):
        self.heave = msg.data

    def pitch_callback(self, msg):
        self.pitch_torque = msg.data

    def yaw_callback(self, msg):
        self.yaw_torque = msg.data

    def surge_callback(self, msg):
        self.surge = msg.data

    def roll_callback(self, msg):
        self.roll_torque = msg.data
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.rotation = msg.orientation
        self.roll_angle, self.pitch_angle, self.yaw_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def publish_forces_pwm(self):
        # Build NED forces/torques vector
        if self.rotation is None:
            return
        forces_ned = np.array([
            0, 0, self.heave,      # linear forces X, Y, Z in NED
            self.roll_torque, self.pitch_torque, self.yaw_torque  # torques roll, pitch, yaw
        ])

        # Compute rotation matrix from IMU orientation
        # q = quaternion_from_euler(self.roll, self.pitch_angle+0.1, self.yaw_angle)
        # q =self.rotation
        q = [self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w]
        R = quaternion_matrix(q)[:3, :3]
        # Transform linear forces and torques to robot frame
        lin_forces_robot = np.dot(R, forces_ned[:3])
        # torques_robot = np.dot(R, forces_ned[3:6])
        torques_robot = forces_ned[3:6]
        forces_robot = np.hstack([lin_forces_robot, torques_robot])


        ###### for pattern search #####
        forces_robot[0] += self.surge
        ###############################
        
        # self.get_logger().info(f"force:{forces_robot}")
        
        # Convert to PWM
        # forces_robot[0] = -forces_robot[0]
        # Added negative just to see how it will work

        forces_robot[0] *= self.surge_scale
        forces_robot[1] *= self.sway_scale 

        if self.invert_surge:
            forces_robot[0] = -forces_robot[0]
        if self.invert_sway:
            forces_robot[1] = -forces_robot[1]
        if self.invert_heave:
            forces_robot[2] = -forces_robot[2]
        if self.invert_roll:
            forces_robot[3] = -forces_robot[3]
        if self.invert_pitch:
            forces_robot[4] = -forces_robot[4]
        if self.invert_yaw:
            forces_robot[5] = -forces_robot[5]

        pwm_values = [self._force_to_pwm(-f) for f in forces_robot]
        # Publish
        msg = UInt16MultiArray()
        msg.data = [int(p) for p in pwm_values]
        self.pwm_pub.publish(msg)
        # self.get_logger().info(f"pwm:{pwm_values}")

        # self.get_logger().debug(f"Published PWM: {msg.data}")

    def _force_to_pwm(self, force):
        if force > 0:
            pwm = 1500 + 9.7 * force
        elif force == 0:
            pwm = 1500
        else:
            pwm = 1500 + 12.781 * force
        return np.clip(pwm, 1300, 1700)

def main():
    rclpy.init()
    node = ForceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
