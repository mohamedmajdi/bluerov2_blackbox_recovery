#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt16MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ForceController(Node):
    def __init__(self):
        super().__init__('frame_transform')
        self.get_logger().info("starting frame transformation")
        # Parameters for topic names
        self.declare_parameter('heave_topic', 'heave_force')
        self.declare_parameter('pitch_topic', 'pitch_torque')
        self.declare_parameter('yaw_topic', 'yaw_torque')
        self.declare_parameter('surge_topic','searching/surge_force')
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('output_topic', 'forces_pwm')
        self.declare_parameter('publish_rate', 0.1)  # seconds

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Get parameter values
        heave_topic = self.get_parameter('heave_topic').get_parameter_value().string_value
        pitch_topic = self.get_parameter('pitch_topic').get_parameter_value().string_value
        yaw_topic = self.get_parameter('yaw_topic').get_parameter_value().string_value
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

        # Publisher
        self.pwm_pub = self.create_publisher(UInt16MultiArray, pwm_topic, 10)

        # Store latest values
        self.heave = 0.0
        self.surge = 0.0
        self.pitch_torque = 0.0
        self.yaw_torque = 0.0
        self.roll = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0

        self.rotation = None


        # Timer for publishing every `publish_rate` seconds
        self.create_timer(publish_rate, self.publish_forces_pwm)

    def heave_callback(self, msg):
        self.heave = msg.data

    def pitch_callback(self, msg):
        self.pitch_torque = msg.data

    def yaw_callback(self, msg):
        self.yaw_torque = msg.data

    def surge_callback(self, msg):
        self.surge = msg.data

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.rotation = msg.orientation
        self.roll, self.pitch_angle, self.yaw_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def publish_forces_pwm(self):
        # Build NED forces/torques vector
        if self.rotation is None:
            return
        forces_ned = np.array([
            0, 0, self.heave,      # linear forces X, Y, Z in NED
            0, self.pitch_torque, self.yaw_torque  # torques roll, pitch, yaw
        ])

        # Compute rotation matrix from IMU orientation
        # q = quaternion_from_euler(self.roll, self.pitch_angle+0.1, self.yaw_angle)
        # q =self.rotation
        q = [self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w]
        R = quaternion_matrix(q)[:3, :3]
        # Transform linear forces and torques to robot frame
        lin_forces_robot = np.dot(R, forces_ned[:3])
        torques_robot = np.dot(R, forces_ned[3:6])
        forces_robot = np.hstack([lin_forces_robot, torques_robot])


        ###### for pattern search #####
        forces_robot[0] += self.surge
        ###############################
        
        # self.get_logger().info(f"force:{forces_robot}")

        # Convert to PWM

        # Added negative just to see how it will work
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
