#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from mavros_msgs.msg import RCOut
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class BlueROVDeadReckoning(Node):
    def __init__(self):
        super().__init__('bluerov_dead_reckoning')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.declare_parameter('T1_inverted', True)
        self.declare_parameter('T2_inverted', True)
        self.declare_parameter('T3_inverted', False)
        self.declare_parameter('T4_inverted', False)
        self.declare_parameter('T5_inverted', False)
        self.declare_parameter('T6_inverted', False)
        self.declare_parameter('T7_inverted', False)
        self.declare_parameter('T8_inverted', False)

        self.T1_inverted = self.get_parameter('T1_inverted').get_parameter_value().bool_value
        self.T2_inverted = self.get_parameter('T2_inverted').get_parameter_value().bool_value
        self.T3_inverted = self.get_parameter('T3_inverted').get_parameter_value().bool_value
        self.T4_inverted = self.get_parameter('T4_inverted').get_parameter_value().bool_value
        self.T5_inverted = self.get_parameter('T5_inverted').get_parameter_value().bool_value
        self.T6_inverted = self.get_parameter('T6_inverted').get_parameter_value().bool_value
        self.T7_inverted = self.get_parameter('T7_inverted').get_parameter_value().bool_value
        self.T8_inverted = self.get_parameter('T8_inverted').get_parameter_value().bool_value


        
        self.add_on_set_parameters_callback(self._on_set_parameters)


        # Robot parameters
        self.m = 11.5
        self.W = 112.8
        self.B = 113.8
        self.r_cb = np.array([0, 0, 0])
        self.r_cg = np.array([0, 0, 0.02])
        self.I = np.diag([0.16, 0.16, 0.16])

        # Added mass (positive values)
        self.MA = np.diag([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])


        # Linear damping (positive, dissipative)
        self.D = np.diag([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])


        # Quadratic damping (positive)
        self.Dq = np.diag([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])
        # Thruster allocation matrix
        self.Bmat = np.array([
            [0.707, 0.707, -0.707, -0.707, 0, 0, 0, 0],
            [-0.707, 0.707, -0.707, 0.707, 0, 0, 0, 0],
            [0, 0, 0, 0, -1, 1, 1, -1],
            [0.0, -0.0, 0.0, -0.0, -0.218, -0.218, 0.218, 0.218],
            [0.0, 0.0, -0.0, -0.0, 0.120, -0.120, 0.120, -0.120],
            [-0.1888/2, 0.1888/2, 0.1888/2, -0.1888/2, 0, 0, 0, 0]
        ])

        # Initial state
        self.eta = np.array([0.0, 7.5,0.0,0.0,0.0,0.0])  # [x y z roll pitch yaw]
        # self.eta = np.zeros(6)
        self.v = np.zeros(6)    # [u v w p q r]

        # Thruster forces (replace with real input)
        self.tau_thrusters = np.zeros(8)

        # Publisher
        self.pub = self.create_publisher(Odometry, 'dead_reckoning_odom', 10)

        #subscribers
        self.pwm_sub = self.create_subscription(RCOut,'rc/out',self.cal_thrust,10)
        # self.imu_sub = self.create_subscription(Imu,'imu/data',self.cal_angle,qos_profile=qos_profile)
        # self.filtered_sub = self.create_subscription(Odometry,'odometry/filtered',self.update_pose,10)
        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # HIGH covariance (low confidence)
        self.cov = 0.01
        self.pose_cov = (np.eye(6) * self.cov).flatten().tolist()
        self.twist_cov = (np.eye(6) * self.cov).flatten().tolist()

        self.i = 1

    # def update_pose(self,msg):
    #     f_x = msg.pose.pose.position.x
    #     f_y = msg.pose.pose.position.y
    #     f_z = msg.pose.pose.position.z

    #     qx = msg.pose.pose.orientation.x
    #     qy = msg.pose.pose.orientation.y
    #     qz = msg.pose.pose.orientation.z
    #     qw = msg.pose.pose.orientation.w

    #     # Convert quaternion -> roll pitch yaw
    #     roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    #     # Extract velocities (if available)
    #     u = msg.twist.twist.linear.x
    #     v = msg.twist.twist.linear.y
    #     w = msg.twist.twist.linear.z
    #     p = msg.twist.twist.angular.x
    #     q = msg.twist.twist.angular.y
    #     r = msg.twist.twist.angular.z

    #     self.eta = [f_x,f_y,f_z,roll,pitch,yaw]
    #     self.v = [u,v,w,p,q,r]

    def cal_angle(self,msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion -> roll pitch yaw
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

        self.eta[3] = roll
        self.eta[4] = pitch
        self.eta[5] = yaw

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'T1_inverted':
                self.T1_inverted = p.value
                self.get_logger().info(f"Thruster 1 inversion updated to: {self.T1_inverted}")
            elif p.name == 'T2_inverted':
                self.T2_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T2_inverted}")
            elif p.name == 'T3_inverted':
                self.T3_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T3_inverted}")
            elif p.name == 'T4_inverted':
                self.T4_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T4_inverted}")
            elif p.name == 'T5_inverted':
                self.T5_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T5_inverted}")
            elif p.name == 'T6_inverted':
                self.T6_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T6_inverted}")
            elif p.name == 'T7_inverted':
                self.T7_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T7_inverted}")
            elif p.name == 'T8_inverted':
                self.T8_inverted = p.value
                self.get_logger().info(f"Thruster 2 inversion updated to: {self.T8_inverted}")
        return SetParametersResult(successful=True)


    def cal_thrust(self,msg):
        self.tau_thrusters[0] = -self._pwm_to_force(msg.channels[0],inverted= self.T1_inverted)
        self.tau_thrusters[1] = -self._pwm_to_force(msg.channels[1],inverted=self.T2_inverted)
        self.tau_thrusters[2] = -self._pwm_to_force(msg.channels[2],inverted=self.T3_inverted)
        self.tau_thrusters[3] = -self._pwm_to_force(msg.channels[3],inverted=self.T4_inverted)
        self.tau_thrusters[4] = self._pwm_to_force(msg.channels[4],inverted=self.T5_inverted)
        self.tau_thrusters[5] = self._pwm_to_force(msg.channels[5],inverted=self.T6_inverted)
        self.tau_thrusters[6] = self._pwm_to_force(msg.channels[6],inverted=self.T7_inverted)
        self.tau_thrusters[7] = self._pwm_to_force(msg.channels[7],inverted=self.T8_inverted)

        # self.get_logger().info(f"thrust force:{self.tau_thrusters}")



    def timer_callback(self):
        dt = 0.1

        if self.i < 1000:
            self.cov = 0.01
            self.pose_cov = (np.eye(6) * self.cov).flatten().tolist()
            self.twist_cov = (np.eye(6) * self.cov).flatten().tolist()
            self.i += 1

        # Restoring forces
        phi, theta, psi = self.eta[3:6]
        g = np.zeros(6)
        g[0] = (self.W - self.B) * np.sin(theta)
        g[1] = -(self.W - self.B) * np.cos(theta)*np.sin(phi)
        g[2] = -(self.W - self.B) * np.cos(theta) * np.cos(phi)
        g[3] = self.r_cg[2] * self.W * np.cos(theta)*np.sin(phi)
        g[4] = self.r_cg[2] * self.W * np.sin(theta)
        g[5] = 0

        # Thruster generalized forces
        tau = self.Bmat @ self.tau_thrusters
        self.get_logger().info(f"thrust force: {self.tau_thrusters}")
        self.get_logger().info(f"6dof forces:{tau}")


        # Damping
        damping = self.D @ self.v + self.Dq @ (self.v * np.abs(self.v))

        # Mass matrix (simplified)
        M = np.diag([self.m, self.m, self.m, self.I[0,0], self.I[1,1], self.I[2,2]]) + self.MA

        # Accelerations
        v_dot = np.linalg.inv(M) @ (tau - damping - g)

        # Integrate
        eta_dot = self.eta_dot_from_body_vel(self.eta,self.v)
        self.eta += eta_dot * dt
        # self.eta[0:3] += eta_dot[0:3] * dt
        self.v += v_dot * dt
        

        if (self.eta[2] < 0.0):
            self.eta[2] = 0.0
            self.v[2] = 0.0
        elif(self.eta[2] > 4.6):
            self.eta[2] = 4.6
            self.v[2] = 0.0

        if (self.eta[0] < 0.0):
            self.eta[0] = 0.0
            self.v[0] = 0.0
        elif(self.eta[0] > 13.0):
            self.eta[0] = 13.0
        
        if (self.eta[2] < 0.0):
            self.eta[0] = 0.0
            self.v[0] = 0.0
        elif(self.eta[2] > 8.0):
            self.eta[0] = 8.0
            self.v[0] = 0.0

            self.v[0] = 0.0
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = float(self.eta[0])
        odom.pose.pose.position.y = float(self.eta[1])
        odom.pose.pose.position.z = float(self.eta[2])

        # Orientation (quaternion)
        roll, pitch, yaw = self.eta[3:6]
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        odom.pose.pose.orientation.w = cr*cp*cy + sr*sp*sy
        odom.pose.pose.orientation.x = sr*cp*cy - cr*sp*sy
        odom.pose.pose.orientation.y = cr*sp*cy + sr*cp*sy
        odom.pose.pose.orientation.z = cr*cp*sy - sr*sp*cy

        # Velocities
        odom.twist.twist.linear.x = float(self.v[0])
        odom.twist.twist.linear.y = float(self.v[1])
        odom.twist.twist.linear.z = float(self.v[2])
        odom.twist.twist.angular.x = float(self.v[3])
        odom.twist.twist.angular.y = float(self.v[4])
        odom.twist.twist.angular.z = float(self.v[5])

        # HIGH covariance
        odom.pose.covariance = self.pose_cov
        odom.twist.covariance = self.twist_cov

        self.pub.publish(odom)

    def _pwm_to_force(self,pwm,inverted = False):
        if inverted:
            pwm = 2 * 1500 - pwm
        if pwm > 1536:
            force = (pwm - 1500) / 9.7
        elif pwm < 1464:
            force = (pwm - 1464) /12.781
        else:
            force = 0.0
        return force



    def R_b_to_n(self,phi, theta, psi):
        cphi = np.cos(phi); sphi = np.sin(phi)
        cth  = np.cos(theta); sth = np.sin(theta)
        cpsi = np.cos(psi);   spsi = np.sin(psi)
        R = np.array([
            [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi],
            [spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi],
            [-sth,     cth*sphi,                   cth*cphi]
        ])
        return R

    def T_euler(self,phi, theta):
        cphi = np.cos(phi); sphi = np.sin(phi)
        cth  = np.cos(theta); sth = np.sin(theta)
        # guard against cos(theta) ~= 0 before using this
        T = np.array([
            [1.0, sphi * np.tan(theta),  cphi * np.tan(theta)],
            [0.0, cphi,                 -sphi],
            [0.0, sphi / cth,           cphi / cth]
        ])
        return T

    def eta_dot_from_body_vel(self,eta, v):
        # eta = [x,y,z, phi,theta,psi]
        # v   = [u,v,w, p,q,r] (body frame)
        phi, theta, psi = eta[3], eta[4], eta[5]
        R = self.R_b_to_n(phi, theta, psi)
        T = self.T_euler(phi, theta)

        vel_lin = R @ v[0:3]
        vel_ang = T @ v[3:6]
        return np.hstack((vel_lin, vel_ang))

def main():
    rclpy.init()
    node = BlueROVDeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
