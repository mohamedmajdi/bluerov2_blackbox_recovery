#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from tf_transformations import quaternion_matrix

class ImuLocalization(Node):

    def __init__(self):
        super().__init__("imu_localization")
        self.prev_time = None
        self.sim = False
        if self.sim:
            topic_name = "/bluerov/imu/data"
        else:
            topic_name = "/bluerov2/imu/data"

        self.get_logger().info(f"localization node started subscribing to {topic_name}")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_imu = self.create_subscription(Imu, topic_name, self.update_position, qos_profile=qos_profile)
        self.pub_odom = self.create_publisher(Odometry, 'imu_odom', 10)

        self.pos = np.array([0.2, 7.6, 0.0]) 
        self.vel = np.zeros(3)
        self.vel_threshold = 0.01

    def update_position(self,msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            return
        
        dt = now - self.prev_time
        self.prev_time = now

        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        T = quaternion_matrix(quat)
        R = T[:3, :3]
        accel_body = np.array([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])
        if not self.sim:
            g = np.array([0, 0, 9.81])
        else:
            g= np.zeros(3)
        accel_world = R @ accel_body - g
        self.vel += accel_world * dt

        if np.linalg.norm(self.vel) < self.vel_threshold:
            self.get_logger().info("Ignoring update velocity below threshold")
        else:
            self.pos += self.vel * dt + 0.5 * accel_world * dt**2

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'base_link'
        odom_msg.child_frame_id = 'imu_link'

        # Pose
        odom_msg.pose.pose.position = Point(x=float(self.pos[0]), y=float(self.pos[1]), z=float(self.pos[2]))
        odom_msg.pose.pose.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

        # Twist (velocity)
        odom_msg.twist.twist.linear = Vector3(x=float(self.vel[0]), y=float(self.vel[1]), z=float(self.vel[2]))

        self.pub_odom.publish(odom_msg)
        self.get_logger().info(f"velocity: {np.linalg.norm(self.vel)}")

        self.get_logger().info(f"Odom → pos=({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f})")


def main(args=None):
    rclpy.init(args=args)

    imu_localization = ImuLocalization()

    try:
        rclpy.spin(imu_localization)
    except KeyboardInterrupt:
        pass

    imu_localization.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
