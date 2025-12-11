#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix, quaternion_from_euler

import numpy as np


class CameraPoseFromMarkers(Node):
    def __init__(self):
        super().__init__('camera_pose_from_markers')

        self.subscription = self.create_subscription(
            PoseArray,
            'aruco_poses',       # pose of markers in CAMERA frame
            self.pose_callback,
            10
        )

        # Publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.odom_pub = self.create_publisher(Odometry, 'aruco_odom', 10)
        self.odom_pub = self.create_publisher(PoseWithCovarianceStamped, 'aruco_odom', 10)


        # TF utilities
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Smoothing variables
        self.last_pos = None
        self.last_quat = None
        self.alpha_pos = 0.25
        self.alpha_rot = 0.35

        self.missed_frames = 0
        self.reset_after = 5

        self.get_logger().info("Camera odometry estimation node started.")

    def smooth_position(self, raw_pos):
        return self.alpha_pos * raw_pos + (1 - self.alpha_pos) * self.last_pos

    def smooth_quaternion(self, raw_q):
        q = self.alpha_rot * raw_q + (1 - self.alpha_rot) * self.last_quat
        return q / np.linalg.norm(q)

    def pose_callback(self, msg: PoseArray):

        cam_positions = []
        cam_rotations = []

        for i, pose in enumerate(msg.poses):

            if pose.position.x == -999.99:
                continue

            # Camera → Marker transform
            T_cam_marker = quaternion_matrix([
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w
            ])
            T_cam_marker[:3, 3] = [
                pose.position.x, pose.position.y, pose.position.z
            ]

            T_marker_cam = np.linalg.inv(T_cam_marker)

            marker_frame = f"aruco_{i}"

            try:
                t = self.tf_buffer.lookup_transform(
                    "ned", marker_frame, rclpy.time.Time()
                )

                T_ned_marker = quaternion_matrix([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ])
                T_ned_marker[:3, 3] = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ]

                T_ned_cam = T_ned_marker @ T_marker_cam

                cam_positions.append(T_ned_cam[:3, 3])
                cam_rotations.append(quaternion_from_matrix(T_ned_cam))

            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {marker_frame}: {e}")
                continue

        if len(cam_positions) == 0:
            self.missed_frames += 1

            if self.missed_frames > self.reset_after:
                self.last_pos = None
                self.last_quat = None
            return

        self.missed_frames = 0

        raw_pos = np.mean(cam_positions, axis=0)
        raw_quat = np.mean(cam_rotations, axis=0)
        raw_quat /= np.linalg.norm(raw_quat)

        if self.last_pos is None:
            cam_pos = raw_pos
            cam_quat = raw_quat
        else:
            cam_pos = self.smooth_position(raw_pos)
            cam_quat = self.smooth_quaternion(raw_quat)

        self.last_pos = cam_pos
        self.last_quat = cam_quat

        static_translation = [0.0, 0.0, -0.2]
        static_euler = [-1.57, -1.57, 0.0]  # roll, pitch, yaw

        # Convert euler  to quaternion
        static_quat = quaternion_from_euler(static_euler[0], static_euler[1], static_euler[2])

        # Create homogeneous transformation matrix
        T_cam_bl = quaternion_matrix(static_quat)
        T_cam_bl[:3, 3] = static_translation

        # Transform camera pose in NED to base_link
        T_ned_bl = T_ned_cam @ T_cam_bl

        # Extract translation and rotation
        base_pos = T_ned_bl[:3, 3]
        base_quat = quaternion_from_matrix(T_ned_bl)
        base_quat /= np.linalg.norm(base_quat)

        # try:
        #     # Lookup camera -> base_link transform
        #     t_cam_bl = self.tf_buffer.lookup_transform(
        #         "base_link", "camera", rclpy.time.Time()
        #     )

        #     # Convert to homogeneous matrix
        #     T_cam_bl = quaternion_matrix([
        #         t_cam_bl.transform.rotation.x,
        #         t_cam_bl.transform.rotation.y,
        #         t_cam_bl.transform.rotation.z,
        #         t_cam_bl.transform.rotation.w
        #     ])
        #     T_cam_bl[:3, 3] = [
        #         t_cam_bl.transform.translation.x,
        #         t_cam_bl.transform.translation.y,
        #         t_cam_bl.transform.translation.z
        #     ]

        #     # Transform camera pose in NED to base_link
        #     T_ned_bl = T_ned_cam @ T_cam_bl

        #     # Extract translation and rotation
        #     base_pos = T_ned_bl[:3, 3]
        #     base_quat = quaternion_from_matrix(T_ned_bl)
        #     base_quat /= np.linalg.norm(base_quat)

        # except Exception as e:
        #     self.get_logger().warn(f"Camera->base_link TF lookup failed: {e}")
        #     return


        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "ned"  # world frame

        # Position
        pose_msg.pose.pose.position.x = float(base_pos[0])
        pose_msg.pose.pose.position.y = float(base_pos[1])
        pose_msg.pose.pose.position.z = float(base_pos[2])

        # Orientation
        pose_msg.pose.pose.orientation.x = float(base_quat[0])
        pose_msg.pose.pose.orientation.y = float(base_quat[1])
        pose_msg.pose.pose.orientation.z = float(base_quat[2])
        pose_msg.pose.pose.orientation.w = float(base_quat[3])

        # Covariance (keep same as before)
        pose_msg.pose.covariance = [
            0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 100.0
        ]

        self.odom_pub.publish(pose_msg)




        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = "camera"

        tf_msg.transform.translation.x = cam_pos[0]
        tf_msg.transform.translation.y = cam_pos[1]
        tf_msg.transform.translation.z = cam_pos[2]
        tf_msg.transform.rotation.x = cam_quat[0]
        tf_msg.transform.rotation.y = cam_quat[1]
        tf_msg.transform.rotation.z = cam_quat[2]
        tf_msg.transform.rotation.w = cam_quat[3]

        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = CameraPoseFromMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
