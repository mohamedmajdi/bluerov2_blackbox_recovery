#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('marker_length', 0.3)  # meters
        self.declare_parameter('calibration_file', 'bluerov2_localization/param/camera_calibration_11_11.npz')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        # Load camera calibration
        data = np.load(self.calib_file)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        # ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters() 

        # CV bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Publisher for detected marker poses
        self.pose_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.min_area = 500
        self.get_logger().info(f"Aruco detector node started, listening to {self.camera_topic}")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Prepare PoseArray
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = [Pose() for _ in range(9)]  # always 9 markers (IDs 0–8)

        # Initialize all poses with -999.99 (marker not found)
        for pose in pose_array.poses:
            pose.position.x = pose.position.y = pose.position.z = -999.99
            pose.orientation.x = pose.orientation.y = pose.orientation.z = pose.orientation.w = -999.99

        if ids is not None:
            valid_indices = [i for i, marker_id in enumerate(ids.flatten()) if 0 <= marker_id <= 8]

            if valid_indices:
                filtered_corners = []
                filtered_ids = []

                for i in valid_indices:

                    pts = corners[i][0]  # (4,2)

                    area = cv2.contourArea(pts)

                    if area < self.min_area:
                        continue
                    filtered_corners.append(corners[i])
                    filtered_ids.append(ids[i])

                    
                # Estimate pose of detected markers
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    filtered_corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                )

                for i, marker_id in enumerate(filtered_ids):
                    marker_id = int(marker_id)
                    if 0 <= marker_id <= 8:
                        # Get translation and rotation
                        tvec = tvecs[i][0]
                        rvec = rvecs[i][0]

                        # if tvec[2] < 0.0:
                        #     tvec[1] = -tvec[1]
                        #     tvec[2] = -tvec[2]

                        rot_matrix, _ = cv2.Rodrigues(rvec)
                        quat = self.rotation_matrix_to_quaternion(rot_matrix)

                        # Fill pose for the corresponding marker ID
                        pose_array.poses[marker_id].position.x = float(tvec[0])
                        pose_array.poses[marker_id].position.y = float(tvec[1])
                        pose_array.poses[marker_id].position.z = float(tvec[2])
                        pose_array.poses[marker_id].orientation.x = quat[0]
                        pose_array.poses[marker_id].orientation.y = quat[1]
                        pose_array.poses[marker_id].orientation.z = quat[2]
                        pose_array.poses[marker_id].orientation.w = quat[3]

                        # Draw for visualization
                        rvec = rvec.reshape((3,1))
                        tvec = tvec.reshape((3,1))
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length/2)

                # Draw all detected markers
                cv2.aruco.drawDetectedMarkers(cv_image, filtered_corners, np.array(filtered_ids))

        # Publish PoseArray (always 9 poses)
        self.pose_pub.publish(pose_array)

        # Optional visualization
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert rotation matrix to quaternion (x, y, z, w)"""
        q = np.empty((4,))
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(1.0 + t) * 2
            q[3] = 0.25 * t
            q[0] = (R[2, 1] - R[1, 2]) / t
            q[1] = (R[0, 2] - R[2, 0]) / t
            q[2] = (R[1, 0] - R[0, 1]) / t
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                t = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[3] = (R[2, 1] - R[1, 2]) / t
                q[0] = 0.25 * t
                q[1] = (R[0, 1] + R[1, 0]) / t
                q[2] = (R[0, 2] + R[2, 0]) / t
            elif R[1, 1] > R[2, 2]:
                t = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[3] = (R[0, 2] - R[2, 0]) / t
                q[0] = (R[0, 1] + R[1, 0]) / t
                q[1] = 0.25 * t
                q[2] = (R[1, 2] + R[2, 1]) / t
            else:
                t = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[3] = (R[1, 0] - R[0, 1]) / t
                q[0] = (R[0, 2] + R[2, 0]) / t
                q[1] = (R[1, 2] + R[2, 1]) / t
                q[2] = 0.25 * t
        return q.tolist()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
