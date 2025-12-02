import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from bluerov2_interface.msg import Detection  # your custom message


class PixelToPosePublisher(Node):
    def __init__(self):
        super().__init__('pixel_to_pose_ned_publisher')

        # Parameters
        self.declare_parameter(
            'calibration_file',
            '/home/mahmoud/bluerov_ws/src/bluerov2_localization/param/stonefish_camera_only.npz'
        )
        self.declare_parameter('object_width', 0.3)
        self.declare_parameter('object_height', 0.15)
        self.declare_parameter('camera_frame', 'bluerov/zed_camera_link')
        self.declare_parameter('world_frame', 'world_ned')  # NED/world frame for published poses

        # Load camera calibration
        calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        data = np.load(calib_file)
        self.camera_matrix = data['camera_matrix']
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        self.object_width = self.get_parameter('object_width').get_parameter_value().double_value
        self.object_height = self.get_parameter('object_height').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.subscription = self.create_subscription(
            Detection,
            '/bluerov2/detections',
            self.detection_callback,
            10
        )

        # Publishers for PoseStamped
        self.box_pub = self.create_publisher(PoseStamped, 'box_pose', 10)
        self.handle_pub = self.create_publisher(PoseStamped, 'handle_pose', 10)

    def detection_callback(self, msg):
        # Lookup transform from camera -> world frame
        try:
            t_cam2world = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time().to_msg()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Helper: transform a point in camera frame to PoseStamped in world frame
        def transform_to_world(x, y, z):
            p_camera = PoseStamped()
            p_camera.header.frame_id = self.camera_frame
            p_camera.header.stamp = self.get_clock().now().to_msg()
            p_camera.pose.position.x = x
            p_camera.pose.position.y = y
            p_camera.pose.position.z = z
            p_camera.pose.orientation.x = 0.0
            p_camera.pose.orientation.y = 0.0
            p_camera.pose.orientation.z = 0.0
            p_camera.pose.orientation.w = 1.0
            transformed_pose = do_transform_pose(p_camera.pose, t_cam2world)

            # Create output PoseStamped
            pose_world = PoseStamped()
            pose_world.header.frame_id = self.world_frame
            pose_world.header.stamp = self.get_clock().now().to_msg()
            pose_world.pose = transformed_pose

            return pose_world

        # Box detection
        if msg.blackbox_xcenter != -1:
            x, y, z = self.pixel_to_meter(
                msg.blackbox_xmin, msg.blackbox_xmax,
                msg.blackbox_ymin, msg.blackbox_ymax
            )
            self.publish_tf('box', z, -y, -x)
            pose_world = transform_to_world(z, -y, -x)
            self.box_pub.publish(pose_world)
            self.get_logger().info(f'Box Pose (NED): {pose_world.pose.position}')

        # Handle detection
        if msg.handle_xcenter != -1:
            x, y, z = self.pixel_to_meter(
                msg.handle_xmin, msg.handle_xmax,
                msg.handle_ymin, msg.handle_ymax
            )
            self.publish_tf('handle', z, -y, -x)
            pose_world = transform_to_world(z, -y, -x)
            self.handle_pub.publish(pose_world)
            self.get_logger().info(f'Handle Pose (NED): {pose_world.pose.position}')

    def pixel_to_meter(self, xmin, xmax, ymin, ymax):
        # Convert bounding box pixels to 3D coordinates in camera frame
        pixel_width = xmax - xmin
        pixel_height = ymax - ymin
        x_center = (xmax + xmin) / 2
        y_center = (ymax + ymin) / 2

        Zx = self.fx * self.object_width / pixel_width
        Zy = self.fy * self.object_height / pixel_height
        Z = (Zx + Zy) / 2

        X = (x_center - self.cx) * Z / self.fx
        Y = (y_center - self.cy) * Z / self.fy

        return X, Y, Z

    def publish_tf(self, child_frame, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PixelToPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
