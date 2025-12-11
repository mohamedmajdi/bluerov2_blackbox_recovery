import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImuFrameRepublisher(Node):
    def __init__(self):
        super().__init__('imu_frame_republisher')

        # 1. Configuration
        # We rename the frame to this value. 
        # This allows us to create a Static Transform between base_link -> imu_link
        # to fix the rotation issues.
        self.target_frame = 'imu_link'
        
        # QoS Settings: Best Effort is usually required for IMU data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Subscriber: Listens to the original raw data
        # Even if this data says frame_id="base_link", we will capture it here.
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile
        )

        # 3. Publisher: Sends out the fixed message with the new frame_id
        self.publisher = self.create_publisher(
            Imu,
            'imu/data_republished',
            qos_profile
        )

        self.get_logger().info(f'Republisher started. Renaming input frame to: {self.target_frame}')

    def imu_callback(self, msg):
        # Create a new message to avoid modifying the original pointer
        new_msg = Imu()
        
        # Copy all sensor data from the original message
        new_msg.header = msg.header
        new_msg.orientation = msg.orientation
        new_msg.orientation.y = -msg.orientation.y
        new_msg.orientation.z = -msg.orientation.z
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity.y = -msg.angular_velocity.y    
        new_msg.angular_velocity.z = msg.angular_velocity.z
        # new_msg.angular_velocity.z = -msg.angular_velocity.z
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration = msg.linear_acceleration

        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        # CRITICAL FIX: Overwrite the frame_id
        # The original msg.header.frame_id might be "base_link", but we overwrite it
        # to "imu_link" so the EKF uses our specific static transform.
        new_msg.header.frame_id = self.target_frame
        
        # Publish the fixed message
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()