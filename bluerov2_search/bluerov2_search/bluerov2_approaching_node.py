#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
import math 

class Boxapproaching(LifecycleNode):

    def __init__(self):
        super().__init__("box_approaching")
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('surge_force',1.5)
        self.declare_parameter('look_ahead',0.5)
        self.declare_parameter('switching_th', 0.5)
        self.declare_parameter('enable', True)
        self.declare_parameter('input_topic', 'input')
        self.declare_parameter('output_yaw_topic', 'desired_yaw')
        self.declare_parameter('output_surge_topic', 'searching/surge_force')
        self.declare_parameter('invert_surge', False)
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('stonefish', False)
        self.declare_parameter('desired_depth',4.0)
        self.declare_parameter('depth_topic','desired_depth')
        self.declare_parameter('depth_status_topic','controller/depth_status')
        self.declare_parameter('area_redius',1.0)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._input_sub = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring Box Approaching Node...")
        self.rate = self.get_parameter('rate').value
        self.surge_force = self.get_parameter('surge_force').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.switching_th = self.get_parameter('switching_th').value
        self.enable = self.get_parameter('enable').value
        self.invert_surge = self.get_parameter('invert_surge').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.stonefish = self.get_parameter('stonefish').value
        self.depth_status_topic_name = self.get_parameter('depth_status_topic').value
        self.area_raduis = self.get_parameter('area_redius').value
        self.desired_depth = self.get_parameter('desired_depth').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self._input_topic = self.get_parameter('input_topic').value
        output_yaw_topic = self.get_parameter('output_yaw_topic').value
        output_surge_topic = self.get_parameter('output_surge_topic').value

        self.box_pose = [0.85, 1.077]
        if self.stonefish:
            self.box_pose[0] = 5.5 - self.box_pose[0]
            self.box_pose[1] = 3.5 - self.box_pose[1]

        self.start_pose = None
        self.waypoints = []
        self.pose = None
        self.current_wp_idx = 0
        self.depth_stable = False
        self.prev_stable = False
        self.i = 0

        self._output_yaw_pub = self.create_publisher(Float64, output_yaw_topic, 10)
        self._output_surge_pub = self.create_publisher(Float64, output_surge_topic, 10)
        self._output_depth_pub = self.create_publisher(Float64, self.depth_topic, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Box Approaching Node...")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._input_sub = self.create_subscription(
            Odometry,
            self._input_topic,
            self._get_robot_pose,
            qos_profile=qos_profile
        )
        self.depth_status_sub = self.create_subscription(
            String,
            self.depth_status_topic_name,
            self._get_depth_status,
            qos_profile=qos_profile
        )
        self._timer = self.create_timer(1.0 / self.rate, self._path_follower)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Box Approaching Node...")
        if hasattr(self, "_timer") and self._timer:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if hasattr(self, "_input_sub") and self._input_sub:
            self.destroy_subscription(self._input_sub)
            self._input_sub = None
        if hasattr(self, "depth_status_sub") and self.depth_status_sub:
            self.destroy_subscription(self.depth_status_sub)
            self.depth_status_sub = None
        self.send_stop_force()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Box Approaching Node...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down Box Approaching Node...")
        return TransitionCallbackReturn.SUCCESS

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'rate':
                self.rate = p.value
                self.get_logger().info(f"rate updated to: {self.rate}") 
            elif p.name == 'surge_force': 
                self.surge_force = p.value 
                self.get_logger().info(f"surge_force updated to: {self.surge_force}")
            elif p.name == 'look_ahead':
                self.look_ahead = p.value
                self.get_logger().info(f"look_ahead updated to: {self.look_ahead}")
            elif p.name == 'switching_th': 
                self.switching_th = p.value
                self.get_logger().info(f"switching_th updated to: {self.switching_th}") 
            elif p.name == 'yaw_offset': 
                self.yaw_offset = p.value 
                self.get_logger().info(f"yaw_offset updated to: {self.yaw_offset}") 
                
            elif p.name == 'desired_depth':
                self.desired_depth = p.value 
                self.get_logger().info(f"desired_depth updated to: {self.desired_depth}") 
            elif p.name == 'enable': 
                self.enable = p.value 
                if self.enable:
                    self.get_logger().info("searching path enabled") 
                else: 
                    self.get_logger().info("searching path disabled") 
            elif p.name == 'invert_surge': 
                self.invert_surge = p.value 
                if self.invert_surge: 
                    self.get_logger().info(f"surge force inverted") 
                else: 
                    self.get_logger().info(f"surge force normal") 
            elif p.name == 'stonefish': 
                self.stonefish = p.value 
                if self.stonefish: 
                    self.get_logger().info(f"stonefish mode enabled") 
                else: self.get_logger().info(f"stonefish mode disabled") 
            elif p.name == 'area_redius':
                self.area_raduis = p.value 
                self.get_logger().info(f"area_raduis updated to: {self.area_raduis}") 
        return SetParametersResult(successful=True)

    def _get_depth_status(self, msg: String):
        self.depth_stable = (msg.data == "stable")

    def _get_robot_pose(self, msg: Odometry):
        if not self.stonefish:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
        else:
            px = 5.5 - msg.pose.pose.position.x
            py = 3.5 - msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) - self.yaw_offset
        self.pose = (px, py, yaw)

        if self.start_pose is None:
            self.start_pose = (px, py)
            self._compute_target_waypoint()

    def _compute_target_waypoint(self):
        x0, y0 = self.start_pose
        bx, by = self.box_pose
        R = self.area_raduis

        dx = bx - x0
        dy = by - y0
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return

        dx /= L
        dy /= L

        ix = bx - dx * R
        iy = by - dy * R

        self.waypoints = [(x0, y0), (ix, iy)]
        self.get_logger().info(f"Waypoints set to: {self.waypoints}")

    @staticmethod
    def wrap_angle(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def LOS_stats(self, pos, P1, P2, delta):
        x, y = pos
        x1, y1 = P1
        x2, y2 = P2

        dx = x2 - x1
        dy = y2 - y1
        L = math.hypot(dx, dy)
        if L < 1e-9:
            return {'psi_des': 0.0, 's': 0.0, 'L': L, 'ey': 0.0, 'los_point': (x1, y1), 'alpha': 0.0}

        alpha = math.atan2(dy, dx)
        rx = x - x1
        ry = y - y1
        cos_a = math.cos(alpha)
        sin_a = math.sin(alpha)
        xt = cos_a * rx + sin_a * ry
        yt = -sin_a * rx + cos_a * ry

        x_los = xt + delta
        if x_los < 0.0: x_los = 0.0
        if x_los > L: x_los = L
        x_l = x1 + x_los * cos_a
        y_l = y1 + x_los * sin_a

        psi_des = math.atan2(y_l - y, x_l - x)
        return {'psi_des': psi_des, 's': xt, 'L': L, 'ey': yt, 'los_point': (x_l, y_l), 'alpha': alpha}

    def _path_follower(self):
        depth_msg = Float64()
        depth_msg.data = float(self.desired_depth)
        self._output_depth_pub.publish(depth_msg)

        if not self.enable or not self.depth_stable:
            return

        if self.pose is None:
            # no pose yet
            return

        # if finished all waypoints, stop
        if self.current_wp_idx >= len(self.waypoints) - 1:
            self.get_logger().info("Completed all waypoints. Stopping.")
            self.send_stop_force()
            return


        if self.depth_stable and not self.prev_stable:
            self.get_logger().info(f"Depth is stable starting aproaching box")
            self.prev_stable = True

        x, y, yaw = self.pose
        P1 = self.waypoints[self.current_wp_idx]
        P2 = self.waypoints[self.current_wp_idx + 1]

        stats = self.LOS_stats((x, y), P1, P2, self.look_ahead)
        psi_des = stats['psi_des']
        s_along = stats['s']
        seg_len = stats['L']

        if s_along >= seg_len - self.switching_th:
            if self.current_wp_idx < len(self.waypoints) - 2:
                self.current_wp_idx += 1
                P1 = self.waypoints[self.current_wp_idx]
                P2 = self.waypoints[self.current_wp_idx + 1]
                stats = self.LOS_stats((x, y), P1, P2, self.look_ahead)
                psi_des = stats['psi_des']
            else:
                self.send_stop_force()
                return
            
        psi_des -= self.yaw_offset

        psi_msg = Float64()
        psi_msg.data = float(psi_des)
        self._output_yaw_pub.publish(psi_msg)

        surge = Float64()
        surge.data = -self.surge_force if self.invert_surge else self.surge_force
        self._output_surge_pub.publish(surge)

    def send_stop_force(self):
        force = Float64()
        force.data = 0.0
        self._output_surge_pub.publish(force)

def main(args=None):
    rclpy.init(args=args)
    node = Boxapproaching()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
