#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16,Float64,String
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
import math 

class SearchingPattern(LifecycleNode):

    def __init__(self):
        super().__init__("search_pattern")
        self.get_logger().info("Initializing Bluerov2 Goal Searching Node...")

        self.declare_parameter('x_min', 1.5)
        self.declare_parameter('x_max', 7.5)
        self.declare_parameter('y_min', 1.0)
        self.declare_parameter('y_max', 7.0)
        self.declare_parameter('spacing', 1.5)
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
        self.declare_parameter('desired_depth',2.5)
        self.declare_parameter('depth_topic','desired_depth')
        self.declare_parameter('depth_status_topic','controller/depth_status')

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Runtime attributes
        self._input_sub = None
        self._output_pub = None

    # Lifecycle callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring lawmower search pattern...")

        # Load parameters
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.spacing = self.get_parameter('spacing').value
        self.rate = self.get_parameter('rate').value
        self.surge_force = self.get_parameter('surge_force').value
        self.look_ahead = self.get_parameter('look_ahead').value
        self.switching_th = self.get_parameter('switching_th').value
        input_topic = self.get_parameter('input_topic').value
        self.get_logger().info(f"input_topic:{input_topic}")
        output_yaw_topic = self.get_parameter('output_yaw_topic').value
        output_surge_topic = self.get_parameter('output_surge_topic').value
        self.enable = self.get_parameter('enable').value
        self.invert_surge = self.get_parameter('invert_surge').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.stonefish = self.get_parameter('stonefish').value
        self.depth_status_topic = self.get_parameter('depth_status_topic').value
        self.get_logger().info(f"rate:{self.rate}")

        self.desired_depth = self.get_parameter('desired_depth').value
        self.depth_topic = self.get_parameter('depth_topic').value


        self.waypoints = []
        self.i = 0
        self.lawmower_pattern()

        if len(self.waypoints) < 2:
            self.get_logger().error("Not enough waypoints generated.")
            return TransitionCallbackReturn.FAILURE

        self._output_yaw_pub = self.create_publisher(Float64, output_yaw_topic, 10)
        self._output_surge_pub = self.create_publisher(Float64, output_surge_topic, 10)
        self._output_depth_pub = self.create_publisher(Float64, self.depth_topic, 10)

        self._input_topic = input_topic

        self.current_wp_idx = 0
        self.pose = None

        self.depth_stable = False 
        self.prev_stable = False

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating search node...")

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

        self.depth_status_topic = self.create_subscription(
            String,
            self.depth_status_topic,
            self._get_depth_status,
            qos_profile=qos_profile
        )

        self.get_logger().info(f"Start diving to the desired depth: {self.desired_depth}m")
        self._timer = self.create_timer(1.0 / self.rate, self._path_follower)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating search node...")
        # Stop the control loop
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        # Optionally destroy subscriptions to avoid receiving messages
        if hasattr(self, "_input_sub") and self._input_sub is not None:
            self.destroy_subscription(self._input_sub)
            self._input_sub = None

        if hasattr(self, "depth_status_topic") and self.depth_status_topic is not None:
            self.destroy_subscription(self.depth_status_topic)
            self.depth_status_topic = None

        self.send_stop_force()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Search node...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down search node...")
        return TransitionCallbackReturn.SUCCESS

    # Parameter update callback for runtime tuning
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'x_min':
                self.x_min = p.value
                self.get_logger().info(f"x_min updated to: {self.x_min}")
           
            elif p.name == 'x_max':
                self.x_max = p.value
                self.get_logger().info(f"x_max updated to: {self.x_min}")

            elif p.name == 'y_min':
                self.y_min = p.value
                self.get_logger().info(f"y_min updated to: {self.y_min}")   
            elif p.name == 'y_max':
                self.y_max = p.value
                self.get_logger().info(f"y_max updated to: {self.y_max}")
            elif p.name == 'spacing':
                self.spacing = p.value
                self.get_logger().info(f"spacing updated to: {self.spacing}")
            elif p.name == 'rate':
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
                else:
                    self.get_logger().info(f"stonefish mode disabled")
            
            return SetParametersResult(successful=True)



    def lawmower_pattern(self):
        y = self.y_min
        direction = 1

        while y <= self.y_max:
            if direction > 0:
                self.waypoints.append((self.x_min, y))
                self.waypoints.append((self.x_max, y))
            else:
                self.waypoints.append((self.x_max, y))
                self.waypoints.append((self.x_min, y))

            y += self.spacing
            direction *= -1

        # self.get_logger().info(f"Waypoints generated: {self.waypoints}")

    def _get_depth_status(self, msg: String):
        if msg.data == "stable":
            self.depth_stable = True
        else:
            self.depth_stable = False
    def _get_robot_pose(self, msg: Odometry):
        if not self.stonefish:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
        else:
            px = 5.5 - msg.pose.pose.position.x
            py = 3.5 - msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) - self.yaw_offset
        self.pose = (px, py, yaw)


    @staticmethod
    def wrap_angle(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    # LOS guidance: compute psi_des, along-track s, segment length L, cross-track ey
    def LOS_stats(self, pos, P1, P2, delta):
        x, y = pos
        x1, y1 = P1
        x2, y2 = P2

        dx = x2 - x1
        dy = y2 - y1

        L = math.hypot(dx, dy)

        if L < 1e-9:
            return {'psi_des': 0.0, 's': 0.0, 'L': L, 'ey': 0.0, 'los_point': (x1, y1), 'alpha': 0.0}
        
        ######  path direction ######
        alpha = math.atan2(dy, dx)

        ##### compute error #####
        rx = x - x1
        ry = y - y1
        cos_a = math.cos(alpha)
        sin_a = math.sin(alpha)
        xt = cos_a * rx + sin_a * ry   # along-track
        yt = -sin_a * rx + cos_a * ry  # cross-track (signed)

        ##### LOS point #####
        x_los_along = xt + delta
        ey = yt
        # clamp LOS point to segment ends to keep psi_des well-defined
        if x_los_along < 0.0:
            x_los_along = 0.0
        if x_los_along > L:
            x_los_along = L
        x_los = x1 + x_los_along * cos_a
        y_los = y1 + x_los_along * sin_a

        #### desired heading ####
        psi_des = math.atan2(y_los - y, x_los - x)
        
        return {'psi_des': psi_des, 's': xt, 'L': L, 'ey': ey, 'los_point': (x_los, y_los), 'alpha': alpha}

    # Guidance loop executed by timer
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
            self.get_logger().info(f"Depth is stable starting search")
            self.prev_stable = True

       

        # current segment P1->P2
        P1 = self.waypoints[self.current_wp_idx]
        P2 = self.waypoints[self.current_wp_idx + 1]
        x, y, yaw = self.pose

        stats = self.LOS_stats((x, y), P1, P2, self.look_ahead)
        psi_des = stats['psi_des']
        s_along = stats['s']
        seg_len = stats['L']
        ey = stats['ey']

        # switching rule: along-track progress method (with buffer)
        if s_along >= seg_len - self.switching_th:
            # advance to next segment if exists
            if self.current_wp_idx < len(self.waypoints) - 2:
                self.current_wp_idx += 1
                # recompute LOS for the new segment (immediate)
                P1 = self.waypoints[self.current_wp_idx]
                P2 = self.waypoints[self.current_wp_idx + 1]
                stats = self.LOS_stats((x, y), P1, P2, self.look_ahead)
                psi_des = stats['psi_des']
                s_along = stats['s']
                seg_len = stats['L']
                ey = stats['ey']
                # log small summary
                self.get_logger().debug(f"Switched to segment {self.current_wp_idx}: P1={P1} P2={P2}")
            else:
                # last segment ended
                self.get_logger().info("Reached final segment end. Stopping.")
                self.send_stop_force()
                return
            

        ####### For simulation purposes only ########
        psi_des -= self.yaw_offset
        self.i += 1
        if self.i % 50 == 0:
            self.get_logger().info(f"current position x:{x}, y:{y}, yaw:{math.degrees(self.wrap_angle(yaw))}")
            self.get_logger().info(f"desired waypoint x:{P2[0]}, y:{P2[1]}, desired_yaw:{math.degrees(self.wrap_angle(psi_des))}")
            self.i = 0
        # publish desired yaw in radians
        msg = Float64()
        msg.data = float(psi_des) 
        if self._output_yaw_pub:
            self._output_yaw_pub.publish(msg)

        surge = Float64()
        if self.invert_surge:
            surge.data = -self.surge_force
        else:
            surge.data = self.surge_force

        if self._output_surge_pub:
            self._output_surge_pub.publish(surge)

        # optional debug logging (throttled)
        # note: don't spam logs in high-rate loops
        self.get_logger().debug(
            f"wp_idx={self.current_wp_idx} pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg psi_des={math.degrees(psi_des):.1f}deg"
        )
    def send_stop_force(self):
        force = Float64()
        force.data = 0.0
        self._output_surge_pub.publish(force)

def main(args=None):
    rclpy.init(args=args)
    node = SearchingPattern()
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
