#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt16,Float64, String
import numpy as np
from bluerov2_util.PID import PID  # your reusable PID class
from rcl_interfaces.msg import SetParametersResult

class Bluerov2DepthControl(LifecycleNode):

    def __init__(self):
        super().__init__("depth_controller")
        self.get_logger().info("Initializing Bluerov2 Depth Control Node...")

        # Declare parameters (flattened dot notation for nested PID)
        self.declare_parameter('pid.kp', 0.0)
        self.declare_parameter('pid.ki', 0.0)
        self.declare_parameter('pid.kd', 0.0)
        self.declare_parameter('target_depth', 0.0)
        self.declare_parameter('flotability', 0.0)
        self.declare_parameter('input_topic', 'input')
        self.declare_parameter('setpoint_topic', 'setpoint')
        self.declare_parameter('output_topic', 'controller/heave_force')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('pid.i_limit',0.0)
        self.declare_parameter('enable', True)
        self.declare_parameter("hold",False)
        self.declare_parameter("inverted",False)
        self.declare_parameter("status_topic","controller/depth_status")
        self.declare_parameter("depth_th",0.1)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Runtime attributes
        self._pid = None
        self.goal_depth = 0.0
        self._flotability = 0.0
        self._current_depth = 0.0
        self._input_sub = None
        self._output_pub = None

    # Lifecycle callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring Depth Controller...")

        # Load parameters
        kp = self.get_parameter('pid.kp').value
        ki = self.get_parameter('pid.ki').value
        kd = self.get_parameter('pid.kd').value
        i_limit = self.get_parameter('pid.i_limit').value
        self.goal_depth = self.get_parameter('target_depth').value
        self.flotability = self.get_parameter('flotability').value
        self.get_logger().info(f"flotability:{self.flotability}")
        input_topic = self.get_parameter('input_topic').value
        self.get_logger().info(f"input_topic:{input_topic}")
        output_topic = self.get_parameter('output_topic').value
        setpoint_topic = self.get_parameter('setpoint_topic').value
        self.rate = self.get_parameter('control_rate').value
        self.hold = self.get_parameter('hold').value
        self.enable = self.get_parameter('enable').value
        self.get_logger().info(f"rate:{self.rate}")
        self.inverted = self.get_parameter('inverted').value
        self._pid = PID(kp, ki, kd, i_limit)  # optional limits
        self.depth_status_topoic = self.get_parameter("status_topic").value
        self.get_logger().info(f"output topic: {output_topic}")
        self.depth_th = self.get_parameter("depth_th").value

        self._output_pub = self.create_publisher(Float64, output_topic, 10)
        self._output_status_pub = self.create_publisher(String, self.depth_status_topoic, 10)
        self._input_topic = input_topic
        self.setpoint_topic = setpoint_topic
        
        self.init_control = True
        self.target = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Depth Controller...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._input_sub = self.create_subscription(
            Float64,
            self._input_topic,
            self._on_depth_measurement,
            qos_profile=qos_profile
        )

        self.setpoint_sub = self.create_subscription(Float64,
                            self.setpoint_topic,
                            self.get_target_depth,
                            qos_profile=qos_profile)
        # Control timer
        self._pid.reset()
        self.init_control = True
        self._timer = self.create_timer(1.0 / self.rate, self._control_loop)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Depth Controller...")
        # Stop the control loop
        self.send_stop_force()
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

        # Optionally destroy subscriptions to avoid receiving messages
        if hasattr(self, "_input_sub") and self._input_sub is not None:
            self.destroy_subscription(self._input_sub)
            self._input_sub = None

        if hasattr(self, "setpoint_sub") and self.setpoint_sub is not None:
            self.destroy_subscription(self.setpoint_sub)
            self.setpoint_sub = None

        
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up Depth Controller...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down Depth Controller...")
        return TransitionCallbackReturn.SUCCESS

    # Parameter update callback for runtime tuning
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'pid.kp':
                self._pid.kp = p.value
                self.get_logger().info(f"kp updated to: {self._pid.kp}")
            elif p.name == 'pid.ki':
                self._pid.ki = p.value
                self.get_logger().info(f"ki updated to: {self._pid.ki}")
            elif p.name == 'pid.kd':
                self._pid.kd = p.value
                self.get_logger().info(f"kd updated to: {self._pid.kd}")
            elif p.name == 'pid.i_limit':
                self.get_logger().info(f"integration limit updated to: {self._pid.i_limit}")
                self._pid.i_limit = p.value
            elif p.name == 'hold':
                self.hold = p.value
                self.get_logger().info(f"hold mode updated: {self.hold}")
            elif p.name == 'flotability':
                self.flotability = p.value
                self.get_logger().info(f"flotability changed: {self.flotability}")
            elif p.name == 'target_depth':
                self.goal_depth = p.value
                self.get_logger().info(f"goal depth changed: {self.goal_depth}")
            elif p.name == 'enable':
                self.enable = p.value
                if self.enable:
                    self.get_logger().info("Depth controller enabled")
                else:
                    self.get_logger().info("Depth controller disabled")
            elif p.name == 'inverted':
                self.inverted = p.value
                if self.inverted:
                    self.get_logger().info(f"heave thrust inverted")
                else:
                    self.get_logger().info(f"heave thrust normal")

            elif p.name == 'depth_th':
                self.depth_th = p.value
                self.get_logger().info(f"depth threshold updated to: {self.depth_th}")
            return SetParametersResult(successful=True)


    # Subscriber callback
    def get_target_depth(self,msg):
        self.goal_depth = msg.data
    def _on_depth_measurement(self, msg):
        self._current_depth = -msg.data

    # Timer callback: computes PID output and publishes RC override
    def _control_loop(self):
        if self._pid is None or not self.enable:
            self.send_stop_force()
            return
        
        if self.hold:
            if self.init_control:
                self.init_control = False
                self.target = self._current_depth
        else:
            self.target = self.goal_depth

        out,dic = self._pid.update(self.target, self._current_depth,feedforward=self.flotability/10) 

        if abs(self._current_depth - self.target) < self.depth_th:
            self.status = "stable"
        else:
            self.status = "diving"

        status_msg = String()
        status_msg.data = self.status
        self._output_status_pub.publish(status_msg)

        if self.inverted:
            force = out
        else:
            force = -out
        # self.get_logger().info(f"Target: {self.target}, current depth: {self._current_depth}")
        # self.get_logger().info(f"PID debug: {dic}")

        msg = Float64()
        msg.data = force
        self._output_pub.publish(msg)
    def send_stop_force(self):
        f = 0.0
        msg_force = Float64()
        msg_force.data= f
        self._output_pub.publish(msg_force)


def main(args=None):
    rclpy.init(args=args)
    node = Bluerov2DepthControl()
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
