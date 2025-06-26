import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from mpc_module import P, Node as MpcNode, PATH, calc_ref_trajectory_in_T_step, linear_mpc_control
import math

class Controller(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # --------- Declare and retrieve ROS parameters (fully parameterized for launch integration) ---------
        self.declare_parameter("path_topic", "path")
        self.declare_parameter("cmd_topic", "cmd")
        self.declare_parameter("odom_topic", "odom")  # If not set in launch, default is "odom"

        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        # --------- ROS subscriptions and publishers ---------
        self.create_subscription(PoseArray, path_topic, self.on_path, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)

        # --------- MPC internal buffers ---------
        self.ref_path = None   # Reference path object
        self.sp = None         # Speed profile
        self.vehicle_state = None   # Current vehicle state [x, y, v, yaw]
        self.a_opt = None      # Warm start cache
        self.delta_opt = None

        # Main control loop, 20 Hz
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(f"[MPC Controller] Started with path_topic: {path_topic}, odom_topic: {odom_topic}, cmd_topic: {cmd_topic}")

    # --------- Callback: path subscription (from planner) ---------
    def on_path(self, msg):
        cx, cy, cyaw = [], [], []
        for pose in msg.poses:
            cx.append(pose.position.x)
            cy.append(pose.position.y)
            # Convert quaternion to yaw (Euler angle)
            q = pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            cyaw.append(yaw)
        if len(cx) < 2:
            self.get_logger().warn("Received path too short for MPC control.")
            return
        ck = [0.0] * len(cx)  # Curvature (not used, but required by PATH class)
        self.ref_path = PATH(cx, cy, cyaw, ck)
        self.sp = [P.target_speed] * len(cx)  # Constant speed profile; can be adapted as needed

    # --------- Callback: odometry/localization subscription ---------
    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.hypot(vx, vy)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vehicle_state = [x, y, v, yaw]

    # --------- Main timer callback (core MPC loop) ---------
    def timer_callback(self):
        # Only run MPC if both path and vehicle state are available
        if self.ref_path is None or self.vehicle_state is None:
            return

        node = MpcNode(*self.vehicle_state)
        z_ref, _ = calc_ref_trajectory_in_T_step(node, self.ref_path, self.sp)
        # Warm start support for improved optimization speed
        a_opt, delta_opt, *_ = linear_mpc_control(z_ref, self.vehicle_state, self.a_opt, self.delta_opt)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(delta_opt[0])
        msg.drive.speed = float(self.vehicle_state[2] + a_opt[0] * P.dt)
        self.publisher.publish(msg)

        # Update warm start cache
        self.a_opt = a_opt
        self.delta_opt = delta_opt

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
