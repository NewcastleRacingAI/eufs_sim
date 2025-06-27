import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from newcastle_racing_ai_msgs.msg import ControlCommand
from .parameters import PARAMETERS


class Controller(Node):

    def __init__(self):
        super().__init__("Controller")
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        #####################
        # Subscribers
        #####################
        self._subscription = self.create_subscription(
            PoseArray, self.get_parameter("path_topic").value, self._on_path, 10
        )
        #####################
        # Publishers
        #####################
        self._publisher = self.create_publisher(ControlCommand, self.get_parameter("control_topic").value, 10)
        #####################
        # Timer
        #####################
        self.timer = self.create_timer(self.get_parameter("time_step").value, self._timer_callback)

    def _on_path(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))

    def _timer_callback(self):
        msg = ControlCommand()
        msg.throttle = 0.0  # Example throttle value. Between (0.0, 1.0)
        msg.steering = 0.0  # Example steering value. Between (-1.0, 1.0)
        msg.brake = 0.0  # Example brake value. Between (0.0, 1.0)
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Controller()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
