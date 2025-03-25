import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from .parameters import PARAMETERS

class Perception(Node):

    def __init__(self):
        super().__init__('Perception')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(ColorRGBA, self.get_parameter("camera_topic").value, self._on_camera, 10)
        self._publisher = self.create_publisher(ConeArrayWithCovariance, self.get_parameter("cones_topic").value, 10)
        # self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_camera(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))

    def _timer_callback(self):
        msg = ConeArrayWithCovariance(
        blue_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
        yellow_cones=[],
        orange_cones=[],
        big_orange_cones=[],
        unknown_color_cones=[ConeWithCovariance(point=Point(x=0.0, y=0.0, z=0.0), covariance=[0.0, 0.0, 0.0, 0.0])],
        )
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Perception()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()