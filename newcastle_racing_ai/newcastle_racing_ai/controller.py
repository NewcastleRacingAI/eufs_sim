import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from .parameters import PARAMETERS

class Controller(Node):
    """A value that means `do nothing`"""

    __null_ads = AckermannDriveStamped(
        drive=AckermannDrive(
            steering_angle=0.0,
            steering_angle_velocity=0.0,
            speed=0.0,
            acceleration=0.0,
            jerk=0.0,
        )
    )

    def __init__(self):
        super().__init__('Controller')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        self._subscription = self.create_subscription(PoseArray, self.get_parameter("path_topic").value, self._on_path, 10)
        self._publisher = self.create_publisher(AckermannDriveStamped, self.get_parameter("cmd_topic").value, 10)
        # self.timer = self.create_timer(timer_period, self._timer_callback)

        self._steering_angle = 0.0
        self._steering_angle_velocity = 0.0
        self._speed = 0.0
        self._acceleration = 0.0
        self._jerk = 0.0

        self.ads = self.__null_ads

    def _update_ads(self):
        # TODO: Do the actual math, update self.ads accordingly,
        # `_timer_callback()` will take care of publishing.`.
        pass

    def _on_path(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        self._update_ads()
        self.get_logger().debug('Updated ADS: "%s"' % self.ads)

    def _timer_callback(self):
        msg = AckermannDriveStamped(drive=AckermannDrive(steering_angle=0.0, steering_angle_velocity=0.0, speed=0.0, acceleration=0.0, jerk=0.0))
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Controller()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()