import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from ackermann_msgs.msg import AckermannDrive

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        self._subscription = self.create_subscription(Vector3, 'next_waypoint', self._on_next_waypoint, 10)
        self._publisher = self.create_publisher(AckermannDrive, 'drive_command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_next_waypoint(self, msg):
        self.get_logger().info('Received: "%s"' % msg)

    def _timer_callback(self):
        msg = AckermannDrive(steering_angle=0.0, steering_angle_velocity=0.0, speed=0.0, acceleration=0.0, jerk=0.0)
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