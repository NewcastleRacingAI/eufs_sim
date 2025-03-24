import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, PoseArray

class Planner(Node):

    def __init__(self):
        super().__init__('Planner')
        self._subscription = self.create_subscription(PoseArray, 'cones', self._on_next_waypoint, 10)
        self._publisher = self.create_publisher(Vector3, 'next_waypoint', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_next_waypoint(self, msg):
        self.get_logger().info('Received: "%s"' % msg)

    def _timer_callback(self):
        msg = Vector3(x=0.0, y=0.0, z=0.0)
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Planner()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()