import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

class Perception(Node):

    def __init__(self):
        super().__init__('Perception')
        self._subscription = self.create_subscription(ColorRGBA, 'camera', self._on_camera, 10)
        self._publisher = self.create_publisher(PoseArray, 'cones', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_camera(self, msg):
        self.get_logger().info('Received: "%s"' % msg)

    def _timer_callback(self):
        msg = PoseArray(poses=[Pose(position=Point(), orientation=Quaternion())])
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