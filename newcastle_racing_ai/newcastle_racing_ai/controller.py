import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from newcastle_racing_ai_msgs import ControlCommand, PathWithBoundaries
from .parameters import PARAMETERS

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        #self._subscription = self.create_subscription(PoseArray, self.get_parameter("path").value, self._on_path, 10)
        self.create_subscription(PathWithBoundaries, '/path', self._on_path, 10)
        self.create_subscription(Imu, '/nrfai/imu', self.imu_callback, 10)
        #self._publisher = self.create_publisher(AckermannDriveStamped, self.get_parameter("cmd_topic").value, 10)
        self.control_publisher = self.create_publisher(ControlCommand, '/fsds/control_command', 10)
        #self.timer = self.create_timer(timer_period, self._timer_callback)

    def _on_path(self, msg):
        self.get_logger().info('Received: "%s"' % type(msg))
        #control method goes here and calls publish_control()
        #will need to add a live / sim method as the live method will need to write to the can bus. 

    def imu_callback(self, imu_msg):
        _, _, yaw = quaternion_to_euler(imu_msg.orientation)
        self.car_direction = np.array([np.cos(yaw), np.sin(yaw)])
        self.check_and_compute_path()

    def publish_control(self, throttle, steering, brake):
        # float64 throttle # [-] range : (0, 1)
        # float64 steering # [-] range : (-1, 1)
        # float64 brake # # [-] range : (0, 1)
        msg = ControlCommand(throttle=throttle, steering=steering, brake=brake)
        self.control_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Controller()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
