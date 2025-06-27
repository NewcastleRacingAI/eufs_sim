import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from newcastle_racing_ai_msgs.msg import Track
from .parameters import PARAMETERS


class Perception(Node):

    def __init__(self):
        super().__init__("Perception")
        self._img_counter = 0
        self._depth_counter = 0
        self.declare_parameters(namespace="", parameters=PARAMETERS)
        #####################
        # Subscribers
        #####################
        self._subscription = self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self._on_camera, 10
        )
        self._subscription = self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._on_depth, 10
        )
        self._subscription = self.create_subscription(
            PointCloud2, self.get_parameter("lidar_topic").value, self._on_lidar, 10
        )
        self._subscription = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10
        )
        self._subscription = self.create_subscription(
            Track, self.get_parameter("track_topic").value, self._on_track, 10
        )
        self._subscription = self.create_subscription(Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)
        #####################
        # Publishers
        #####################
        self._publisher = self.create_publisher(ConeArrayWithCovariance, self.get_parameter("cones_topic").value, 10)
        #####################
        # Timer
        #####################
        # self.timer = self.create_timer(self.get_parameter("time_step").value, self._timer_callback)

    def _on_camera(self, msg: Image):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process CAMERA data here if needed. For example, saving the image in ppm format.
        self._img_counter += 1
        self.save_image_ppm(msg, filename=f"camera-{self._img_counter}.ppm")

    def _on_depth(self, msg: Image):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process DEPTH data here if needed. For example, saving the depth image in ppm format.
        self._depth_counter += 1
        self.save_image_ppm(msg, filename=f"depth-{self._depth_counter}.ppm")

    def _on_odom(self, msg: Odometry):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process ODOMETRY data here if needed
        # For example, you can log the position and orientation of the vehicle
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f"Position: {position}, Orientation: {orientation}")
        self.get_logger().info(
            f"Linear Velocity: {msg.twist.twist.linear}, Angular Velocity: {msg.twist.twist.angular}"
        )

    def _on_imu(self, msg: Imu):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process IMU data here if needed

    def _on_lidar(self, msg: PointCloud2):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process LIDAR data here if needed

    def _on_track(self, msg: Track):
        self.get_logger().info('Received: "%s"' % type(msg))
        # Process TRACK data here if needed
        # For example, you can log the cones detected in the track
        self.get_logger().info(f"{len(msg.cones)} cones detected")

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

    def save_image_ppm(self, msg: Image, filename: str, path: str = "/workspace/newcastle_racing_ai/imgs/"):
        # This function is a placeholder for saving the image in ppm format.
        self.get_logger().info("Saving image in ppm format.")
        if msg.encoding not in ("rgb8", "bgr8", "mono8"):
            self.get_logger().warn("Can only handle mono8, rgb8 or bgr8 encoding.")
            return

        # Check that the path to the image directory exists
        os.makedirs(path, exist_ok=True)

        with open(os.path.join(path, filename), "w", encoding="utf-8") as file:
            file.write("P3\n")
            file.write(f"{msg.width} {msg.height}\n")
            file.write("255\n")

            for y in range(msg.height):
                for x in range(msg.width):
                    # Get indices for the pixel components
                    first_byte_idx = y * msg.step + 3 * x
                    green_byte_idx = first_byte_idx + 1
                    last_byte_idx = first_byte_idx + 2

                    if msg.encoding == "rgb8":
                        red_byte_idx = first_byte_idx
                        blue_byte_idx = last_byte_idx
                    elif msg.encoding == "bgr8":
                        red_byte_idx = last_byte_idx
                        blue_byte_idx = first_byte_idx
                    elif msg.encoding == "mono8":
                        red_byte_idx = green_byte_idx = blue_byte_idx = y * msg.step + x
                    else:
                        self.get_logger().warn("Can only handle rgb8 or bgr8 encoding.")
                        return

                    file.write(f"{msg.data[red_byte_idx]} {msg.data[green_byte_idx]} {msg.data[blue_byte_idx]} ")
                file.write("\n")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Perception()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
