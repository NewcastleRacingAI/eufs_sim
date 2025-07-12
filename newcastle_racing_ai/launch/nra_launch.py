import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EqualsSubstitution, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory



PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "nrfai"


def generate_launch_description():
    node_parameters = [
        {
            "camera_topic": LaunchConfiguration("camera_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "cones_topic": LaunchConfiguration("cones_topic"),
            "path_topic": LaunchConfiguration("path_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "control_topic": LaunchConfiguration("control_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "track_topic": LaunchConfiguration("track_topic"),
            "reset_topic": LaunchConfiguration("reset_topic"),
            "time_step": LaunchConfiguration("time_step"),
        }
    ]
    mode = LaunchConfiguration("mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value=TextSubstitution(text="camera")),
            DeclareLaunchArgument("imu_topic", default_value=TextSubstitution(text="imu")),
            DeclareLaunchArgument("lidar_topic", default_value=TextSubstitution(text="lidar")),
            DeclareLaunchArgument("cones_topic", default_value=TextSubstitution(text="cones")),
            DeclareLaunchArgument("path_topic", default_value=TextSubstitution(text="path")),
            DeclareLaunchArgument("depth_topic", default_value=TextSubstitution(text="depth")),
            DeclareLaunchArgument("control_topic", default_value=TextSubstitution(text="control")),
            DeclareLaunchArgument("odom_topic", default_value=TextSubstitution(text="odom")),
            DeclareLaunchArgument("track_topic", default_value=TextSubstitution(text="track")),
            DeclareLaunchArgument("reset_topic", default_value=TextSubstitution(text="reset")),
            DeclareLaunchArgument("time_step", default_value=TextSubstitution(text="5.0")),
            DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"]),

            # Ros bridge used to collect sensor data from the AIRsim simulator
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"
                    )
                ),
                launch_arguments={
                    "default_call_service_timeout": "5.0",
                    "call_services_in_new_thread": "True",
                    "send_action_goals_in_new_thread": "True",
                }.items(),
                condition=IfCondition(EqualsSubstitution(mode, "sim")), # Enabled if mode==sim
            ),
            # ZED camera wrapper used to collect data from the real camera. It must be connected via USB
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]),
                launch_arguments={
                    'camera_model': 'zed',
                }.items(),
                condition=IfCondition(EqualsSubstitution(mode, "real")), # Enabled if mode==real
            ),

            Node(
                package=PACKAGE_NAME,
                namespace=NAMESPACE,
                executable="controller",
                name="controller",
                parameters=node_parameters,
            ),
            Node(
                package=PACKAGE_NAME,
                namespace=NAMESPACE,
                executable="perception",
                name="perception",
                parameters=node_parameters,
            ),
            Node(
                package=PACKAGE_NAME,
                namespace=NAMESPACE,
                executable="planner",
                name="planner",
                parameters=node_parameters,
            ),
        ]
    )
