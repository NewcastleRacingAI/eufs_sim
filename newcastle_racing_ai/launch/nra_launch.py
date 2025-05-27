import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution

PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "car"


def generate_launch_description():
    node_parameters = [
        {
            "camera_topic": LaunchConfiguration("camera_topic"),
            "cones_topic": LaunchConfiguration("cones_topic"),
            "path_topic": LaunchConfiguration("path_topic"),
            "cmd_topic": LaunchConfiguration("cmd_topic"),
            "time_step": LaunchConfiguration("time_step"),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value=TextSubstitution(text="/camera")),
            DeclareLaunchArgument("cones_topic", default_value=TextSubstitution(text="/cones")),
            DeclareLaunchArgument("path_topic", default_value=TextSubstitution(text="/path")),
            DeclareLaunchArgument("cmd_topic", default_value=TextSubstitution(text="/cmd")),
            DeclareLaunchArgument("time_step", default_value=TextSubstitution(text="0.0")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("eufs_launcher"), "eufs_launcher.launch.py")
                )
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml")
                )
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
