import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

PACKAGE_NAME = "newcastle_racing_ai"
NAMESPACE = "car"

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('eufs_launcher'),
                    'eufs_launcher.launch.py'))
        ),
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            executable='controller',
            name='controller'
        ),
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            executable='perception',
            name='perception'
        ),
        Node(
            package=PACKAGE_NAME,
            namespace=NAMESPACE,
            executable='planner',
            name='planner',
        )
    ])