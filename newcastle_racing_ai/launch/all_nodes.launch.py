from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    workspace_dir = os.getenv('WORKSPACE_DIR', '/workspace')

    return LaunchDescription([
        # Path planning node (as a process)
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/ft-fsd-path-planning/bin/path_planning_node')],
            output='screen'
        ),

        # # MPCC control node (as a ROS 2 node)
        # Node(
        #     package='mpcc_control',
        #     executable='mpcc_control_node',
        #     output='screen'
        # ),

        # Newcastle Racing AI launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(workspace_dir, 'newcastle_racing_ai/launch/nra_launch.py')
            )
        ),
    ])