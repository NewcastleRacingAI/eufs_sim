#!/bin/bash
source /opt/ros/galactic/setup.bash

# The following line is needed only if we have made changes AFTER the docker image was built.
# Generally, it is not needed.

# colcon build --packages-select newcastle_racing_ai newcastle_racing_ai_msgs

source /workspace/install/setup.bash
ros2 launch newcastle_racing_ai nra_launch.py "$@"
