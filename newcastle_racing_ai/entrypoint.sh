#!/bin/bash
source /opt/ros/galactic/setup.bash

cd workspace/MPCC/C++
rm -rf External/
./install.sh
cd /workspace/ft-fsd-path-planning
pip install --no-cache-dir -r requirements.txt

cd /workspace
source /opt/ros/galactic/setup.bash
pip3 install --no-cache-dir pandas matplotlib scipy
rosdep install --from-paths src --ignore-src -r -y

# colcon build --packages-select newcastle_racing_ai newcastle_racing_ai_msgs
colcon build --symlink-install
source install/setup.bash

# launch the nodes
RUN /workspace/launch/all_nodes.launch.py