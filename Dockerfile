FROM osrf/ros:galactic-desktop


# Install dependencies from apt
RUN apt update && \ 
    apt install -y python3-rosdep ros-galactic-gazebo-ros-pkgs ros-galactic-ackermann-msgs ros-galactic-joint-state-publisher ros-galactic-xacro 

WORKDIR /deps

# Install yaml-cpp from source
RUN git clone https://github.com/jbeder/yaml-cpp.git && \
    cd yaml-cpp && \
    mkdir build && \
    cd build && \
    cmake -DYAML_BUILD_SHARED_LIBS=ON .. && \
    make && \
    make install

WORKDIR /workspace

# Install dependencies from rosdep
RUN git clone https://gitlab.com/eufs/eufs_sim.git && \
    git clone https://gitlab.com/eufs/eufs_msgs.git && \
    export EUFS_MASTER=/workspace && \
    rosdep update && \
    rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y

# Build with colcon
RUN . /opt/ros/galactic/setup.sh && colcon build

# Setup .bashrc
RUN echo 'export EUFS_MASTER=/workspace' >> ~/.bashrc && \
    echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc && \ 
    echo 'source /workspace/install/setup.bash' >> ~/.bashrc

# To finalise, `rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y && colcon build`, 
# `. ./install/setup.bash` and then `ros2 launch eufs_launcher eufs_launcher.launch.py`
