FROM osrf/ros:galactic-desktop


# Install dependencies from apt
RUN apt update && \ 
    apt install -y \
    libyaml-cpp-dev \
    python3-rosdep \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-ackermann-msgs \
    ros-galactic-joint-state-publisher \
    ros-galactic-xacro \
    ros-galactic-rosbridge-server \
    wget 

ENV EUFS_MASTER=/workspace

WORKDIR ${EUFS_MASTER}

# Install dependencies from rosdep
RUN git clone --depth 1 https://gitlab.com/eufs/eufs_sim.git && \
    git clone --depth 1 https://gitlab.com/eufs/eufs_msgs.git && \
    rosdep update && \
    rosdep install --from-paths ${EUFS_MASTER} --ignore-src -r -y

# Build with colcon
RUN . /opt/ros/galactic/setup.sh && colcon build

# Setup .bashrc
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc && \ 
    echo 'source /workspace/install/setup.bash' >> ~/.bashrc

# Build the msg package with colcon
COPY newcastle_racing_ai_msgs newcastle_racing_ai_msgs
RUN . /opt/ros/galactic/setup.sh && colcon build --packages-select newcastle_racing_ai_msgs

# Build the node package with colcon
COPY newcastle_racing_ai newcastle_racing_ai
RUN . /opt/ros/galactic/setup.sh && colcon build --packages-select newcastle_racing_ai

ENTRYPOINT ["/bin/bash", "newcastle_racing_ai/entrypoint.sh"]
