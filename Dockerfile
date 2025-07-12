FROM stereolabs/zed:5.0-devel-cuda12.8-ubuntu22.04

ARG ROS_DISTRO="humble"

# Installing ROS_DISTRO
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository universe && \
    sudo apt update && sudo apt install curl -y && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb" && \
    sudo dpkg -i /tmp/ros2-apt-source.deb

RUN apt update && \
    apt install -y ros-humble-desktop

# Install dependencies from apt
RUN apt update && \ 
    apt install -y \
    libyaml-cpp-dev \
    python3-rosdep \
    ros-dev-tools \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-zed-msgs \
    build-essential \
    cmake \
    git \
    wget 

ENV EUFS_MASTER=/workspace

WORKDIR ${EUFS_MASTER}

# Install dependencies from rosdep
# git clone --depth 1 https://gitlab.com/eufs/eufs_sim.git && \ We are not using this anymore
RUN git clone --depth 1 https://gitlab.com/eufs/eufs_msgs.git && \
    git clone --depth 1 https://github.com/stereolabs/zed-ros2-wrapper.git && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths ${EUFS_MASTER} --ignore-src -r -y

# Build with colcon
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Setup .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \ 
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Build the msg package with colcon
COPY newcastle_racing_ai_msgs newcastle_racing_ai_msgs
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select newcastle_racing_ai_msgs

# Build the node package with colcon
COPY newcastle_racing_ai newcastle_racing_ai
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --packages-select newcastle_racing_ai

ENTRYPOINT ["/bin/bash", "newcastle_racing_ai/entrypoint.sh"]
