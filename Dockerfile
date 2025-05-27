FROM osrf/ros:galactic-desktop


# Install dependencies from apt
RUN apt update && \ 
    apt install -y wget python3-rosdep ros-galactic-gazebo-ros-pkgs ros-galactic-ackermann-msgs ros-galactic-joint-state-publisher ros-galactic-xacro 

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

# THIS PART WILL BE UNCOMMENTED WHEN WE HAVE FINALIZED THE PACKAGE
# COPY newcastle_racing_ai newcastle_racing_ai
# COPY newcastle_racing_ai_msgs newcastle_racing_ai_msgs

# Build our package with colcon
# RUN . /opt/ros/galactic/setup.sh && colcon build --packages-select newcastle_racing_ai
