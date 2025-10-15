ARG ROS_DISTRO=rolling

# Use official ROS2 images
FROM ros:${ROS_DISTRO}

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Set up ROS 2 repository and GPG keys
ARG ROS_APT_SOURCE_VERSION=1.1.0
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" \
    && apt install -y /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Set up ROS2 workspace
WORKDIR /synapticon_ros2_control_ws
RUN git clone https://github.com/synapticon/synapticon_ros2_control src/synapticon_ros2_control

# Modify CMakeLists.txt to suppress warnings
RUN sed -i 's/-Wall -Wextra -Wpedantic/-Wno-pedantic -Wno-error=pedantic/g' src/synapticon_ros2_control/CMakeLists.txt

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && apt-get update -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Set up entrypoint to source the environment
#RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /synapticon_ros2_control_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]
