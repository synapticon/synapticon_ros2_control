ARG ROS_DISTRO=humble

# Use official OSRF ROS2 images
FROM ros:${ROS_DISTRO}

# Set build argument for Ethernet interface
ARG ETHERNET_INTERFACE
ENV ETHERNET_INTERFACE=$ETHERNET_INTERFACE

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# # Update and install necessary packages
# RUN apt-get update && apt-get install -y \
#     software-properties-common \
#     gedit \
#     vim \
#     sed \
#     iproute2 \
#     && apt-add-repository universe \
#     && apt-add-repository restricted \
#     && apt-add-repository multiverse \
#     && apt-get update \
#     && apt-get install -y \
#     git \
#     python3-colcon-common-extensions \
#     ros-humble-ros2-control \
#     ros-humble-controller-manager \
#     ros-humble-xacro \
#     ros-humble-ros2-controllers \
#     && rm -rf /var/lib/apt/lists/*

# Set up ROS2 workspace
WORKDIR /synapticon_ros2_control_ws
RUN git clone https://github.com/synapticon/synapticon_ros2_control src/synapticon_ros2_control

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && apt-get update -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Set up entrypoint to source the environment
RUN echo "source /synapticon_ros2_control_ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]

