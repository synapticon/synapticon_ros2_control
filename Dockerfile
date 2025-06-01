ARG ROS_DISTRO=rolling

# Use official ROS2 images
FROM ros:${ROS_DISTRO}

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Set up ROS 2 repository and GPG keys
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
	ros-${ROS_DISTRO}-rviz2
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

