================================
Synapticon Package Installation
================================

OPTION 1: Installing from source

   Create a ROS2 workspace:

   .. code:: bash

      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws/src

   Clone the Synapticon package:

   .. code:: bash

      git clone https://github.com/synapticon/synapticon_ros2_control

   After cloning it, you need to set up the ethernet interface. To do so, first execute ``ifconfig`` and remember the interface name.

   After that, replace the ``eno0`` with your ethernet interface in
   ``/home/YOUR_USER/ros2_ws/src/synapticon_ros2_control/src/
   torque_control_executable.cpp``
   and
   ``/home/YOUR_USER/ros2_ws/src/synapticon_ros2_control/description/ros2_control/
   single_dof.ros2_control.xacro``
   and
   ``/home/YOUR_USER/ros2_ws/src/synapticon_ros2_control/description/ros2_control/
   two_dof.ros2_control.xacro``.
   Alternatively, you can do it with commands:

   .. code:: bash

      sed -i "s/eno0/YOUR_ETHERNET_INTERFACE/g" /home/USER/ros2_ws/src/synapticon_ros2_control/src/torque_control_executable.cpp
      sed -i "s/eno0/YOUR_ETHERNET_INTERFACE/g" /home/USER/ros2_ws/src/synapticon_ros2_control/description/ros2_control/single_dof.ros2_control.xacro
      sed -i "s/eno0/YOUR_ETHERNET_INTERFACE/g" /home/USER/ros2_ws/src/synapticon_ros2_control/description/ros2_control/two_dof.ros2_control.xacro

   Install build tools:

   .. code:: bash

      sudo apt install python3-colcon-common-extensions

   Build the package:

   .. code:: bash

      cd ~/ros2_ws
      rosdep install --from-paths src -y --ignore-src
      colcon build

   Additionally, you can source the workspace by adding the following line to the ``/home/USER/.bashrc`` file, but above the line where you sourced the ROS installation (above this line: ``source /opt/ros/humble/setup.bash``):

   .. code:: bash

      source /home/USER/ros2_ws/install/setup.bash

OPTION 2: Binary installation

   If needed, add the ROS repository (this is done only once):

   .. code:: bash

      sudo apt install software-properties-common
      sudo add-apt-repository universe
      sudo apt update

   Install Synapticon package:

   .. code:: bash

      sudo apt install ros-humble-synapticon-ros2-control

   Make sure your rosdep is initialized and updated:

   .. code:: bash

      sudo rosdep init
      rosdep update

   Install its dependencies:

   .. code:: bash

      rosdep install synapticon_ros2_control

   The package will get installed to ``/opt/ros/humble/share/synapticon_ros2_control/``.

VERIFICATION

   To check if the master could be run and if the slaves are found, in the container terminal execute the following. If you installed from source:

   .. code:: bash

      sudo ./home/YOUR_USER/ros2_ws/install/synapticon_ros2_control/bin/torque_control_executable         

   or if you installed using binary installation:

   .. code:: bash

      sudo ./opt/ros/humble/share/synapticon_ros2_control/bin/torque_control_executable

   Before running other scripts, stop this one by CTRL+C (or wait, it will shutdown automatically after a while).

