==================
ROS2 Installation
==================

To make sure that locale supports UTF-8, run the following
commands:

.. code:: bash

   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

Enable Ubuntu Universe repository:

.. code:: bash

   sudo apt install software-properties-common
   sudo add-apt-repository universe

Add GPG key:

.. code:: bash

   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

Add repository to the sources:

.. code:: bash

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Update apt cache:

.. code:: bash

   sudo apt update

This command is for updating the packages on your system and if
the commands after it will work, we recommend skipping it:

.. code:: bash

   sudo apt upgrade

Finally, install ROS and compilers:

.. code:: bash

   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools

After the installation is complete, add the following line to
the end of ``/home/USER/.bashrc`` file:

.. code:: bash

   source /opt/ros/humble/setup.bash

In order for ROS2 not to interfere with communication on other
ports, we need to set Domain ID (detailed information is
available
`here <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html>`__).
In our case, we just used 1. To do so, add the following at the
end of ``/home/YOUR_USER/.bashrc``

.. code:: bash

   export ROS_DOMAIN_ID=1

After this, restart all your terminals for the source command
to be active. To verify the installation, open two terminals
and run:

.. code:: bash

   ros2 run demo_nodes_cpp talker
   ros2 run demo_nodes_py listener

If the nodes are communicating, the installation was
successful.

